/*
 * SigmaStudioFW.c
 *
 * SigmaStudio互換のSPI通信API（void署名維持）と、結果付きAPI（sigma_spi.h）の実装。
 *
 * - 一回の転送はmutexで排他し、IT転送はcallbackが保存する結果だけで成否を判定する。
 * - エラー・タイムアウト時はHAL_SPI_Abort()で停止を確認し、確認できない場合は
 *   SPI5の割り込み源を封じてFAULTへラッチする。FAULT中は静的バッファへ触れる前に拒否する。
 * - スケジューラ開始前はRTOS待機を行わない。単一呼出し前提のポーリングで転送する。
 * - ISR（callback）では結果保存と通知だけを行い、printf・停止待ち・mutex操作は行わない。
 */

#include "SigmaStudioFW.h"

#include "sigma_spi.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "task.h"

#include <stdbool.h>

extern osMutexId_t spiMutexHandle;
extern osSemaphoreId_t spiTxBinarySemHandle;
extern osSemaphoreId_t spiTxRxBinarySemHandle;

enum
{
    SIGMA_SPI_HEADER_SIZE          = 3U,
    SIGMA_SPI_BLOCK_TX_BUFFER_SIZE = SIGMA_SPI_HEADER_SIZE + SIGMA_WRITE_BLOCK_MAX_PAYLOAD,
    SIGMA_SPI_MEMORY_WORD_SIZE     = 4U,
    SIGMA_SPI_BLOCK_CHUNK_SIZE     = SIGMA_WRITE_BLOCK_MAX_PAYLOAD & ~(SIGMA_SPI_MEMORY_WORD_SIZE - 1U),
    SIGMA_SPI_MUTEX_TIMEOUT_MS     = 200U,
    SIGMA_SPI_IT_TIMEOUT_MS        = 100U,
    SIGMA_SPI_POLL_TIMEOUT_MS      = 100U,
    SIGMA_SPI_IT_TX_BUFFER_SIZE    = 16U,
    SIGMA_SPI_SAFELOAD_BUFFER_SIZE = 64U,
    SIGMA_SPI_READ_BUFFER_SIZE     = 64U,
    SIGMA_SPI_MAX_HAL_TRANSFER     = 0xFFFFU,
};

_Static_assert(SIGMA_SPI_BLOCK_CHUNK_SIZE > 0U, "SigmaDSP SPI block chunk must contain at least one word");
_Static_assert(SIGMA_SPI_BLOCK_CHUNK_SIZE + SIGMA_SPI_HEADER_SIZE <= SIGMA_SPI_BLOCK_TX_BUFFER_SIZE,
               "SigmaDSP SPI block buffer must hold header and one chunk");
_Static_assert(SIGMA_SPI_IT_TX_BUFFER_SIZE > SIGMA_SPI_HEADER_SIZE,
               "SigmaDSP SPI IT buffer must hold header and payload");
_Static_assert(SIGMA_SPI_SAFELOAD_BUFFER_SIZE > SIGMA_SPI_HEADER_SIZE,
               "SigmaDSP SPI safeload buffer must hold header and payload");
_Static_assert(SIGMA_SPI_READ_BUFFER_SIZE > SIGMA_SPI_HEADER_SIZE,
               "SigmaDSP SPI read buffer must hold header and payload");

// 静的バッファ（転送中にスコープ外にならないようにするため）
static uint8_t spi_block_tx_buf[SIGMA_SPI_BLOCK_TX_BUFFER_SIZE];
static uint8_t spi_it_tx_buf[SIGMA_SPI_IT_TX_BUFFER_SIZE];
static uint8_t spi_safeload_buf[SIGMA_SPI_SAFELOAD_BUFFER_SIZE];
static uint8_t spi_read_tx_buf[SIGMA_SPI_READ_BUFFER_SIZE];
static uint8_t spi_read_rx_buf[SIGMA_SPI_READ_BUFFER_SIZE];

volatile uint32_t sigma_spi_it_write_calls          = 0;
volatile uint32_t sigma_spi_it_write_errors         = 0;
volatile uint32_t sigma_spi_it_write_timeouts       = 0;
volatile uint32_t sigma_spi_it_mutex_timeouts       = 0;

volatile sigma_spi_diagnostics_t g_sigma_spi_diagnostics = {0U};

// IT転送の状態。taskとISRが短いcritical sectionで共有する。
typedef enum
{
    SIGMA_SPI_STATE_IDLE = 0,
    SIGMA_SPI_STATE_PENDING_TX,
    SIGMA_SPI_STATE_PENDING_TXRX,
    SIGMA_SPI_STATE_COMPLETED,
    SIGMA_SPI_STATE_ERROR,
    SIGMA_SPI_STATE_STOPPING,
    SIGMA_SPI_STATE_FAULT,
} sigma_spi_transfer_state_t;

typedef enum
{
    SIGMA_SPI_XFER_TX = 0,
    SIGMA_SPI_XFER_TXRX,
} sigma_spi_xfer_kind_t;

static volatile sigma_spi_transfer_state_t s_transfer_state = SIGMA_SPI_STATE_IDLE;
static volatile uint32_t s_transfer_hal_error = 0U;
static volatile bool s_channel_faulted = false;

// 永続診断の更新。並行する呼出し（mutex解放後の記録や拒否経路）でカウンタの
// read-modify-writeとlast_*の組が混ざらないよう、記録全体を短いIRQ禁止区間で
// 更新する。本関数はISRから呼ばれない。
// detailが非NULLなら、この呼出し固有の結果を呼出側へ直接返す（後読み依存を防ぐ）。
static void sigma_spi_diag_note(sigma_spi_result_t result, sigma_spi_op_t op, uint32_t address,
                                uint32_t hal_status, uint32_t hal_error, uint32_t abort_status,
                                sigma_spi_call_detail_t* detail)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    switch (result)
    {
        case SIGMA_SPI_RESULT_OK:              g_sigma_spi_diagnostics.ok_count++; break;
        case SIGMA_SPI_RESULT_INVALID_ARG:     g_sigma_spi_diagnostics.invalid_arg_count++; break;
        case SIGMA_SPI_RESULT_NOT_INITIALIZED: g_sigma_spi_diagnostics.not_initialized_count++; break;
        case SIGMA_SPI_RESULT_MUTEX_TIMEOUT:   g_sigma_spi_diagnostics.mutex_timeout_count++; break;
        case SIGMA_SPI_RESULT_START_FAILED:    g_sigma_spi_diagnostics.start_failed_count++; break;
        case SIGMA_SPI_RESULT_TRANSFER_ERROR:  g_sigma_spi_diagnostics.transfer_error_count++; break;
        case SIGMA_SPI_RESULT_TIMEOUT:         g_sigma_spi_diagnostics.transfer_timeout_count++; break;
        case SIGMA_SPI_RESULT_STOP_FAILED:     g_sigma_spi_diagnostics.stop_failed_count++; break;
        case SIGMA_SPI_RESULT_FAULT:           g_sigma_spi_diagnostics.fault_reject_count++; break;
        default: break;
    }

    g_sigma_spi_diagnostics.last_result       = (uint32_t) result;
    g_sigma_spi_diagnostics.last_operation    = (uint32_t) op;
    g_sigma_spi_diagnostics.last_address      = address;
    g_sigma_spi_diagnostics.last_hal_status   = hal_status;
    g_sigma_spi_diagnostics.last_hal_error    = hal_error;
    g_sigma_spi_diagnostics.last_abort_status = abort_status;

    __set_PRIMASK(primask);

    if (detail != NULL)
    {
        detail->result       = result;
        detail->hal_status   = hal_status;
        detail->hal_error    = hal_error;
        detail->abort_status = abort_status;
    }
}

static uint32_t sigma_spi_transfer_hal_error_get(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t hal_error = s_transfer_hal_error;
    __set_PRIMASK(primask);
    return hal_error;
}

static void sigma_spi_transfer_begin_shared(sigma_spi_transfer_state_t pending_state)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_transfer_hal_error = 0U;
    s_transfer_state = pending_state;
    __set_PRIMASK(primask);
    __DMB();
}

static bool sigma_spi_runtime(void)
{
    return (osKernelGetState() == osKernelRunning);
}

static bool sigma_spi_sync_objects_ready(void)
{
    return (spiMutexHandle != NULL) && (spiTxBinarySemHandle != NULL) && (spiTxRxBinarySemHandle != NULL);
}

// 要求が16bitアドレス空間を越えないか検証する。DSP側の自動インクリメントが
// 範囲をwrapしないよう、lengthをword数（切り上げ）へ換算して判定する。
static bool sigma_spi_address_range_valid(uint16_t address, uint32_t length)
{
    const uint64_t total_words =
        ((uint64_t) length + (SIGMA_SPI_MEMORY_WORD_SIZE - 1U)) / SIGMA_SPI_MEMORY_WORD_SIZE;
    return ((uint64_t) address + total_words) <= 0x10000ULL;
}

static void sigma_spi_drain_notifications(void)
{
    if (spiTxBinarySemHandle != NULL)
    {
        while (osSemaphoreAcquire(spiTxBinarySemHandle, 0U) == osOK)
        {
        }
    }
    if (spiTxRxBinarySemHandle != NULL)
    {
        while (osSemaphoreAcquire(spiTxRxBinarySemHandle, 0U) == osOK)
        {
        }
    }
}

// 進行中の可能性がある転送を停止し、再始動可能な状態を確認する。呼出側がmutexを保持する。
// 確認できない場合はSPI5の割り込み源を封じてFAULTへラッチし、STOP_FAILEDを返す。
static sigma_spi_result_t sigma_spi_stop_locked(uint32_t* hal_error_out, uint32_t* abort_status_out)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_transfer_state != SIGMA_SPI_STATE_FAULT)
    {
        s_transfer_state = SIGMA_SPI_STATE_STOPPING;
    }
    __set_PRIMASK(primask);
    __DMB();

    // HAL_SPI_Abortは成功時にErrorCodeを消去するため、元のErrorCodeを先に保存する。
    const uint32_t hal_error = (s_transfer_hal_error != 0U) ? s_transfer_hal_error : hspi5.ErrorCode;
    const HAL_StatusTypeDef abort_status = HAL_SPI_Abort(&hspi5);

    // 戻り値に加え、SPI無効・割り込み無効まで確認する。Stateは失敗時もREADYに
    // なり得るため、Stateだけでは停止成功と判定しない。DXPはHALのエラー処理が
    // 無効化しないため、確認対象に含める。
    const bool stopped = (abort_status == HAL_OK) &&
                         (hspi5.State == HAL_SPI_STATE_READY) &&
                         ((hspi5.Instance->CR1 & (SPI_CR1_SPE | SPI_CR1_CSTART)) == 0U) &&
                         ((hspi5.Instance->IER & (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
                                                  SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF)) == 0U);

    if (hal_error_out != NULL)
    {
        *hal_error_out = hal_error;
    }
    if (abort_status_out != NULL)
    {
        *abort_status_out = (uint32_t) abort_status;
    }

    if (!stopped)
    {
        // 停止を確定できない。旧callbackがバッファへアクセスしないよう、SPI5の
        // 割り込み源だけを封じる（他経路のIRQは触らない）。FAULT解除は再起動を基本とする。
        HAL_NVIC_DisableIRQ(SPI5_IRQn);
        HAL_NVIC_ClearPendingIRQ(SPI5_IRQn);
        s_channel_faulted = true;
        s_transfer_state = SIGMA_SPI_STATE_FAULT;
        __DMB();
        return SIGMA_SPI_RESULT_STOP_FAILED;
    }

    // 停止確認済み。残存通知とpending IRQを掃除してからバッファ再利用を許可する。
    sigma_spi_drain_notifications();
    HAL_NVIC_ClearPendingIRQ(SPI5_IRQn);
    s_transfer_hal_error = 0U;
    s_transfer_state = SIGMA_SPI_STATE_IDLE;
    __DMB();
    return SIGMA_SPI_RESULT_OK;
}

// IT転送（呼出側がmutex保持）。開始直後にcallbackが先行しても結果を取りこぼさない。
static sigma_spi_result_t sigma_spi_transfer_it(sigma_spi_xfer_kind_t kind, const uint8_t* tx, uint8_t* rx,
                                                uint16_t size, uint16_t timeout_ms,
                                                uint32_t* hal_status_out,
                                                uint32_t* hal_error_out, uint32_t* abort_status_out)
{
    osSemaphoreId_t semaphore = (kind == SIGMA_SPI_XFER_TX) ? spiTxBinarySemHandle : spiTxRxBinarySemHandle;
    const sigma_spi_transfer_state_t pending_state =
        (kind == SIGMA_SPI_XFER_TX) ? SIGMA_SPI_STATE_PENDING_TX : SIGMA_SPI_STATE_PENDING_TXRX;

    if (semaphore == NULL)
    {
        // 呼出側の事前確認に依存しない防御。静的バッファへは触れない。
        return SIGMA_SPI_RESULT_NOT_INITIALIZED;
    }

    // 古い通知を掃除し、結果・対象転送を初期化してからHAL開始APIを呼ぶ。
    sigma_spi_drain_notifications();
    sigma_spi_transfer_begin_shared(pending_state);

    const HAL_StatusTypeDef status = (kind == SIGMA_SPI_XFER_TX) ?
        HAL_SPI_Transmit_IT(&hspi5, (uint8_t*) tx, size) :
        HAL_SPI_TransmitReceive_IT(&hspi5, (uint8_t*) tx, rx, size);
    if (hal_status_out != NULL)
    {
        *hal_status_out = (uint32_t) status;
    }

    if (status != HAL_OK)
    {
        // BUSY等では旧転送が残っている可能性があるため、再利用前に停止を確認する。
        const sigma_spi_result_t stop_result = sigma_spi_stop_locked(hal_error_out, abort_status_out);
        if (stop_result != SIGMA_SPI_RESULT_OK)
        {
            return stop_result;
        }
        return SIGMA_SPI_RESULT_START_FAILED;
    }

    (void) osSemaphoreAcquire(semaphore, pdMS_TO_TICKS(timeout_ms));

    // 通知の有無ではなく、callbackが保存した結果で判定する。期限切れ後の最終確認と
    // STOPPINGへの遷移はcallbackと原子的に行い、実完了（COMPLETED）をSTOPPINGで
    // 上書きしてTIMEOUTと誤判定しないようにする。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const sigma_spi_transfer_state_t final_state = s_transfer_state;
    if ((final_state != SIGMA_SPI_STATE_COMPLETED) && (final_state != SIGMA_SPI_STATE_ERROR))
    {
        s_transfer_state = SIGMA_SPI_STATE_STOPPING;
    }
    __set_PRIMASK(primask);
    __DMB();

    if (final_state == SIGMA_SPI_STATE_COMPLETED)
    {
        // 期限と完了callbackの競合。完了として扱う（HAL成功時は全IT無効化済み）。
        return SIGMA_SPI_RESULT_OK;
    }

    if (final_state == SIGMA_SPI_STATE_ERROR)
    {
        if (hal_error_out != NULL)
        {
            *hal_error_out = sigma_spi_transfer_hal_error_get();
        }
    }

    // 未完了またはエラーの場合は停止を確認してから再利用可能とする。HALのエラー
    // 処理はDXP割り込みを無効化しないため、共通停止でSPI・全ITの無効を確定する。
    const sigma_spi_result_t stop_result = sigma_spi_stop_locked(hal_error_out, abort_status_out);
    if (stop_result != SIGMA_SPI_RESULT_OK)
    {
        return stop_result;
    }
    if (final_state == SIGMA_SPI_STATE_ERROR)
    {
        return SIGMA_SPI_RESULT_TRANSFER_ERROR;
    }
    return SIGMA_SPI_RESULT_TIMEOUT;
}

// ポーリング転送（呼出側がmutex保持、スケジューラ前はlockなし）。停止判定は共通化する。
static sigma_spi_result_t sigma_spi_transfer_poll(const uint8_t* tx, uint16_t size,
                                                  uint32_t* hal_status_out,
                                                  uint32_t* hal_error_out, uint32_t* abort_status_out)
{
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi5, (uint8_t*) tx, size, SIGMA_SPI_POLL_TIMEOUT_MS);
    if (hal_status_out != NULL)
    {
        *hal_status_out = (uint32_t) status;
    }
    if (status == HAL_OK)
    {
        return SIGMA_SPI_RESULT_OK;
    }

    const sigma_spi_result_t stop_result = sigma_spi_stop_locked(hal_error_out, abort_status_out);
    if (stop_result != SIGMA_SPI_RESULT_OK)
    {
        return stop_result;
    }
    // HAL_BUSYは転送開始前の失敗（READY以外の周辺状態）として区別する。
    if (status == HAL_BUSY)
    {
        return SIGMA_SPI_RESULT_START_FAILED;
    }
    // HAL_TIMEOUTは転送完了待ちの期限切れとして区別する。
    if (status == HAL_TIMEOUT)
    {
        return SIGMA_SPI_RESULT_TIMEOUT;
    }
    return SIGMA_SPI_RESULT_TRANSFER_ERROR;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi != &hspi5)
    {
        return;
    }
    if (s_transfer_state != SIGMA_SPI_STATE_PENDING_TX)
    {
        // 対象外・旧転送のcallbackは結果として採用しない。
        return;
    }

    s_transfer_state = SIGMA_SPI_STATE_COMPLETED;
    __DMB();
    osSemaphoreRelease(spiTxBinarySemHandle);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi != &hspi5)
    {
        return;
    }
    if (s_transfer_state != SIGMA_SPI_STATE_PENDING_TXRX)
    {
        return;
    }

    s_transfer_state = SIGMA_SPI_STATE_COMPLETED;
    __DMB();
    osSemaphoreRelease(spiTxRxBinarySemHandle);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi != &hspi5)
    {
        return;
    }

    const sigma_spi_transfer_state_t state = s_transfer_state;
    if ((state != SIGMA_SPI_STATE_PENDING_TX) && (state != SIGMA_SPI_STATE_PENDING_TXRX))
    {
        return;
    }

    // 結果と元のErrorCodeを保存してから、該当する待機先だけを通知する。
    s_transfer_hal_error = hspi->ErrorCode;
    s_transfer_state = SIGMA_SPI_STATE_ERROR;
    __DMB();

    if (state == SIGMA_SPI_STATE_PENDING_TX)
    {
        osSemaphoreRelease(spiTxBinarySemHandle);
    }
    else
    {
        osSemaphoreRelease(spiTxRxBinarySemHandle);
    }
}

sigma_spi_result_t sigma_spi_write_block(uint8_t devAddress, uint16_t address, uint32_t length, uint8_t* pData,
                                         sigma_spi_call_detail_t* detail)
{
    uint32_t hal_status = 0U;
    uint32_t hal_error = 0U;
    uint32_t abort_status = 0U;

    // 単一フレームの上限はヘッダー込みstorage（SIGMA_WRITE_BLOCK_MAX_PAYLOAD）で検証する。
    // word整列はchunk分割が必要な場合だけ要求し、既存のchunk方式を維持する。
    if ((pData == NULL) || (length == 0U) ||
        ((length > SIGMA_WRITE_BLOCK_MAX_PAYLOAD) && ((length % SIGMA_SPI_MEMORY_WORD_SIZE) != 0U)))
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_INVALID_ARG, SIGMA_SPI_OP_WRITE_BLOCK_POLL,
                            address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_INVALID_ARG;
    }

    // 要求全体の16bitアドレス範囲を最初の転送前に検証する。範囲外なら一度も送信せず、
    // DSPメモリを部分的に書き換えない。
    if (!sigma_spi_address_range_valid(address, length))
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_INVALID_ARG, SIGMA_SPI_OP_WRITE_BLOCK_POLL,
                            address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_INVALID_ARG;
    }

    if (s_channel_faulted)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_WRITE_BLOCK_POLL, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    const bool runtime = sigma_spi_runtime();
    if (runtime)
    {
        // スケジューラ実行中に同期オブジェクトが無い場合、排他なしポーリングへは遷移しない。
        if (!sigma_spi_sync_objects_ready())
        {
            sigma_spi_diag_note(SIGMA_SPI_RESULT_NOT_INITIALIZED, SIGMA_SPI_OP_WRITE_BLOCK_POLL,
                                address, 0U, 0U, 0U, detail);
            return SIGMA_SPI_RESULT_NOT_INITIALIZED;
        }
        if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(SIGMA_SPI_MUTEX_TIMEOUT_MS)) != osOK)
        {
            sigma_spi_diag_note(SIGMA_SPI_RESULT_MUTEX_TIMEOUT, SIGMA_SPI_OP_WRITE_BLOCK_POLL,
                                address, 0U, 0U, 0U, detail);
            return SIGMA_SPI_RESULT_MUTEX_TIMEOUT;
        }

        // 待機中に別要求が停止失敗でFAULTへラッチした場合、バッファ操作前に拒否する。
        if (s_channel_faulted)
        {
            osMutexRelease(spiMutexHandle);
            sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_WRITE_BLOCK_POLL,
                                address, 0U, 0U, 0U, detail);
            return SIGMA_SPI_RESULT_FAULT;
        }
    }

    sigma_spi_result_t result = SIGMA_SPI_RESULT_OK;
    uint32_t data_offset = 0U;
    uint32_t remaining = length;
    uint16_t current_address = address;

    while (remaining > 0U)
    {
        const uint16_t chunk_length = (uint16_t) ((remaining > SIGMA_SPI_BLOCK_CHUNK_SIZE)
                                                      ? SIGMA_SPI_BLOCK_CHUNK_SIZE
                                                      : remaining);

        spi_block_tx_buf[0] = devAddress;
        spi_block_tx_buf[1] = (uint8_t) ((current_address >> 8) & 0x00FF);
        spi_block_tx_buf[2] = (uint8_t) (current_address & 0x00FF);
        for (uint16_t i = 0; i < chunk_length; i++)
        {
            spi_block_tx_buf[i + SIGMA_SPI_HEADER_SIZE] = pData[data_offset + i];
        }

        result = sigma_spi_transfer_poll(spi_block_tx_buf,
                                         (uint16_t) (SIGMA_SPI_HEADER_SIZE + chunk_length),
                                         &hal_status, &hal_error, &abort_status);
        if (result != SIGMA_SPI_RESULT_OK)
        {
            // 途中chunkの失敗では残りの書込みを中止する。
            break;
        }

        remaining -= chunk_length;
        data_offset += chunk_length;
        if (remaining > 0U)
        {
            const uint32_t chunk_words =
                ((uint32_t) chunk_length + (SIGMA_SPI_MEMORY_WORD_SIZE - 1U)) / SIGMA_SPI_MEMORY_WORD_SIZE;
            current_address = (uint16_t) ((uint32_t) current_address + chunk_words);
        }
    }

    if (runtime)
    {
        osMutexRelease(spiMutexHandle);
    }

    sigma_spi_diag_note(result, SIGMA_SPI_OP_WRITE_BLOCK_POLL, address, hal_status, hal_error, abort_status, detail);
    return result;
}

void SIGMA_WRITE_REGISTER_BLOCK(uint8_t devAddress, uint16_t address, uint32_t length, uint8_t* pData)
{
    (void) sigma_spi_write_block(devAddress, address, length, pData, NULL);
}

sigma_spi_result_t sigma_spi_write_block_it(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData,
                                            sigma_spi_call_detail_t* detail)
{
    uint32_t hal_status = 0U;
    uint32_t hal_error = 0U;
    uint32_t abort_status = 0U;

    sigma_spi_it_write_calls++;

    if ((pData == NULL) || (length == 0U) ||
        (length > (SIGMA_SPI_IT_TX_BUFFER_SIZE - SIGMA_SPI_HEADER_SIZE)) ||
        (((uint32_t) SIGMA_SPI_HEADER_SIZE + (uint32_t) length) > SIGMA_SPI_MAX_HAL_TRANSFER) ||
        !sigma_spi_address_range_valid(address, length))
    {
        sigma_spi_it_write_errors++;
        sigma_spi_diag_note(SIGMA_SPI_RESULT_INVALID_ARG, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_INVALID_ARG;
    }
    if (s_channel_faulted)
    {
        sigma_spi_it_write_errors++;
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }
    if (!sigma_spi_runtime() || !sigma_spi_sync_objects_ready())
    {
        // スケジューラ実行中の未初期化を、排他なしポーリングへfallbackしない。
        sigma_spi_it_write_errors++;
        sigma_spi_diag_note(SIGMA_SPI_RESULT_NOT_INITIALIZED, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_NOT_INITIALIZED;
    }

    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(SIGMA_SPI_MUTEX_TIMEOUT_MS)) != osOK)
    {
        sigma_spi_it_mutex_timeouts++;
        sigma_spi_diag_note(SIGMA_SPI_RESULT_MUTEX_TIMEOUT, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_MUTEX_TIMEOUT;
    }

    // 待機中に別要求が停止失敗でFAULTへラッチした場合、バッファ操作前に拒否する。
    if (s_channel_faulted)
    {
        osMutexRelease(spiMutexHandle);
        sigma_spi_it_write_errors++;
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    spi_it_tx_buf[0] = devAddress;
    spi_it_tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
    spi_it_tx_buf[2] = (uint8_t) (address & 0x00FF);
    for (uint16_t i = 0; i < length; i++)
    {
        spi_it_tx_buf[i + SIGMA_SPI_HEADER_SIZE] = pData[i];
    }

    const sigma_spi_result_t result = sigma_spi_transfer_it(SIGMA_SPI_XFER_TX,
                                                            spi_it_tx_buf, NULL,
                                                            (uint16_t) (SIGMA_SPI_HEADER_SIZE + length),
                                                            SIGMA_SPI_IT_TIMEOUT_MS,
                                                            &hal_status, &hal_error, &abort_status);

    osMutexRelease(spiMutexHandle);

    if (result == SIGMA_SPI_RESULT_TIMEOUT)
    {
        sigma_spi_it_write_timeouts++;
    }
    else if (result != SIGMA_SPI_RESULT_OK)
    {
        sigma_spi_it_write_errors++;
    }

    sigma_spi_diag_note(result, SIGMA_SPI_OP_WRITE_BLOCK_IT, address, hal_status, hal_error, abort_status, detail);
    return result;
}

void SIGMA_WRITE_REGISTER_BLOCK_IT(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    (void) sigma_spi_write_block_it(devAddress, address, length, pData, NULL);
}

sigma_spi_result_t sigma_spi_safeload_begin(sigma_spi_call_detail_t* detail)
{
    if (s_channel_faulted)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_SAFELOAD, 0U, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    if (!sigma_spi_runtime())
    {
        // スケジューラ開始前はRTOS待機を行わない。単一呼出し前提でポーリングする。
        return SIGMA_SPI_RESULT_OK;
    }

    if (!sigma_spi_sync_objects_ready())
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_NOT_INITIALIZED, SIGMA_SPI_OP_SAFELOAD, 0U, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_NOT_INITIALIZED;
    }

    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(SIGMA_SPI_MUTEX_TIMEOUT_MS)) != osOK)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_MUTEX_TIMEOUT, SIGMA_SPI_OP_SAFELOAD, 0U, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_MUTEX_TIMEOUT;
    }

    // 待機中に別要求が停止失敗でFAULTへラッチした場合、バッファ操作前に拒否する。
    if (s_channel_faulted)
    {
        osMutexRelease(spiMutexHandle);
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_SAFELOAD, 0U, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    return SIGMA_SPI_RESULT_OK;
}

void sigma_spi_safeload_end(void)
{
    if (sigma_spi_runtime() && (spiMutexHandle != NULL))
    {
        osMutexRelease(spiMutexHandle);
    }
}

sigma_spi_result_t sigma_spi_safeload_write_locked(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData,
                                                   sigma_spi_call_detail_t* detail)
{
    uint32_t hal_status = 0U;
    uint32_t hal_error = 0U;
    uint32_t abort_status = 0U;

    if ((pData == NULL) || (length == 0U) ||
        (length > (SIGMA_SPI_SAFELOAD_BUFFER_SIZE - SIGMA_SPI_HEADER_SIZE)) ||
        (((uint32_t) SIGMA_SPI_HEADER_SIZE + (uint32_t) length) > SIGMA_SPI_MAX_HAL_TRANSFER) ||
        !sigma_spi_address_range_valid(dataAddress, length))
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_INVALID_ARG, SIGMA_SPI_OP_SAFELOAD, dataAddress, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_INVALID_ARG;
    }
    if (s_channel_faulted)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_SAFELOAD, dataAddress, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    spi_safeload_buf[0] = devAddress;
    spi_safeload_buf[1] = (uint8_t) ((dataAddress >> 8) & 0x00FF);
    spi_safeload_buf[2] = (uint8_t) (dataAddress & 0x00FF);
    for (uint16_t i = 0; i < length; i++)
    {
        spi_safeload_buf[i + SIGMA_SPI_HEADER_SIZE] = pData[i];
    }

    sigma_spi_result_t result;
    if (sigma_spi_runtime())
    {
        result = sigma_spi_transfer_it(SIGMA_SPI_XFER_TX, spi_safeload_buf, NULL,
                                       (uint16_t) (SIGMA_SPI_HEADER_SIZE + length),
                                       SIGMA_SPI_IT_TIMEOUT_MS,
                                       &hal_status, &hal_error, &abort_status);
    }
    else
    {
        result = sigma_spi_transfer_poll(spi_safeload_buf,
                                         (uint16_t) (SIGMA_SPI_HEADER_SIZE + length),
                                         &hal_status, &hal_error, &abort_status);
    }

    sigma_spi_diag_note(result, SIGMA_SPI_OP_SAFELOAD, dataAddress, hal_status, hal_error, abort_status, detail);
    return result;
}

sigma_spi_result_t sigma_spi_safeload_write_data(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData,
                                                 sigma_spi_call_detail_t* detail)
{
    const sigma_spi_result_t begin = sigma_spi_safeload_begin(detail);
    if (begin != SIGMA_SPI_RESULT_OK)
    {
        return begin;
    }

    const sigma_spi_result_t result = sigma_spi_safeload_write_locked(devAddress, dataAddress, length, pData, detail);
    sigma_spi_safeload_end();
    return result;
}

void SIGMA_SAFELOAD_WRITE_DATA(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    (void) sigma_spi_safeload_write_data(devAddress, dataAddress, length, pData, NULL);
}

void SIGMA_WRITE_DELAY(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    HAL_Delay(15);
}

sigma_spi_result_t sigma_spi_read_register(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData,
                                           sigma_spi_call_detail_t* detail)
{
    uint32_t hal_status = 0U;
    uint32_t hal_error = 0U;
    uint32_t abort_status = 0U;

    if ((pData == NULL) || (length == 0U) ||
        (length > (SIGMA_SPI_READ_BUFFER_SIZE - SIGMA_SPI_HEADER_SIZE)) ||
        (((uint32_t) SIGMA_SPI_HEADER_SIZE + (uint32_t) length) > SIGMA_SPI_MAX_HAL_TRANSFER) ||
        !sigma_spi_address_range_valid(address, length))
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_INVALID_ARG, SIGMA_SPI_OP_READ, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_INVALID_ARG;
    }
    if (s_channel_faulted)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_READ, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }
    if (!sigma_spi_runtime() || !sigma_spi_sync_objects_ready())
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_NOT_INITIALIZED, SIGMA_SPI_OP_READ, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_NOT_INITIALIZED;
    }

    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(SIGMA_SPI_MUTEX_TIMEOUT_MS)) != osOK)
    {
        sigma_spi_diag_note(SIGMA_SPI_RESULT_MUTEX_TIMEOUT, SIGMA_SPI_OP_READ, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_MUTEX_TIMEOUT;
    }

    // 待機中に別要求が停止失敗でFAULTへラッチした場合、バッファ操作前に拒否する。
    if (s_channel_faulted)
    {
        osMutexRelease(spiMutexHandle);
        sigma_spi_diag_note(SIGMA_SPI_RESULT_FAULT, SIGMA_SPI_OP_READ, address, 0U, 0U, 0U, detail);
        return SIGMA_SPI_RESULT_FAULT;
    }

    // ADAU1466 SPI Read: [Chip Addr | R/W bit = 1] [Addr High] [Addr Low] [dummy x length]
    spi_read_tx_buf[0] = devAddress | 0x01U;
    spi_read_tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
    spi_read_tx_buf[2] = (uint8_t) (address & 0x00FF);
    for (uint16_t i = 0; i < length; i++)
    {
        spi_read_tx_buf[i + SIGMA_SPI_HEADER_SIZE] = 0x00U;
    }

    const sigma_spi_result_t result = sigma_spi_transfer_it(SIGMA_SPI_XFER_TXRX,
                                                            spi_read_tx_buf, spi_read_rx_buf,
                                                            (uint16_t) (SIGMA_SPI_HEADER_SIZE + length),
                                                            SIGMA_SPI_IT_TIMEOUT_MS,
                                                            &hal_status, &hal_error, &abort_status);

    if (result == SIGMA_SPI_RESULT_OK)
    {
        // RX出力は正常完了時だけコピーする（失敗時はcallerの出力バッファを変更しない）。
        for (uint16_t i = 0; i < length; i++)
        {
            pData[i] = spi_read_rx_buf[i + SIGMA_SPI_HEADER_SIZE];
        }
    }

    osMutexRelease(spiMutexHandle);

    sigma_spi_diag_note(result, SIGMA_SPI_OP_READ, address, hal_status, hal_error, abort_status, detail);
    return result;
}

void SIGMA_READ_REGISTER(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    (void) sigma_spi_read_register(devAddress, address, length, pData, NULL);
}
