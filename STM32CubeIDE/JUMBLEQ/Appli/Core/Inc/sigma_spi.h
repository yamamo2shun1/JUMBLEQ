/*
 * sigma_spi.h
 *
 * SigmaDSP向けSPI通信の結果付きAPIと永続診断値。
 *
 * SigmaStudio互換のvoid APIは SigmaStudioFW.h が所有し、本ヘッダーは
 * 手書きコード用の結果付きAPIだけを宣言する（生成物から独立）。
 *
 * SIGMA_SPI_RESULT_OK は「SPI転送が正常完了した」ことだけを意味し、
 * DSP内部でのパラメータ適用完了やレジスター値の正しさは保証しない。
 */

#ifndef INC_SIGMA_SPI_H_
#define INC_SIGMA_SPI_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SIGMA_SPI_RESULT_OK = 0,
    SIGMA_SPI_RESULT_INVALID_ARG,      // NULL、長さ0、storage上限超過、次アドレスoverflow
    SIGMA_SPI_RESULT_NOT_INITIALIZED,  // 同期オブジェクト未生成、またはスケジューラ未実行
    SIGMA_SPI_RESULT_MUTEX_TIMEOUT,    // mutex取得の期限切れ
    SIGMA_SPI_RESULT_START_FAILED,     // HAL開始失敗（停止確認済みで再利用可能）
    SIGMA_SPI_RESULT_TRANSFER_ERROR,   // HAL ErrorCallback（OVR/MODF/FRE/UDR等）
    SIGMA_SPI_RESULT_TIMEOUT,          // 完了待ちの期限切れ（停止確認後に確定）
    SIGMA_SPI_RESULT_STOP_FAILED,      // 停止確認に失敗（FAULTへラッチ）
    SIGMA_SPI_RESULT_FAULT,            // FAULT中のため転送を拒否
} sigma_spi_result_t;

typedef enum
{
    SIGMA_SPI_OP_NONE = 0,
    SIGMA_SPI_OP_WRITE_BLOCK_POLL,
    SIGMA_SPI_OP_WRITE_BLOCK_IT,
    SIGMA_SPI_OP_SAFELOAD,
    SIGMA_SPI_OP_READ,
} sigma_spi_op_t;

// AUDIO_DIAG_LOGに依存しない永続診断。
typedef struct
{
    uint32_t ok_count;
    uint32_t invalid_arg_count;
    uint32_t not_initialized_count;
    uint32_t mutex_timeout_count;
    uint32_t start_failed_count;
    uint32_t transfer_error_count;
    uint32_t transfer_timeout_count;
    uint32_t stop_failed_count;
    uint32_t fault_reject_count;
    uint32_t last_result;        // sigma_spi_result_t
    uint32_t last_operation;     // sigma_spi_op_t
    uint32_t last_address;
    uint32_t last_hal_status;    // 開始または停止時のHAL status
    uint32_t last_hal_error;     // 元のhspi5.ErrorCode（HAL_SPI_Abortで消える前に保存）
    uint32_t last_abort_status;  // HAL_SPI_Abortの戻り値
} sigma_spi_diagnostics_t;

extern volatile sigma_spi_diagnostics_t g_sigma_spi_diagnostics;

// ポーリング書込み（チャンク分割、スケジューラ前は排他なしの単一呼出し前提）。
sigma_spi_result_t sigma_spi_write_block(uint8_t devAddress, uint16_t address, uint32_t length, uint8_t* pData);

// IT書込み。スケジューラ実行中のmutex／セマフォが必要。
sigma_spi_result_t sigma_spi_write_block_it(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData);

// IT読み出し。成功時だけpDataへコピーする。
sigma_spi_result_t sigma_spi_read_register(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData);

// Safeload一括操作（data→control→frame待ち）用の外側排他。
// beginからendまでの間は、lock済み書込み以外のSPI操作を呼ばないこと。
sigma_spi_result_t sigma_spi_safeload_begin(void);
void sigma_spi_safeload_end(void);
sigma_spi_result_t sigma_spi_safeload_write_locked(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData);

// 単発Safeload書込み（互換wrapper用）。begin/write/endを内部で行う。
sigma_spi_result_t sigma_spi_safeload_write_data(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData);

#endif /* INC_SIGMA_SPI_H_ */
