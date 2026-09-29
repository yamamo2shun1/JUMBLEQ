/*
 * adau1466.h
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_ADAU1466_H_
#define INC_ADAU1466_H_

#include <stdbool.h>
#include <stdint.h>

#include "sigma_spi.h"

// DSP入力source ID。各mux値への変換はドライバ内で行う。値は変更しない。
typedef enum
{
    AUDIO_INPUT_SOURCE_CH1 = 0,
    AUDIO_INPUT_SOURCE_CH2 = 1,
    AUDIO_INPUT_SOURCE_USB12 = 2,
    AUDIO_INPUT_SOURCE_USB34 = 3,
    AUDIO_INPUT_SOURCE_MASTER = 4,
} AudioInputSource_t;

enum
{
    INPUT_TYPE_LINE = 0,
    INPUT_TYPE_PHONO,
};

// ヘッドホン出力source値。DSPのHP_OUT mux値かつEEPROM保存値。値は変更しない。
typedef enum
{
    HP_SOURCE_CH_FADER_A = 0,
    HP_SOURCE_CH_FADER_B = 1,
    HP_SOURCE_THRU = 2,
    HP_SOURCE_MASTER = 3,
} HpSource_t;

// 10-bit POTs on the muxed ADC can stop slightly short of full-scale on hardware.
#define POT_10BIT_ADC_MAX            1023U
#define POT_10BIT_MIN_DEADZONE      10U
#define POT_10BIT_DB_MAX_SNAP_START 1005U
#define POT_10BIT_DW_MAX_SNAP_START 1005U

double convert_pot2dB(uint16_t adc_val);
int16_t convert_pot2dB_int(uint16_t adc_val);

// サンプルレート適用の失敗理由。SPI転送失敗とPLLロック待ちタイムアウトを区別する。
// 共有SPI診断の後読みに依存せず、失敗した処理から直接返す。
typedef enum
{
    ADAU1466_RATE_FAIL_NONE = 0,
    ADAU1466_RATE_FAIL_UNSUPPORTED,   // 未対応レート
    ADAU1466_RATE_FAIL_SPI,           // SPI操作失敗（step/spi_result/spi_hal_errorが有効）
    ADAU1466_RATE_FAIL_PLL_TIMEOUT,   // PLLロック待ちタイムアウト（SPI読み出し自体は成功の場合あり）
} adau1466_rate_fail_t;

typedef enum
{
    ADAU1466_RATE_STEP_NONE = 0,
    ADAU1466_RATE_STEP_MUX,           // USBレート選択muxのsafeload
    ADAU1466_RATE_STEP_SOUT_SOURCE,   // SOUT sourceレジスタ
    ADAU1466_RATE_STEP_CLK_GEN,       // CLK_GEN2 M
    ADAU1466_RATE_STEP_PLL_READ,      // PLLロック読み出し
} adau1466_rate_step_t;

typedef struct
{
    adau1466_rate_fail_t reason;
    adau1466_rate_step_t step;
    uint32_t spi_result;       // sigma_spi_result_t（失敗した呼出し固有の結果）
    uint32_t spi_hal_status;   // 失敗した呼出しのHAL status
    uint32_t spi_hal_error;    // 失敗した呼出しのhspi5.ErrorCode
    uint32_t spi_abort_status; // 停止を伴う失敗時のHAL_SPI_Abort戻り値
} adau1466_rate_failure_t;

// DSPリセット・program download・初期レート適用を行う。初期レート適用の成否を返し、
// program downloadそのものの成否は従来どおり検証しない。
bool AUDIO_Init_ADAU1466_Checked(uint32_t hz);
// 互換wrapper。初期レート適用の失敗はログのみで、戻り値を持たない。
void AUDIO_Init_ADAU1466(uint32_t hz);
// レートを適用する。失敗時はfailureへ理由（SPI失敗／PLLタイムアウト等）を返す。
bool AUDIO_Update_ADAU1466_SampleRate_Checked(uint32_t hz, adau1466_rate_failure_t* failure);
// 互換wrapper。失敗理由は取得できない。
bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz);

void set_dc_inputA(float ch_fader_position);
void set_dc_inputB(float ch_fader_position);
void safeload_write_q8_24(uint16_t addr, uint8_t mem_page, double val);

// USB Gain/MuteはSPI書込み結果を返す。不正チャンネルはSIGMA_SPI_RESULT_INVALID_ARG。
sigma_spi_result_t control_input_from_usb_gain(uint8_t ch, int16_t db);
sigma_spi_result_t control_input_from_usb_mute(uint8_t ch, bool muted);
void control_input_from_ch1_gain(const uint16_t adc_val);
void control_input_from_ch2_gain(const uint16_t adc_val);
void control_input_from_return_gain(const uint16_t adc_val);
void mute_input_from_return(void);

void control_send1_out_gain(const uint16_t adc_val);
void control_send2_out_gain(const uint16_t adc_val);

void control_dryA_out_gain(const uint16_t adc_val);
void control_dryB_out_gain(const uint16_t adc_val);

void control_wet_out_gain(const uint16_t adc_val);
void control_ch1_out_gain(const uint16_t adc_val);
void control_ch2_out_gain(const uint16_t adc_val);
void control_hp_out_gain(const uint16_t adc_val);

void select_input_type(AudioInputSource_t audio_input_source, uint8_t input_type);
void set_input_insert_enabled(AudioInputSource_t audio_input_source, bool enabled);
void select_send_source(AudioInputSource_t audio_input_source, bool select_insert);

void select_ch_fader_assign_a_source(AudioInputSource_t audio_input_source);
void select_ch_fader_assign_b_source(AudioInputSource_t audio_input_source);
void select_ch_fader_assign_post_source(AudioInputSource_t audio_input_source);
void select_return_ch_source(AudioInputSource_t audio_input_source);
void select_hp_out_source(HpSource_t hp_source);

#endif /* INC_ADAU1466_H_ */
