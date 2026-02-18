/*
 * audio_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_AUDIO_CONTROL_H_
#define INC_AUDIO_CONTROL_H_

#include "main.h"

// バッファサイズ設定 - 小さいほど低レイテンシーだがアンダーラン/オーバーランのリスク増
// 96kHz再生の安定性を優先し、TX/RING は余裕を持たせる。
// 48kHz時のレイテンシー目安: SAI_RNG_BUF_SIZE / sample_rate * 1000 [ms]
#define SAI_RNG_BUF_SIZE 8192  // リングバッファ（2のべき乗必須）
#define SAI_TX_BUF_SIZE  1024  // 4ch DMAバッファ (USB->SAI)
#define SAI_RX_BUF_SIZE  1024  // 4ch DMAバッファ (SAI->USB)
// TXリングの目標水位（word単位）。小さくするほど低レイテンシーだが耐ジッタ性は下がる。
#define SAI_TX_TARGET_LEVEL_WORDS SAI_TX_BUF_SIZE

#define POT_CH_SEL_WAIT           1
#define ADC_NUM                   8
#define POT_MA_SIZE               4  // 移動平均のサンプル数
#define MAG_MA_SIZE               1  // 移動平均のサンプル数
#define POT_NUM                   8
#define MAG_SW_NUM                6
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_XFADE_CUTOFF          16
#define MAG_XFADE_RANGE           1408

// Runtime DSP parameter update switch for A/B diagnosis.
// 0: disable ui_control_task() DSP writes (noise root-cause test mode)
// 1: enable normal runtime control updates
#define ENABLE_DSP_RUNTIME_CONTROL 1

void reset_audio_buffer(void);
uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);
uint8_t get_current_xfA_position(void);
uint8_t get_current_xfB_position(void);
int16_t get_current_ch1_db(void);
int16_t get_current_ch2_db(void);
int16_t get_current_master_db(void);
int16_t get_current_dry_wet(void);
char* get_current_input_typeA_str(void);
char* get_current_input_typeB_str(void);
char* get_current_input_srcA_str(void);
char* get_current_input_srcB_str(void);
char* get_current_input_srcP_str(void);

void AUDIO_Init_AK4619(uint32_t hz);
void AUDIO_Init_ADAU1466(uint32_t hz);

void start_adc(void);
void ui_control_task(void);

void start_sai(void);

void start_audio_control(void);
bool is_started_audio_control(void);

bool get_sr_changed_state(void);
void reset_sr_changed_state(void);
void AUDIO_SAI_Reset_ForNewRate(void);
void audio_task(void);

#endif /* INC_AUDIO_CONTROL_H_ */
