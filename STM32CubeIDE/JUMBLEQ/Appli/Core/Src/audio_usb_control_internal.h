/*
 * audio_usb_control_internal.h
 *
 * Private API of the UAC2 control plane module. The LED blink intervals are
 * owned there; the transport reports the applied stream state so the blink
 * behavior stays identical to the previous audio_control.c implementation.
 */

#ifndef AUDIO_USB_CONTROL_INTERNAL_H_
#define AUDIO_USB_CONTROL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

void audio_usb_control_set_tx_stream_blink(bool streaming);
void audio_usb_control_set_rx_stream_blink(bool streaming);

// ---- アプリ側feedback管理 ----
// TinyUSB内部の自動計算はAUDIO_FEEDBACK_METHOD_DISABLEDで止め、feedback値は
// ここで計算して公開API tud_audio_n_fb_set() で更新する。定数・平滑化履歴の
// Task/ISR間共有は短い排他で保護し、ISRでは待機・ログ出力を行わない。

// 指定レートで定数と平滑化履歴を初期化し、公称feedback値を設定する。
// Audio Task・SET_CUR受理時に呼ぶ。切り替え中は固定target_hzを渡す。
void audio_usb_control_feedback_reset(uint32_t sample_rate_hz);

// USB OUTパケット受信ごとの軽量更新。現在のFIFO水位からfeedback値を計算する。
// 搬送禁止中は水位不足による補正を行わない。
void audio_usb_control_feedback_update(void);

// ---- Feature Unit（音量・ミュート）の非同期適用 ----
// USB callbackは要求値とdirty bitの記録だけを行い、DSPへの適用は専用Taskが
// チャンネルごとの最新値へ集約して実行する。失敗時はpendingを保持して再試行する。

// 適用失敗時の操作。診断のlast_failed_operationで使用する。
enum
{
    AUDIO_USB_FEATURE_OP_NONE = 0u,
    AUDIO_USB_FEATURE_OP_GAIN = 1u,
    AUDIO_USB_FEATURE_OP_MUTE = 2u,
};

// AUDIO_DIAG_LOGに依存しない永続診断。
typedef struct
{
    uint32_t request_count;            // SET_CUR受理数（mute/volume合計）
    uint32_t master_request_count;     // Master(ch0)要求数
    uint32_t coalesced_request_count;  // 適用待ち中チャンネルへの追加要求数
    uint32_t applied_channel_count;    // GainとMuteの両方が成功したチャンネル数
    uint32_t failed_channel_count;     // 適用失敗チャンネル数
    uint32_t retry_count;              // バックオフ後の再試行回数
    uint32_t last_result;              // 最後のSPI結果（成功・失敗を問わない）
    uint32_t last_failed_result;       // 最後に失敗した操作のSPI結果（失敗時のみ更新）
    uint32_t last_failed_channel;      // 最後に失敗したチャンネル
    uint32_t last_failed_operation;    // AUDIO_USB_FEATURE_OP_*
    uint32_t last_apply_tick_ms;       // 最後に適用を試みたtick
    uint32_t backoff_active;           // 再試行バックオフ中か
    uint32_t task_create_failed;       // 専用Task生成に失敗したか
} AudioUsbFeatureDiagnostics_t;

extern volatile AudioUsbFeatureDiagnostics_t g_audio_usb_feature_diagnostics;

// 音量・ミュート適用専用Taskを開始する。DSP初期化とSAI開始の後に呼ぶ。
// 二重開始は無視する。生成失敗時は診断のtask_create_failedに記録する。
void audio_usb_control_feature_task_start(void);

// LED blink interval getters (consumed by led_control.c).
uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);

#endif /* AUDIO_USB_CONTROL_INTERNAL_H_ */
