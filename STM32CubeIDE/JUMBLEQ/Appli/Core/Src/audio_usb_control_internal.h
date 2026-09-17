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

// LED blink interval getters (consumed by led_control.c).
uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);

#endif /* AUDIO_USB_CONTROL_INTERNAL_H_ */
