/*
 * audio_control_internal.h
 *
 * Private API of audio_control.c for the Core audio modules.
 */

#ifndef AUDIO_CONTROL_INTERNAL_H_
#define AUDIO_CONTROL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

// サンプルレート切り替えの状態。READYだけが通常搬送を許可する。
typedef enum
{
    AUDIO_RATE_STATE_READY = 0,
    AUDIO_RATE_STATE_SWITCHING,
    AUDIO_RATE_STATE_FAILED,
} AudioRateState_t;

// レート切り替え要求の処理結果。FAILEDでは同じレート要求でも全手順をやり直す。
typedef enum
{
    AUDIO_RATE_SWITCH_NO_CHANGE = 0,
    AUDIO_RATE_SWITCH_SUCCESS,
    AUDIO_RATE_SWITCH_FAILED,
} AudioRateSwitchResult_t;

// 要求・適用・状態の一貫したsnapshot。USB/UI向けの参照はすべてこれを使う。
typedef struct
{
    uint32_t requested_hz;        // 最後に受理した要求値（物理適用の成功を意味しない）
    uint32_t requested_sequence;  // 受理ごとに増加する要求sequence
    uint32_t applied_hz;          // 最後に切り替え全体が成功した値（履歴）
    uint32_t applied_sequence;    // 適用が確定した要求sequence
    bool applied_hz_valid;        // 現在もその設定を信用できるか
    AudioRateState_t state;
} AudioRateSnapshot_t;

// UAC2 clock SET_CUR entry point. The USB control module validates the
// requested rate against its supported list and hands the request over here.
// Only the Audio Task applies the change; the control callback must not
// reinitialize SAI/GPDMA or the DSP path.
void audio_control_request_sample_rate(uint32_t sample_rate_hz);

// 要求・適用・状態を短い排他でまとめて取得する。
void audio_control_get_rate_snapshot(AudioRateSnapshot_t* snapshot);

// CLK_VALID用。最新要求と適用状態が整合し、全手順が成功した場合だけtrue。
bool audio_control_clock_valid(void);

// ISR/USB callbackから参照できる物理搬送レート。READY時は適用値、
// 切り替え中は固定targetを返す。
uint32_t audio_control_transport_sample_rate_hz(void);

// ISR/USB callbackから参照できる搬送許可。READYかつ適用確認済みで、受理済みの
// 未処理レート要求がない場合にtrue。
bool audio_control_transport_ready(void);

// ISR/USB callbackから参照できる経路安全フラグ。DMA/SAI停止確認を経た再構築後だけ
// true。停止未確認のまま復旧がBACKOFF/FAILEDへ入った場合はfalseとなり、通常搬送で
// DMA参照バッファを再利用しない。
bool audio_control_transport_paths_safe(void);

// 現在の要求sequence。commit境界の検証用にAudio Taskがservice入口で固定する。
uint32_t audio_control_rate_request_sequence(void);

// commit境界の検証。入口で固定した要求sequenceから変化しておらず、搬送許可と経路安全が
// 続いている場合だけtrue。要求publishと同じcritical sectionで評価するため、判定後に
// 新要求が受理された場合は次の境界でfalseになる。
bool audio_control_transport_commit_allowed(uint32_t request_sequence);

// 搬送のcommit区間とレート要求publishを直列化するTask間mutex。
// 時間のかかるTinyUSB APIを割り込み禁止区間へ入れずに排他するために使う。
// 取得できない場合(false)はcommitを見送る。Audio Task context only。
bool audio_control_commit_lock(void);
void audio_control_commit_unlock(void);

// 要求publish用。commit中は取得できるまで待つため、受理後に旧commitが残らない。
// 戻り値falseはAudio Task起動前（commitが走らない）の場合のみ。USB callback context。
bool audio_control_commit_lock_wait(void);

// StartAudioTaskの初期レート適用結果を公開する。成功時だけREADYになり、
// 通常搬送とCLK_VALIDを許可する。失敗時はFAILEDのままクロック無効を保持する。
void audio_control_publish_initial_rate_result(uint32_t applied_hz, bool success);

#endif /* AUDIO_CONTROL_INTERNAL_H_ */
