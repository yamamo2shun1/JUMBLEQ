# JUMBLEQ 自作コード命名規則

この文書は `STM32CubeIDE/JUMBLEQ/Appli/Core` の自作コードに適用する命名規則を定義する。
外部・生成コードの識別子には適用しない。

## 対象外

- STM32CubeMX生成コードの識別子（`MX_*`、`HAL_*`、`Start*Task`、`*TaskHandle`、`main.c` の生成部など）
- HAL / CMSIS / FreeRTOS / TinyUSB / SEGGER / SigmaStudio+ / TCPP0203 / stm32-ssd1306 が所有する識別子
- DMA/リンカ契約シンボル（`adc_val`、`sai_tx_dma_buf`、`sai_rx_dma_buf`、`Node_*`、`List_*`）
- FreeRTOS契約シンボル（`ucHeap`）

## 関数

- 公開API: `<module>_<verb>_<object>` の lower_snake_case。
- private（static）関数: lower_snake_case。所有モジュールの接頭辞を付けることを推奨する。
- 識別子内で大文字小文字を混ぜない（`typeA` ではなく `type_a`）。

## 型

型名は `<component>_<name>_t` の lower_snake_caseとし、匿名 struct / union / enum の typedef で定義する。
componentは次の優先順で決める。

1. 永続診断型は、対応する診断グローバル名から `g_` を除いた名前をcomponentとする。
   - 例: `g_audio_tx_diagnostics` → `audio_tx_diagnostics_t`
   - 例: `g_audio_usb_feature_diagnostics` → `audio_usb_feature_diagnostics_t`
   - 例: `g_sigma_spi_diagnostics` → `sigma_spi_diagnostics_t`
2. それ以外は所有モジュールの接頭辞をcomponentとする。

| 所有モジュール（定義ファイル） | component |
| --- | --- |
| adau1466.h / adau1466.c | `adau1466_` |
| eeprom.h / eeprom.c | `eeprom_` |
| eeprom_config_internal.h / eeprom_config.c | `eeprom_config_` |
| timecode_oscillator.h / timecode_oscillator.c | `timecode_oscillator_` |
| timecode_synth.h / timecode_synth.c | `timecode_synth_` |
| audio_control_internal.h / audio_control.c | `audio_control_` |
| audio_transport_internal.h / audio_transport.c | `audio_transport_` |
| audio_usb_control_internal.h / audio_usb_control.c | `audio_usb_control_` |
| ui_control.h / ui_control.c | `ui_control_` |
| ui_persist_internal.h | `ui_persist_` |
| ui_midi_control_internal.h / ui_midi_control.c | `ui_midi_control_` |
| ui_uf2_control_internal.h / ui_uf2_control.c | `ui_uf2_control_` |
| ui_pot_control_internal.h / ui_pot_control.c | `ui_pot_control_` |
| ui_routing_control_internal.h / ui_routing_control.c | `ui_routing_`（関数接頭辞が `ui_routing_*` のため） |
| ui_ch_fader_internal.h / ui_ch_fader.c | `ui_ch_fader_` |
| led_control.c | `led_` |

- 所有が定義ファイルと異なる型は所有をコメント等で明記する。
  例: `audio_transport_failure_t` は audio_transport 所有（include循環を避けるため宣言のみ `audio_diagnostics_internal.h` に置く）。

## 変数・定数

- ファイルスコープオブジェクト: `static` + `s_` + lower_snake_case。const有無を問わない。
- 複数翻訳単位で共有するグローバル: `g_` + lower_snake_case。型名と一致させる必要はない。
- ブロックスコープstatic: 宣言位置で局所性が明確なため `s_` は付けず、lower_snake_caseとする。
- マクロ・enum値: UPPER_SNAKE_CASE。

## 単位

物理量を表す名前に単位接尾辞を付ける。無次元（0..1の比率など）には付けない。

`_ms`、`_us`、`_hz`、`_db`、`_bytes`、`_words`、`_frames`、`_samples`、`_count`、`_tick`

## 略語

次の略語を許容する。

`src`、`db`、`cfg`、`ptr`、`buf`、`len`、`idx`、`num`、`min`、`max`、`avg`、`prev`、`init`

- 同一概念で長短を混在させない（例: `audio_diag_` と `audio_diagnostics_` を混ぜない）。
- モジュールAPIの接頭辞は、そのモジュールの公開・内部ヘッダーの関数接頭辞と一致させる。

## 診断変数の例外

デバッガから参照する診断変数は、次の例外を認める。

- `dbg_` + lower_snake_case（例: `dbg_tx_used_min`、`dbg_usb_isr_count`）
- `.noinit` に置くFault記録（`g_hardFaultInfo` / `HardFaultInfo_t`）は名称・可視性とも変更しない。

診断変数の可視性（static / 非static）を変更する場合は、デバッガのWatch式や手動デバッグでの用途を確認し、
参照名の変更を明示してから行う。
