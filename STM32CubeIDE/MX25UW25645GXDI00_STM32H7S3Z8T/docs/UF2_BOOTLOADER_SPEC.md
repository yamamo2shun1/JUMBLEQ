# UF2ブートローダ 仕様整理（MX25UW25645GXDI00_STM32H7S3Z8T）

更新日: 2026-02-12

## 前提
- 対象プロジェクト: MX25UW25645GXDI00_STM32H7S3Z8T
- 現状は外部フラッシュ（XIP）アプリを起動するシンプル構成
- 追加するのは UF2 ブートローダ（USB MSC 経由）

## 決定済み要件
- ブートモード入口: SW2 を押しながら電源 ON（USB 接続）
- USB: HS（High Speed）使用
- 検証: CRC32 のみ
- アプリ配置: 外部フラッシュ上の固定オフセット
- ボリューム名: UF2BOOT
- USB VID/PID: 0x31BF / 0x0100

## 目的
- USB MSC として UF2 を受け取り、外部フラッシュのアプリ領域へ書き込み
- CRC32 検証後に XIP でアプリ起動

## 起動フロー（方針）
1. 低レベル初期化（クロック / MPU / キャッシュ）
2. XSPI + ExtMem Manager 初期化
3. ブートモード判定
   - SW2 押下かつ USB 接続なら UF2 モード
   - それ以外はアプリ起動
4. UF2 モード
   - USB MSC デバイス起動
   - UF2 ブロック受信 -> 外部フラッシュ書き込み
   - CRC32 検証 OK ならメタデータ更新
5. 通常モード
   - XIP アプリへジャンプ

## メモリマップ（方針）
- 内蔵フラッシュ
  - ブートローダ格納
- 外部フラッシュ（32MB = 0x0200_0000）
  - META 領域: 0x9000_0000 - 0x9000_FFFF (64KB)
  - APP 領域: 0x9001_0000 - 0x9040_FFFF (4MB)
  - 予備:     0x9041_0000 - 0x91FF_FFFF (残り)

### 固定オフセット
- `APP_OFFSET = 0x0001_0000` (64KB)
- `APP_BASE   = 0x9000_0000 + APP_OFFSET = 0x9001_0000`
- `EXTMEM_XIP_IMAGE_OFFSET` は `APP_OFFSET` と同値にする
- `EXTMEM_HEADER_OFFSET` は 0

※ META 領域は最小 4KB でも良いが、消去セクタ境界に合わせ 64KB を確保する前提。

## 消去・書き込み戦略（推奨）
- デバイスは 4KB セクタ消去と 64KB ブロック消去をサポート
- APP 領域は 64KB ブロック単位で消去（更新時間短縮のため）
- META のような小領域は 4KB セクタ消去を許可
- UF2 受信時は、書き込み対象アドレス範囲に対応する 64KB ブロックのみ事前消去

## UF2 処理方針
- USB MSC で 512B UF2 ブロックを受信
- `targetAddr` を外部フラッシュ APP 領域にマップ
- ブロック毎に書き込み
- 受信完了後、CRC32 を算出
- CRC32 OK -> META 更新

## UF2 MSC 仮想ディスク構成（提案）
- 目的: 4MB アプリの UF2 ファイル（サイズ約 8MB）を無理なく転送できる容量を確保
- ディスクサイズ: 16MB（16384KB）
- セクタサイズ: 512B
- クラスタサイズ: 1 セクタ
- FAT: FAT16
- ルートエントリ数: 64
- ボリュームラベル: `UF2BOOT`

### 生成するファイル（読み取り専用）
- `INFO_UF2.TXT`
  - ボード名/ビルド情報/対応範囲など
- `INDEX.HTM`
  - 製品ページや手順案内への誘導（必要なら）
- `CURRENT.UF2`（任意）
  - 現在のアプリを吸い出す用途（不要なら省略）

### 書き込み受理ルール
- UF2 ブロックのみ受理（UF2 マジック一致）
- `targetAddr` が `APP_BASE .. APP_BASE + APP_SIZE - 1` にあるものだけ書き込み
- 範囲外は無視（ディスクの通常書き込みは無視）
- 初回書き込み時に対象 64KB ブロックだけ消去
- `numBlocks` と `blockNo` を用いて受信完了を判断し CRC32 検証

## 運用手順（UF2 生成と書き込み）
### 前提
- アプリのリンクアドレスは `0x90010000`（APP_BASE）
- UF2 変換は `uf2conv.py` を使用
- `uf2families.json` は `uf2conv.py` と同じフォルダに置く
- family ID: `STM32H7RS = 0x6db66083`

### 1. ELF -> BIN 変換
```
arm-none-eabi-objcopy -O binary app.elf app.bin
```

### 2. BIN -> UF2 変換
```
python uf2conv.py -c -b 0x90010000 -f STM32H7RS -o app.uf2 app.bin
```

### 3. 書き込み
- SW2 押下で電源 ON（USB 接続）
- PC に `UF2BOOT` がマウントされる
- `app.uf2` を `UF2BOOT` にドラッグ＆ドロップ
- 書き込み完了後、自動でアプリへジャンプ

## CRC32 メタデータ仕様（提案）

### 位置
- META ベース: `0x9000_0000`
- 先頭 256B をメタデータ領域として使用

### 形式（リトルエンディアン）
```
typedef struct {
  uint32_t magic;       // 0x4D544131 ('MTA1')
  uint16_t version;     // 0x0001
  uint16_t header_size; // sizeof(MetaHeader) = 64
  uint32_t flags;       // bit0: valid
  uint32_t app_base;    // 0x90010000
  uint32_t app_size;    // max 0x00400000
  uint32_t app_crc32;   // CRC32(アプリ全体)
  uint32_t build_id;    // 任意 (ビルド番号 or Git short hash)
  uint32_t image_size;  // 実際に書き込んだサイズ
  uint32_t reserved[6]; // 0
} MetaHeader;
```

### 更新タイミング
- UF2 受信完了後に CRC32 を算出
- CRC32 が一致した場合のみ `flags.valid = 1` として META を更新
- 更新時は 4KB セクタ消去 -> 先頭 256B 書き込み

### 検証タイミング
- 起動時に `magic/version/header_size` を確認
- `flags.valid == 1` かつ `app_size` が上限内なら CRC32 検証
- 一致すれば XIP 起動、失敗なら UF2 モードへ

## LED 表示パターン（提案）
- 通常起動（XIPアプリへジャンプ）: LED0 1秒点灯 -> 消灯
- UF2待機（SW2押下で起動した更新モード）: LED0 ゆっくり点滅（1Hz）
- UF2書き込み中: LED1 速い点滅（4Hz）
- CRC32検証中: LED2 ブリージング（0.5Hz）
- 検証OK / 起動準備: LED0+LED1+LED2 を200ms同時点灯 -> 消灯
- 検証NG / エラー: LED2 2連点滅（100ms×2）を1秒周期で繰り返し
- USB未接続でSW2押下（無効）: LED1 3回短点滅

## XIP 起動方針
- `VTOR = APP_BASE`
- `MSP = *(uint32_t*)APP_BASE`
- `PC  = *(uint32_t*)(APP_BASE + 4)`
- キャッシュ clean/invalidate を実施してからジャンプ

## 現行 Boot 実装の確認（2026-02-12）
- `BOOT_Application()` は `stm32_boot_xip.c` に実装
- `MapMemory()` で `extmem_list_config` の全メモリを `EXTMEM_MemoryMappedMode()` へ
- `EXTMEM_MEMORY_BOOTXIP` が非対応なら `BOOT_ERROR_INCOMPATIBLEMEMORY`
- `JumpToApplication()` は以下の順で実行
  - `EXTMEM_GetMapAddress(EXTMEM_MEMORY_BOOTXIP, &Application_vector)`
  - `EXTMEM_XIP_IMAGE_OFFSET` と `EXTMEM_HEADER_OFFSET` を加算（現状は未定義のため両方 0）
  - SysTick / I-Cache / D-Cache を停止
  - `VTOR` 設定 -> `MSP` 設定 -> `PC` にジャンプ
- `Boot/Core/Src/main.c` では以下の順に初期化
  - `MX_XSPI1_Init()`
  - `MX_EXTMEM_MANAGER_Init()`
  - `BOOT_Application()`
- MPU では `0x90000000` を 32MB サイズで実行可・キャッシュ可として設定
- リンカは内蔵フラッシュ `0x08000000` から 64KB をブートローダ領域として使用
- `extmem_list_config` は NOR SFDP + XSPI1 + 8lines で初期化

## 次のアクション候補（残り）
1. 実装作業（ブートモード判定、USB MSC、UF2書き込み、META更新、LED制御）に着手

## 実装状況（2026-02-12）
- ブートモード判定、UF2解析、外部フラッシュ書き込み、META更新、LED制御の骨格を実装
- USB MSC は TinyUSB で動作（HS）
- MSC は最小FAT16（16MB）を返す読み取り専用ディスクとして実装
- UF2 書き込み後に CRC32 検証 -> 自動ジャンプまで動作確認済み
