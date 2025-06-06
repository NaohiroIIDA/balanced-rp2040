# Balanced RP2040

2輪型倒立振子ロボットのファームウェアプロジェクトです。Raspberry Pi Pico（RP2040）とODriveモーターコントローラーを使用して、自立走行するロボットを制御します。

## ハードウェア構成

- マイコン: Waveshare RP2040-Zero
- IMUセンサー: MPU6050（I2C接続）
- モーターコントローラー: ODrive（UART接続）
- RCコントローラー入力（オプション）
- ブザー（起動時の警告用）

### 接続図

```
RP2040-Zero
├── I2C (MPU6050)
│   ├── SDA: GPIO4
│   └── SCL: GPIO5
├── UART (ODrive)
│   ├── TX: GPIO0
│   └── RX: GPIO1
├── RC入力
│   └── ピッチ制御: GPIO2
└── ブザー: GPIO8
```

## 主な機能

- PID制御による姿勢安定化
- シリアルコマンドによるリアルタイムパラメータ調整
- RCコントローラーによるピッチ角オフセット制御
- 安全機能（最大角度制限、エラーチェック）
- デバッグ用シリアル出力

## 開発環境

- PlatformIO
- Framework: Arduino
- ボード: Waveshare RP2040-Zero
- 必要なライブラリ:
  - Adafruit MPU6050（バージョン2.2.6以上）

## セットアップ方法

1. PlatformIOをインストール
2. プロジェクトをクローン:
   ```bash
   git clone https://github.com/NaohiroIIDA/balanced-rp2040.git
   ```
3. 依存ライブラリのインストール:
   ```bash
   pio lib install "adafruit/Adafruit MPU6050"
   ```
4. ファームウェアのビルドとアップロード:
   ```bash
   pio run -t upload
   ```

## パラメータ調整

詳細な調整手順は[tuning_guide.txt](tuning_guide.txt)を参照してください。

### 使用可能なシリアルコマンド

- `status` - 現在のパラメータ値と制御状態を表示
- `kp <value>` - 比例ゲインの設定
- `ki <value>` - 積分ゲインの設定
- `kd <value>` - 微分ゲインの設定
- `ff <value>` - レートフィードフォワードゲインの設定
- `offset <value>` - ピッチ軸のオフセット値設定
- `start` - バランス制御を開始
- `stop` - バランス制御を停止（モーター停止）

### パラメータ調整の安全機能

- シリアルコマンド入力時に自動的にモーターが停止します
- パラメータ設定後、自動的に制御が再開されます
- 必要に応じて`stop`コマンドで手動停止、`start`コマンドで再開が可能です

### デバッグ出力の見方

シリアルモニタには以下の情報が表示されます：
- `P:` - 現在のピッチ角（度）
- `R:` - ピッチレート（度/秒）
- `E:` - ピッチ角誤差
- `I:` - 積分項
- `M:` - モーター有効状態（ON/OFF）

## 安全上の注意

1. 初回起動時は必ずロボットを手で支えた状態で行ってください
2. 起動後5秒間はモーターが無効化されています（ブザー音で警告）
3. 最大ピッチ角（45度）を超えると自動的に停止します
4. パラメータ調整は少しずつ行ってください
5. パラメータ調整時は自動的にモーターが停止するため、ロボットを手で支えてください

## ライセンス

MIT License

## 貢献

バグ報告や機能改善の提案は、GitHubのIssueやPull Requestsを通じてお願いします。
