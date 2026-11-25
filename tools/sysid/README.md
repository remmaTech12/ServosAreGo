# サーボのシステム同定

固定した振り子治具で位置指令と応答を記録する。初版の実機アダプタは DYNAMIXEL XL330-M288-T（モデル番号 1200）。通常のサーボ操作と計測は同じシリアルポートを使う。既存の設定画面で Dynamixel と通信速度を選び、検出した ID を指定する。

## 計測

リポジトリのルートから `python web/build_html.py` でファームウェアを含む ServosAreGo.html を生成する（PlatformIO が必要）。Web Serial 対応ブラウザで開き、ファームウェアを書き込み、System Identification を開く。

XL330 は安定化した 5 V 電源と共通 GND に接続する。治具の機械的可動範囲を確認し、自由に下垂した位置のエンコーダ値を zero に入力する。計測前はトルク OFF、通常位置モード、I/D/FF および内蔵プロファイルはゼロ。質量は kg、軸から先端質量までの長さは m。通信は 1 Mbps を推奨する。5 ms は目標周期で、1周期以上遅れると停止する。

単発では Save JSON の保存後に Release trial を押す。一括では書き込み先フォルダーを選び、各試験の保存完了後に次へ進む。軌道を収めるには、sin_sin の最大振幅まで治具が動ける必要がある。完走したファイルのみを BAM に渡す。単発の診断は Save diagnostics で保存する。比較結果と診断 JSON は BAM の入力ディレクトリから分ける。

CLI は pyserial を使用する。ブラウザを切断し、同じ設定のまま次のように起動する（数値は例）。

```sh
python tools/sysid/record.py --port /dev/ttyUSB0 --id 1 --kp 100 \
  --zero 2048 --limit 95 --mass 0.03 --arm-mass 0.008 --length 0.10 \
  --trajectory up_and_down --out raw
```

`--standard` は4軌道 × 5ゲインを実行する。BAM ログは upstream の記録形式と同じ。別の診断 JSON に指令送信完了時刻（µs）と生電流値を持つ。送信完了時刻はサーボの内部制御時刻の近似。ESP32 の RAM の試験データはリセットで消える。

## BAM と比較

軌道生成・検証に使う Rhoban/bam は b9e11118e17fe8ec803d430202f6fcf31d57901e。BAM のリポジトリで identification extra を導入し、以下のパスは絶対パスを指定する。

```sh
uv sync --extra identification
uv run python -m bam.process --raw "$RAW" --logdir "$PROCESSED" --dt 0.005
uv run python -m bam.fit --actuator xl330 --model m1 --logdir "$PROCESSED" --validation_kp 400 --output "$PARAMS/m1.json"
uv run python "$SERVOS/tools/sysid/compare.py" --logdir "$PROCESSED" --custom-dir "$PARAMS" --output-dir "$RESULTS"
```

出力先ディレクトリを先に作成する。比較 CSV に加え、試験・モデルごとの予測 JSON を Web の Open recording or model prediction で表示できる。

## 機種の追加

sysid::Trial が受信・実行・記録・分割取得を担当する。機種ごとの検査・ゲイン変更・位置指令・トルク切替・計測は Adapter に実装する。初版の生データと角度・PWM換算は XL330 用。新機種では対応チャネルと換算、可動域、BAM モデルとの組み合わせを登録してから計測を有効にする。標準軌道を更新するときは BAM を import できる環境で generate.py を実行する。
