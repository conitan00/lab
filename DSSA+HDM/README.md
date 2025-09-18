# DSSA+HDM

## 実行方法

```bash
make compile    # コンパイル
```
```bash
make run        # 実行
```
```bash
make visualize  # 衝突地点の可視化
```

## フォルダ構成

```
DSSA+HDM/
├── src/                      # ソースコード
│   ├── main.cpp             # メイン実行
│   ├── agent.h              # 船舶クラス
│   ├── biwako.h             # 琵琶湖クラス
│   ├── mover.h              # 船舶移動・意思決定クラス
│   ├── config.h             # 設定ファイル
│   ├── csv.h                # CSVライブラリ
│   └── visualize_collision.py # 衝突可視化スクリプト
├── input/                   # 入力データ
│   ├── icover.csv          # 地形データ
│   ├── port_data.csv       # 港データ
│   ├── pattern/            # 船舶ルートパターン
│   ├── FS_WS1/             # 流速データ（風速1m/s）
│   └── FS_WS10/            # 流速データ（風速10m/s）
├── output/                  # 実行結果
│   └── YYYYMMDD_HHMMSS/    # 各実行結果フォルダ
│       ├── collision/       # 衝突データ
│       ├── result/          # シミュレーション結果
│       ├── trajectory/      # 船舶軌跡データ
│       └── collision_visualization.png # 衝突可視化画像
├── doc/                     # ドキュメント
│   ├── agent.md            # Agentクラス詳細
│   ├── biwako.md           # biwakoクラス詳細
│   ├── mover.md            # MoveAgentsクラス詳細
│   └── config.md           # 設定ファイル詳細
├── Makefile                # 実行スクリプト
└── README.md
```

