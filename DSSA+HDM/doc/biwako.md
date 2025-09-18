# biwako クラス

琵琶湖の環境を表現するクラス。地形、流速、港データなどの環境情報を管理し、船舶の実際の移動計算を行う。

## 変数

| 変数名 | 型 | 値 | 説明 |
|--------|-----|-----|------|
| cell_size | const int | 500 | セルの一辺の長さ (m) |
| num_cell_x | const int | 71 | x方向のセル数 |
| num_cell_y | const int | 131 | y方向のセル数 |
| num_data | const int | 400 | 取得する流速データ数 |
| lon_west | static const double | 135.853125 | 西端の経度 |
| lat_south | static const double | 34.977083 | 南端の緯度 |
| horizontal_size_m | static const double | 35500 | x方向の距離 (m) |
| vertical_size_m | static const double | 65494.84 | y方向の距離 (m) |
| FS_data[401][9246][2] | static double | - | 流速データ [時間][位置][x/y成分] |
| icover_data[72][132] | static int | - | 地形データ (-1:陸地, 0:水域, 7:その他) |
| agent_root[num_agents][2] | static int | - | 各船舶の発着地ルート |
| pattern | static vector<vector<pair<int,int>>> | - | 船舶の出発地パターンを格納 |
| num_ports | static const int | 18 | 港の総数 |
| ID | int | - | 港のID |
| name | string | - | 港の名前 |
| lon | double | - | 港の経度 |
| lat | double | - | 港の緯度 |
| port | static vector<Port> | - | 港データの |

## 関数

| 関数名 | 戻り値 | パラメータ | 説明 |
|--------|--------|------------|------|
| biwako() | - | - | 流速・地形・港・パターンデータを読み込み |
| cal_FS(int min, double x, double y, double* x_flow_ptr, double* y_flow_ptr) | static void | min: 時間(分)<br>x, y: 位置座標<br>x_flow_ptr, y_flow_ptr: 流速格納先 | 船舶位置における流速を計算 |
| cal_actual(int time_step, Agent& agent, int crs, int spd) | static void | time_step: タイムステップ<br>agent: 船舶オブジェクト<br>crs: 方位インデックス<br>spd: 速度インデックス | 流速を考慮した実際の移動経路を計算 (3分間) |
| cal_actual_tcpa(int time_step, Agent& agent, int crs, int spd) | static void | time_step: タイムステップ<br>agent: 船舶オブジェクト<br>crs: 方位インデックス<br>spd: 速度インデックス | TCPA計算用の実際の移動経路を計算 (15分間) |
| cal_goal_intention(int time_step, double cur_x, double cur_y, double goal_heading, double intention_speed) | static double | time_step: タイムステップ<br>cur_x, cur_y: 現在位置<br>goal_heading: 目標方向<br>intention_speed: 意図速度 | 流速を考慮した目標到達のための意図方向を計算 |