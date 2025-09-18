# Agent クラス

船舶エージェントを表現するクラス。各船舶の状態、意思決定、移動を管理する。

## 変数

| 変数名 | 型 | デフォルト値 | 説明 |
|--------|-----|-------------|------|
| ID | int | - | 船舶の一意識別子 |
| num | static int | - | 船舶の総数 |
| max_speed | int | 5 | 最大速度 (m/s) |
| min_speed | int | 1 | 最小速度 (m/s) |
| reference_speed | double | max_speed | 参照速度 |
| ship_size | int | 50 | 船舶サイズ (m) |
| detection_range | double | 20.0 * max_speed * dict.unit_change | 他船検知範囲半径 (m) |
| safety_dom | double | 2.0 * ship_size | 安全領域半径 (m) |
| time_window | static const int | 5 | タイムウィンドウ |
| num_agents | static const int | dict.num_agents | 船舶総数 |
| num_angles | static const int | 19 | 選択可能角度数 |
| num_speeds | static const int | 9 | 選択可能速度数 |
| current_speed | double | 1 | 現在速度 (m/s) |
| current_speed_id | int | 4 | 現在速度のインデックス |
| intention_speed | double | 0 | 意図速度 (m/s) |
| intention_speed_id | int | 4 | 意図速度のインデックス |
| next_intention_speed | double | 0 | 次の意図速度 (DSSA計算用) |
| possible_change_speed[9] | double[] | {-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0} | 速度変化量選択肢 |
| possible_speed[9] | double[] | - | 現在選択可能な速度 |
| actual_speed[20][9] | double[][] | 0 | 実際の航路速度 (3分間) |
| actual_speed_tcpa[20][9] | double[][] | 0 | TCPA計算用実際速度 (15分間) |
| current_heading | double | 0 | 現在方位角 (rad) |
| current_heading_id | int | 9 | 現在方位角のインデックス |
| intention_heading | double | 0 | 意図方位角 (rad) |
| intention_heading_id | int | 9 | 意図方位角のインデックス |
| next_intention_heading | double | 0 | 次の意図方位角 (DSSA計算用) |
| possible_angle_deg | vector<double> | - | 選択可能角度 (度) |
| possible_angle_rad | vector<double> | - | 選択可能角度 (ラジアン) |
| possible_heading[20] | double[] | - | 現在選択可能な方位角 |
| actual_heading[20][9] | double[][] | 0 | 実際の航路方位角 (3分間) |
| actual_heading_tcpa[20][9] | double[][] | 0 | TCPA計算用実際方位角 (15分間) |
| current_pos[2] | double[] | {0} | 現在位置 [x, y] (m) |
| init_pos[2] | double[] | {0} | 初期位置 [x, y] (m) |
| goal_pos[2] | double[] | {0} | 目標位置 [x, y] (m) |
| isSelectable[20][9] | bool[][] | true | 各選択肢の選択可能性 |
| cost[20][9] | double[][] | 0 | 各選択肢のコスト |
| weight_safety | double | 1.0 | 安全性重み |
| notAtGoal | bool | true | 目標未到達フラグ |
| collision | bool | false | 衝突フラグ |
| neighbors[num_agents] | bool[] | false | 近隣船舶検知フラグ |
| satisfied | bool | false | 意思決定完了フラグ |
| path_length | double | 0.0 | 経路総距離 (m) |
| time_step | int | 0 | 累計タイムステップ数 |
| trajectory | vector<double> | - | 船舶軌跡データ |
| start_port_ID | int | - | 出発港ID |
| goal_port_ID | int | - | 到着港ID |
| distance_offset | double | 0.0 | 距離オフセット |

## 関数

| 関数名 | 戻り値 | 説明 |
|--------|--------|------|
| Agent() | - | コンストラクタ。角度選択肢を初期化 |
| init(int episode) | void | エピソード開始時の初期化 |
| set_init_pos(double x, double y) | void | 初期座標を設定 |
| set_current_pos(double x, double y) | void | 現在座標を変更 |
| set_goal_pos(double x, double y) | void | 目標座標を設定 |
| get_goal_heading() | double | 目標方向の絶対角度を取得 (rad) |
| get_distance_to_goal() | double | 目標座標までの直線距離を取得 (m) |
| get_distance_to_other(Agent* other) | double | 他船との距離を取得 (m) |
| queryNeighbors(int from_agent_ID) | bool | 指定船舶が検知範囲内かを返す |
| updateNeighbors(int myID, vector<Agent>& agent) | void | 検知範囲内の船舶リストを更新 |