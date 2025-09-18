# MoveAgents クラス

船舶群の移動と意思決定を管理するメインクラス。

## 変数

| 変数名 | 型 | デフォルト値 | 説明 |
|--------|-----|-------------|------|
| num_agents | static const int | - | 船舶の総数 |
| num_angles | int | - | 選択可能な角度の数 |
| num_speeds | int | - | 選択可能な速度の数 |
| agent | vector<Agent> | - | 船舶エージェントの配列 |
| scenario | string | - | シナリオ名 |
| episode_final | bool | false | エピソード終了フラグ |
| isActive | bool | true | シミュレーション実行中フラグ |
| max_time_step | int | 100 | 最大タイムステップ数 |
| max_epoch | int | 1000000 | 最大エポック数 |
| max_episodes | int | dict.num_episodes | 実行エピソード数 |
| max_round | int | 200 | 最大ラウンド数 |
| time_step | int | 0 | 現在のタイムステップ |
| time_tcpa | static double | - | TCPA計算にかかった時間 |
| time_avoid | static double | - | 回避計算にかかった時間 |
| cnt_avoid | int | 0 | 回避行動回数 |
| powt[10] | static double[] | - | 各種計算時間の記録 |
| ave_timestep | vector<double> | - | 各エピソードの平均タイムステップ |
| SD_timestep | vector<double> | - | 各エピソードのタイムステップ標準偏差 |
| cnt_success | int | 0 | 成功エピソード数 |
| output_dir | string | - | 出力先ディレクトリパス |
| USE_HDM_ALGORITHM | static constexpr bool | true | HDMアルゴリズム使用フラグ |
| NEIGHBOR_THRESHOLD | static constexpr int | 5 | 周辺船舶数の閾値 |
| REFERENCE_SPEED_RATIO | static constexpr double | 1.0 | 基準速度比率 |
| SPEED_WEIGHT_RATIO | static constexpr double | 1.0 | 速度重み比率 |

## 関数

| 関数名 | 戻り値 | パラメータ | 説明 |
|--------|--------|------------|------|
| MoveAgents(string file_name) | - | file_name: 出力ファイル名 | コンストラクタ。船舶の初期化と出力ディレクトリ作成 |
| run_DSSQ() | void | - | DSSAアルゴリズムによるシミュレーション実行 |
| _record(int episode, vector<double> agent_times) | void | episode: エピソード番号<br>agent_times: 各船舶の所要時間 | 結果をCSVファイルに記録 |
| differenceBetweenAngle(double angle1, double angle2) | double | angle1, angle2: 比較する角度 (rad) | 2つの角度間の最短差を計算 |
| create_VarDom() | void | - | 全船舶のドメイン（選択可能領域）を構成 |
| create_VarDomEach(int i) | void | i: 船舶インデックス | 指定船舶のドメインを構成 |
| perform_withDSSA() | void | - | DSSA を用いて全船舶が意思決定を実行 |
| compNextIntentionByDSSA(int i) | void | i: 船舶インデックス | 指定船舶のDSSAによる次の意図決定 |
| updateCostTable(int i) | void | i: 船舶インデックス | 指定船舶の各選択肢のコストを計算・更新 |
| avoid(int i, int crs, int spd) | void | i: 船舶インデックス<br>crs: 方位インデックス<br>spd: 速度インデックス | 陸地回避のためのコストを計算 |
| compTCPAandDCPA(int i, int crs, int spd, int j, double* tcpa_ptr, double* dcpa_ptr) | void | i: 自船インデックス<br>crs: 方位インデックス<br>spd: 速度インデックス<br>j: 他船インデックス<br>tcpa_ptr: TCPA格納先<br>dcpa_ptr: DCPA格納先 | 2船間のTCPA (Time to Closest Point of Approach) とDCPA (Distance at Closest Point of Approach) を計算 |