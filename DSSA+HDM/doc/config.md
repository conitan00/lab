# config.h - 設定ファイル

シミュレーションの各種パラメータを管理する設定ファイル

## 変数

| 変数名 | 型 | デフォルト値 | 説明 |
|--------|-----|-------------|------|
| num_agents | static const int | 30 | シミュレーションする船舶数 |
| num_episodes | static const int | 100 | 実行するエピソード数 |
| input_folder_name | string | "2009-04-04_PR10" | 流速データフォルダ名 |
| dates | string | "20090404" | 対象日付 (YYYYMMDD形式) |
| WS | string | "1" | 風速設定 |
| PR | string | "10" | 降水量設定 |
| probability_for_flip | double | 0.8 | フリップ確率（意思決定の確率的要素） |
| xmax_m | double | 35500 | x方向最大距離 (m) |
| ymax_m | double | 65494.84 | y方向最大距離 (m) |
| m_deg_change[2] | double[] | {80000.0, 119990.4} | 度→メートル変換係数 [x, y] |
| unit_change | double | 180 | 単位変換係数 |
| time_space | int | 3 | タイムステップ間隔 (分) |
| cost | double | 4 | 基本コスト値 |
| num_angles | int | 4 | 選択可能角度数（設定用） |
| space_angle_deg | double | 4 | 角度間隔 (度) |
| num_steps | int | 5 | ステップ数 |
| n_rate | double | 0.5 |  |
| a_rate | double | 0.5 |  |