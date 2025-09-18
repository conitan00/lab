#pragma once
#include <vector>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
//#include <iostream>
using namespace std;

// Forward declaration
class biwako;

//-------船舶クラス---------

class Agent{
    private:

    public:

    int ID; //船舶のID
    static int num;
    int max_speed = 5; //速度の最大値
    int min_speed = 1; //速度の最小値
    double reference_speed = max_speed; //参照速度
    int ship_size = 50; //船舶サイズ(m)
    double detection_range = 20.0 * max_speed * dict.unit_change; //検知範囲半径(m)
    double safety_dom = 2.0 * ship_size; //安全領域半径(m)
    static const int time_window = 5; //タイムウィンドウ
    static const int num_agents = dict.num_agents; //船舶の数
    static const int num_angles = 19; //選択可能な角度の数
    static const int num_speeds = 9; //選択可能な速度の数

    //------speed-------

    double current_speed = 1; //現在速度(m/s)
    int current_speed_id = 4; //現在速度のindex
    double intention_speed = 0; //意図速度(m/s)
    int intention_speed_id = 4; //意図速度のindex
    double next_intention_speed = 0; //意図速度(m/s)(DSSAによる計算時に使用）
    double possible_change_speed[num_speeds] = {-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0}; //選択可能な速度の変化量(m/s)
    double possible_speed[num_speeds]; //現在選択能な速度の選択肢
    double actual_speed[num_angles + 1][num_speeds] = { 0 }; //実際の航路(m/3min)
    double actual_speed_tcpa[num_angles + 1][num_speeds] = { 0 }; //tcpaの計算における実際の経路(m/15min)

    //------angle-------

    double current_heading = 0; //現在角度
    int current_heading_id = 9; //現在角度のindex
    double intention_heading = 0; //意図角度(rad)
    int intention_heading_id = 9; //意図角度のindex
    double next_intention_heading = 0; //意図角度(rad)(DSSAによる計算時に使用）
    vector<double> possible_angle_deg; //選択可能な角度の選択肢(deg)
    vector<double> possible_angle_rad; //選択可能な角度の選択肢(rad)
    double possible_heading[num_angles + 1]; //現在選択可能な角度の選択肢(rad)
    double actual_heading[num_angles + 1][num_speeds] = { 0 }; //実際の航路(m/3min)
    double actual_heading_tcpa[num_angles + 1][num_speeds] = { 0 }; //tcpaの計算における実際の経路(m/15min)

    //-----position-----

    double current_pos[2] = { 0 }; //現在位置
    double init_pos[2] = { 0 }; //初期位置
    double goal_pos[2] = { 0 }; //目標位置

    //------------------

    bool isSelectable[num_angles + 1][num_speeds] = { true };
    double cost[num_angles + 1][num_speeds] = { 0 }; //各選択肢のコスト
    double weight_safety = 1.0;
    bool notAtGoal = true; //目標到達フラグ
    bool collision = false; //衝突フラグ
    bool neighbors[num_agents] = { false }; //近隣の船舶のフラグ
    bool satisfied = false; //意思決定フラグ
    double path_length = 0.0; //経路距離(m)
    int time_step = 0; //累計timestep数
    vector<double> trajectory; //船舶の軌跡
    int start_port_ID; //出発地の港ID
    int goal_port_ID;  //到着地の港ID
    double distance_offset = 0.0;

    //コンストラクタ
    Agent();

    //各エピソード開始時の初期化
    void init(int episode);

    // 初期座標を指定した値に設定
    void set_init_pos(double x, double y);

    // 現在座標を指定した値に変更
    void set_current_pos(double x, double y);
    // 目標座標を指定した値に設定
    void set_goal_pos(double x, double y);

    // 目標方向（絶対角度）を返す
    double get_goal_heading();

    // 目標座標までの直線距離を返す
    double get_distance_to_goal();

    // 検知範囲内の船舶か否かの問合せに回答
    bool queryNeighbors(int from_agent_ID);

    // 検知範囲内の移動中の船舶(neighbors)を更新
    void updateNeighbors(int myID, vector<Agent>& agent);

    // 船舶間の距離を返す
    double get_distance_to_other(Agent* other);

};

// Agent implementation
inline Agent::Agent() {
    time_step = 0;

    // ドメイン初期化
    possible_angle_deg = { -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45 };
    for (int i = 0; i < Agent::num_angles; i++) {
        possible_angle_rad.push_back(possible_angle_deg[i] * (M_PI / 180));
    }
}

//-----初期座標を指定した値に設定------------
inline void Agent::set_init_pos(double x, double y){
    init_pos[0] = x;
    init_pos[1] = y;
    current_pos[0] = x;
    current_pos[1] = y;
}

//-----現在座標を指定した値に変更---------------
inline void Agent::set_current_pos(double x, double y){
    current_pos[0] = x;
    current_pos[1] = y;
}

//-----目標座標を指定した値に設定------------
inline void Agent::set_goal_pos(double x, double y){
    goal_pos[0] = x;
    goal_pos[1] = y;
}

//-----目標方向（絶対角度）を返す---------------
inline double Agent::get_goal_heading(){
    double x = goal_pos[0] - current_pos[0];
    double y = goal_pos[1] - current_pos[1];
    return atan2(y, x);
}

//-----目標座標までの直線距離を返す-------------
inline double Agent::get_distance_to_goal(){
    double x = goal_pos[0] - current_pos[0];
    double y = goal_pos[1] - current_pos[1];
    return sqrt(x * x + y * y);
}

//-----検知範囲内の船舶か否かの問合せに回答-----
inline bool Agent::queryNeighbors(int from_agent_ID){
    return neighbors[from_agent_ID];
}

//-----検知範囲内の移動中の船舶(neighbors)を更新----------------------------------------
inline void Agent::updateNeighbors(int myID, vector<Agent>& agent){
    for (int i = 0; i < num_agents ; i++)
    {
        if (i < myID)
        {
            neighbors[i] = agent[i].queryNeighbors(myID);
        }
        else if (i > myID)
        {
            double x = current_pos[0] - agent[i].current_pos[0];
            double y = current_pos[1] - agent[i].current_pos[1];
            double distance = sqrt(x * x + y * y);

            if (distance <= detection_range || distance <= agent[i].detection_range)
            {
                neighbors[i] = true;
            }
            else
            {
                neighbors[i] = false;
            }
        }
        else
        {
            neighbors[i] = false;
        }
     }
}

//-----船舶間の距離を返す---------------------------------
inline double Agent::get_distance_to_other(Agent* other){
    double x = current_pos[0] - other->current_pos[0];
    double y = current_pos[1] - other->current_pos[1];
    return sqrt(x * x + y * y);
}
