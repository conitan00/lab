#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <cmath>
#include <numeric>
#include <memory>
#include "agent.h"

using namespace std;

class MoveAgents{
private:

public:

    static const int num_agents = Agent::num_agents; //船舶の数
    int num_angles = Agent::num_angles; //選択可能な角度の数
    int num_speeds = Agent::num_speeds; //選択可能な速度の数
    vector<Agent> agent{num_agents}; //船舶クラスを動的にインスタンス化
    string scenario;
    bool episode_final = false;
    bool isActive = true;
    int max_time_step = 100;
    int max_epoch = 1000000;
    int max_episodes = dict.num_episodes; // 実行エピソード数
    int max_round = 200;
    int time_step = 0;
    static double time_tcpa;
    static double time_avoid;
    int cnt_avoid = 0;
    static double powt[10];
    vector<double> ave_timestep; // 各エピソードの平均タイムステップを記録
    vector<double> SD_timestep; // 各エピソードの標準偏差(タイムステップ)を記録
    int cnt_success; // 成功エピソード数を記録
    string output_dir; // 出力先ディレクトリ（data/タイムスタンプ/）
    
    // ========== アルゴリズム切り替え設定 ==========
    static constexpr bool USE_HDM_ALGORITHM = false; // true: DSSA+HDM, false: 基本DSSA+
    
    // HDMアルゴリズム用パラメータ（調整可能）
    static constexpr int NEIGHBOR_THRESHOLD = 5;           // 周辺船舶数の閾値
    static constexpr double REFERENCE_SPEED_RATIO = 1.0;   // 基準速度比率
    static constexpr double SPEED_WEIGHT_RATIO = 1.0;      // 速度重み比率
    // =============================================

    //コンストラクタ
    MoveAgents(string file_name);

    //DSSQの実行
    void run_DSSQ();

    //ファイルへの結果の保存
    void _record(int episode, vector<double> agent_times);

    //2つの角度の差を求める
    double differenceBetweenAngle(double angle1, double angle2);

    //ドメインを構成する
    void create_VarDom();

    //各船舶のドメインを構成
    void create_VarDomEach(int i);

    //DSSAを用いて各船舶が意思決定
    void perform_withDSSA();

    //コストを計算
    void updateCostTable(int i);
    
    //陸地を回避するためのコスト計算
    void avoid(int i, int crs, int spd);

    //TCPA,DCPAを計算
    void compTCPAandDCPA(int i, int crs, int spd, int j, double* tcpa_ptr, double* dcpa_ptr);

    //コストによる各船舶の意思決定
    void compNextIntentionByDSSA(int i);

};

// Implementation - 必要なヘッダーのインクルード
#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <array>
#include <set>
#include <chrono>
#include "dict.h"
#include "biwako.h"
#include <iomanip>
#include <filesystem>
using namespace std::chrono;

// Static member definitions
inline double MoveAgents::time_tcpa = 0;
inline double MoveAgents::time_avoid = 0;
inline double MoveAgents::powt[10] = { 0 };

//DSSQの実行
inline void MoveAgents::run_DSSQ() {
    cnt_success = 0;
    vector<pair<double, double>> collisions;  // 衝突地点記録用

    auto s = high_resolution_clock::now();

    for (int episode = 0; episode < max_episodes; episode++) {
        cout << "\n";
        auto start = high_resolution_clock::now();
        isActive = true;
        time_step = 0;
        time_avoid = 0;
        cnt_avoid = 0;
        vector<double> agent_times;

        for (int i = 0; i < num_agents; i++) {
            agent[i].current_heading = agent[i].get_goal_heading();
            agent[i].current_speed = agent[i].min_speed;
            agent[i].init(episode);
            agent[i].trajectory.push_back(int(agent[i].init_pos[0]));
            agent[i].trajectory.push_back(int(agent[i].init_pos[1]));
        }

        while (isActive) {
            if (time_step > max_time_step) {
                cout << "EPISODE" << episode + 1 << ": TimeOut" << endl;
                break;
            }

            create_VarDom();
            perform_withDSSA();
            bool allAgentsAtGoal = true;

            //--全船舶移動--
            for (int i = 0; i < num_agents; i++) {
                if (agent[i].notAtGoal) {
                    double radian = agent[i].actual_heading[agent[i].intention_heading_id][agent[i].intention_speed_id];
                    double speed = agent[i].actual_speed[agent[i].intention_heading_id][agent[i].intention_speed_id];
                    double curX = agent[i].current_pos[0];
                    double newX = curX + cos(radian) * speed;
                    double curY = agent[i].current_pos[1];
                    double newY = curY + sin(radian) * speed;
                    agent[i].set_current_pos(newX, newY);
                    agent[i].trajectory.push_back(newX);
                    agent[i].trajectory.push_back(newY);
                    agent[i].path_length += sqrt((newX - curX) * (newX - curX) + (newY - curY) * (newY - curY));
                    agent[i].time_step++;
                    allAgentsAtGoal = false;
                }
            }

            if (allAgentsAtGoal) {
                cnt_success++;
                for (int i = 0; i < num_agents; i++) {
                    agent_times.push_back(agent[i].time_step);
                    agent[i].trajectory.push_back(agent[i].current_pos[0]);
                    agent[i].trajectory.push_back(agent[i].current_pos[1]);
                }

                auto end = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(end - start);
                double microseconds = duration.count();
                double seconds = microseconds / 1000000.0;

                double mean_time_step = (accumulate(agent_times.begin(), agent_times.end(), 0.0)) / num_agents;
                double var = 0;
                for (const auto& t : agent_times) {
                    var += (t - mean_time_step) * (t - mean_time_step);
                }
                double SD = sqrt(var / num_agents);
                ave_timestep.push_back(mean_time_step);
                SD_timestep.push_back(SD);

                double total_distance = 0.0;
                for (int i = 0; i < num_agents; i++) {
                    agent[i].path_length += agent[i].distance_offset;
                    total_distance += agent[i].path_length;
                }
                double ave_distance = total_distance / num_agents;

                cout << "EPISODE" << episode + 1 << ": average time step: " << mean_time_step 
                     << ", average distance: " << ave_distance << ", erasped time; " << seconds << "(sec)" << endl;

                double j = 0;
                for (auto& f : ave_timestep) {
                    j += f;
                }
                cout << j / cnt_success << endl;

                if (cnt_success == max_epoch) {
                    episode_final = true;
                    cout << "\n\n<<All episodes finished>>\n";
                    auto e = high_resolution_clock::now();
                    auto d = duration_cast<std::chrono::microseconds>(e - s);
                    double microseco = d.count();
                    double seco = microseco / 1000000.0;
                    cout << "Total Erasped Time: " << seco << "(sec)";
                }
                isActive = false;
                _record(episode, agent_times);
            }
            else {
                // 衝突検出と処理
                for (int i = 0; i < num_agents; i++) {
                    int x_index = (int)(agent[i].current_pos[0] / 500) + 1;
                    int y_index = (int)(agent[i].current_pos[1] / 500) + 1;

                    if (x_index < 0 || x_index >= 72 || y_index < 0 || y_index >= 132) {
                        x_index = 0;
                        y_index = 0;
                    }

                    if (agent[i].get_distance_to_goal() >= 3000 && time_step >= 10 && biwako::icover_data[x_index][y_index] != 0) {
                        cout << "EPISODE" << episode << ": ObstacleCollision, time step: " << time_step << ", ship" << i + 1 << endl;
                        collisions.push_back({ agent[i].current_pos[0], agent[i].current_pos[1] });
                        agent[i].current_speed = 0;
                        agent[i].intention_speed = 0;
                        isActive = false;
                        for (int k = 0; k < num_agents; k++) {
                            agent_times.push_back(agent[k].time_step);
                        }
                        break;
                    }

                    for (int j = 0; j < num_agents; j++) {
                        if (i < j) {
                            if (time_step >= 10 && (agent[i].get_distance_to_other(&agent[j]) <= agent[i].safety_dom || 
                                agent[i].get_distance_to_other(&agent[j]) <= agent[j].safety_dom)) {
                                agent[i].collision = true;
                                agent[j].collision = true;
                                cout << "EPISODE" << episode << ": ShipCollision(ship" << i + 1 << " and ship" << j + 1 << ")" << endl;
                                cout << "timestep: " << time_step << "  distance: " << agent[i].get_distance_to_other(&agent[j]) << "m" << endl;
                                cout << "agent" << i + 1 << " x: " << agent[i].current_pos[0] << " y: " << agent[i].current_pos[1] << endl;
                                cout << "agent" << j + 1 << " y: " << agent[j].current_pos[0] << " y: " << agent[j].current_pos[1] << endl;
                                collisions.push_back({ (agent[i].current_pos[0] + agent[j].current_pos[0]) / 2.0, 
                                                      (agent[i].current_pos[1] + agent[j].current_pos[1]) / 2.0 });
                                agent[i].current_speed = 0;
                                agent[i].intention_speed = 0;
                                agent[j].current_speed = 0;
                                agent[j].intention_speed = 0;
                                isActive = false;
                                for (int k = 0; k < num_agents; k++) {
                                    agent_times.push_back(agent[k].time_step);
                                }
                                break;
                            }
                        }
                    }
                }
                time_step++;
            }
        }

        if (episode == max_episodes - 1) {
            double ave = 0;
            double SD = 0;
            for (auto& a : ave_timestep) {
                ave += a;
            }
            for (auto& s : SD_timestep) {
                SD += s;
            }
            string record_path = output_dir + "/result.csv";
            ofstream result_file(record_path, ios::app);
            result_file << "\n\ntotal\n\nave," << ave / cnt_success << ",SD," << SD / cnt_success << ",suc_rate," << (cnt_success) / (episode + 1.0);
            cout << "\n\ntotal\n\nave: " << ave / cnt_success << ", SD: " << SD / cnt_success << ", success_rate: " << (cnt_success) / (episode + 1.0);

            ofstream f(output_dir + "/collision.csv", ios::trunc);
            f.imbue(std::locale::classic());
            if (f.is_open()) {
                f << fixed << setprecision(15);
                f << "x,y" << endl;
                for (auto& c : collisions) f << c.first << "," << c.second << endl;
            }
            f.close();

            cout << "\n\n<<All episodes finished>>\n";
            auto e = high_resolution_clock::now();
            auto d = duration_cast<microseconds>(e - s);
            double microseco = d.count();
            double seco = microseco / 1000000.0;
            cout << "Total Erasped Time: " << seco << "(sec)";
            break;
        }
    }
}

//コンストラクタ
inline MoveAgents::MoveAgents(string file_name) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &now_c);
#else
    localtime_r(&now_c, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    output_dir = "data/" + oss.str();
#ifdef _WIN32
    std::filesystem::create_directories(output_dir);
#else
    system(("mkdir -p " + output_dir).c_str());
#endif

    string file_path = output_dir + "/trajectory.csv";
    ofstream tra_file(file_path, ios::trunc);
    if (tra_file.is_open()) {
        tra_file << "trajectory\n\n";
        tra_file.close();
    }

    string result_path = output_dir + "/result.csv";
    ofstream res_file(result_path, ios::trunc);
    if (res_file.is_open()) {
        res_file << "result\n\n";
        res_file.close();
    }

    string config_path = output_dir + "/config.txt";
    ofstream config_file(config_path, ios::trunc);
    if (config_file.is_open()) {
        config_file << "num_agents: " << num_agents << endl;
        config_file << "num_episodes: " << max_episodes << endl;
        config_file << "wind_scale: " << dict.WS << endl;
        config_file << "rain_scale: " << dict.PR << endl;
        config_file << "ship_size: " << agent[0].ship_size << endl;
        config_file << "algorithm: " << (USE_HDM_ALGORITHM ? "DSSA+HDM" : "DSSA+") << endl;
        config_file.close();
    }

    for (int i = 0; i < 10; i++) {
        powt[i] = pow(0.5, i);
    }

    for (int i = 0; i < agent.size(); ++i) {
        agent[i].ID = i;
    }

    run_DSSQ();
}

//ファイルへの結果の保存
inline void MoveAgents::_record(int episode, vector<double> agent_times) {
    string file_path = output_dir + "/trajectory.csv";
    ofstream tra_file(file_path, ios::app);
    tra_file.imbue(std::locale::classic());
    if (tra_file.is_open()) {
        tra_file << fixed << setprecision(15);
        tra_file << episode + 1 << "\n\n";
        for (int i = 0; i < num_agents; i++) {
            tra_file << i + 1 << ",";
            for (size_t index = 0; index < agent[i].trajectory.size(); index = index + 2) {
                tra_file << agent[i].trajectory[index] << " " << agent[i].trajectory[index + 1] << ",";
            }
            tra_file << "\n\n";
        }
    }

    string record_path = output_dir + "/result.csv";
    ofstream result_file(record_path, ios::app);
    if (result_file.is_open()) {
        result_file << "\n\nepisode," << episode << "\n\nagent, start_port, goal_port, timestep";
        for (int i = 0; i < num_agents; i++) {
            result_file << "\nagent" << i << "," << agent[i].start_port_ID << "," << agent[i].goal_port_ID << "," << agent_times[i];
        }
        result_file << ",,ave," << ave_timestep.back() << ",SD," << SD_timestep.back() << ",suc_rate," << (cnt_success) / (episode + 1.0) << "\n--------------------------------------------------";
    }
    result_file.close();
}

//2つの角度の差を求める
inline double MoveAgents::differenceBetweenAngle(double angle1, double angle2) {
    double dif = fabs(angle1 - angle2);
    if (dif > M_PI) {
        dif = 2 * M_PI - dif;
    }
    return dif;
}

//ドメインを構成する
inline void MoveAgents::create_VarDom() {
    for (int i = 0; i < num_agents; i++) {
        agent[i].updateNeighbors(i, agent);
        if (agent[i].notAtGoal) {
            double distance_to_goal = agent[i].get_distance_to_goal();
            if (distance_to_goal < agent[i].current_speed * dict.unit_change) {
                agent[i].notAtGoal = false;
                agent[i].current_speed = 0;
                agent[i].intention_speed = 0;
                agent[i].distance_offset = distance_to_goal;
            }
            else {
                create_VarDomEach(i);
                agent[i].intention_speed = agent[i].current_speed;
                agent[i].intention_speed_id = agent[i].current_speed_id;
                agent[i].intention_heading = agent[i].current_heading;
                agent[i].intention_heading_id = agent[i].current_heading_id;
            }
        }
    }
}

// 各船舶のドメインを構成
inline void MoveAgents::create_VarDomEach(int i) {
    double goal_heading = agent[i].get_goal_heading();

    // 選択可能かどうかのフラグ初期化
    for (int a = 0; a <= num_angles + 1; ++a) {
        for (int s = 0; s < num_speeds; ++s) {
            agent[i].isSelectable[a][s] = true;
        }
    }

    // 選択可能な絶対方向を計算
    for (int crs = 0; crs < num_angles; crs++) {
        double radian = agent[i].current_heading + agent[i].possible_angle_rad[crs];
        if (radian > M_PI) {
            agent[i].possible_heading[crs] = radian - 2 * M_PI;
        }
        else if (radian < -M_PI) {
            agent[i].possible_heading[crs] = 2 * M_PI - radian;
        }
        else {
            agent[i].possible_heading[crs] = radian;
        }
    }

    // 選択可能な速度を計算
    for (int spd = 0; spd < num_speeds; spd++) {
        double speed = agent[i].current_speed + agent[i].possible_change_speed[spd];
        if (agent[i].min_speed <= speed && speed <= agent[i].max_speed) {
            agent[i].possible_speed[spd] = speed;
        }
        else {
            agent[i].possible_speed[spd] = 1.0e5;
        }
    }

    // 流速を考慮した実際の航路を計算
    for (int crs = 0; crs < num_angles; crs++) {
        for (int spd = 0; spd < num_speeds; spd++) {
            if (agent[i].possible_speed[spd] != 1.0e5) {
                biwako::cal_actual(time_step, agent[i], crs, spd);
                biwako::cal_actual_tcpa(time_step, agent[i], crs, spd);
            }
            else {
                agent[i].isSelectable[crs][spd] = false;
            }
        }
    }

    // 目標方向についての処理 - HDM/DSSA+条件分岐
    for (int spd = 0; spd < num_speeds; spd++) {
        if (agent[i].possible_speed[spd] != 1.0e5) {
            if (USE_HDM_ALGORITHM) {
                // DSSA+HDM方式: biwako::cal_goal_intention関数を使用
                agent[i].possible_heading[num_angles] = biwako::cal_goal_intention(time_step, agent[i].current_pos[0], agent[i].current_pos[1], goal_heading, agent[i].possible_speed[spd]);
            } else {
                // 基本DSSA+方式: 目標方向を直接使用
                agent[i].possible_heading[num_angles] = goal_heading;
            }

            if (differenceBetweenAngle(agent[i].possible_heading[num_angles], agent[i].current_heading) <= agent[i].possible_angle_rad[num_angles - 1]) {
                biwako::cal_actual(time_step, agent[i], num_angles, spd);
                biwako::cal_actual_tcpa(time_step, agent[i], num_angles, spd);
            }
            else {
                agent[i].isSelectable[num_angles][spd] = false;
            }
        }
        else { 
            agent[i].isSelectable[num_angles][spd] = false; 
        }
    }
}

// DSSAを用いて各船舶が意思決定
inline void MoveAgents::perform_withDSSA() {
    for (int round = 0; round < max_round; round++) {
        for (int i = 0; i < num_agents; i++) {
            agent[i].satisfied = false;
            updateCostTable(i);
            compNextIntentionByDSSA(i);
        }
        int num_satisfied = 0;
        for (int i = 0; i < num_agents; i++) {
            if (agent[i].satisfied) {
                num_satisfied += 1;
            }
        }
        for (int i = 0; i < num_agents; i++) {
            if (agent[i].notAtGoal == true) {
                agent[i].intention_heading = agent[i].next_intention_heading;
                agent[i].intention_speed = agent[i].next_intention_speed;
            }
        }
        if (num_satisfied == num_agents) {
            break;
        }
        else if (round + 1 == max_round) {
            break;
        }
        for (int i = 0; i < num_agents; i++) {
            agent[i].current_heading = agent[i].intention_heading;
            agent[i].current_speed = agent[i].intention_speed;
        }
    }
}

//コストを計算 - HDM/DSSA+条件分岐追加
inline void MoveAgents::updateCostTable(int i) {
    for (int crs = 0; crs < num_angles + 1; crs++) {
        for (int spd = 0; spd < num_speeds; spd++) {
            if (agent[i].isSelectable[crs][spd]) {
                double n = 1.0;
                if (agent[i].get_distance_to_goal() <= 5000) {
                    n = 5000.0 / agent[i].get_distance_to_goal();
                    n = n * n * n;
                }
                
                double heading, speed;
                
                if (USE_HDM_ALGORITHM) {
                    // DSSA+HDM方式: actual値を使用
                    heading = agent[i].actual_heading[crs][spd];
                    speed = agent[i].actual_speed[crs][spd];
                } else {
                    // 基本DSSA+方式: possible値を使用
                    heading = agent[i].possible_heading[crs];
                    speed = agent[i].possible_speed[spd] * dict.unit_change;
                }

                agent[i].cost[crs][spd] = n * n * 0.5 * differenceBetweenAngle(heading, agent[i].get_goal_heading()) / M_PI + 
                                         n * n * 0.5 * fabs(agent[i].reference_speed * dict.unit_change - speed) / (agent[i].max_speed * dict.unit_change);

                for (int j = 0; j < num_agents; j++) {
                    if (agent[i].neighbors[j]) {
                        double tcpa = 0;
                        double dcpa = 0;
                        compTCPAandDCPA(i, crs, spd, j, &tcpa, &dcpa);
                        if (dcpa <= agent[i].safety_dom || dcpa <= agent[j].safety_dom) {
                            double risk = Agent::time_window / (tcpa + 0.00000000000001);
                            agent[i].cost[crs][spd] += risk;
                        }
                    }
                }

                auto st = high_resolution_clock::now();
                avoid(i, crs, spd);
                cnt_avoid++;
                auto en = high_resolution_clock::now();
                auto dur = duration_cast<nanoseconds>(en - st);
                double mic = dur.count();
                double se = mic / 1000000000.0;
                time_avoid += se;
            }
            else {
                agent[i].cost[crs][spd] = 100000000;
            }
        }
    }
}

//陸地を回避するためのコスト計算
inline void MoveAgents::avoid(int i, int crs, int spd) {
    for (int angle = (-1) * dict.num_angles; angle <= dict.num_angles; angle++) {
        double radian = agent[i].actual_heading[crs][spd];
        double speed = std::max(630.0, agent[i].actual_speed[crs][spd]);
        radian += dict.space_angle_deg * angle * M_PI / 180;

        for (int step = 1; step <= (agent[i].get_distance_to_goal() <= 4500 ? 1 : dict.num_steps); step++) {
            double next_pos[2] = { agent[i].current_pos[0] + cos(radian) * speed * step, 
                                   agent[i].current_pos[1] + sin(radian) * speed * step };
            int x_index = (int)(next_pos[0] / 500) + 1;
            int y_index = (int)(next_pos[1] / 500) + 1;

            if (500 <= next_pos[0] && next_pos[0] <= 35000 && 500 <= next_pos[1] && next_pos[1] <= 65000) {
                if (biwako::icover_data[x_index][y_index] != 0) {
                    agent[i].cost[crs][spd] += dict.cost * powt[abs(angle)] * powt[step];
                }
            }
            else {
                agent[i].cost[crs][spd] += dict.cost * powt[abs(angle)] * powt[step];
            }
        }
    }
}

//TCPA,DCPAを計算
inline void MoveAgents::compTCPAandDCPA(int i, int crs, int spd, int j, double* tcpa_ptr, double* dcpa_ptr) {
    double curPos_i[2] = { agent[i].current_pos[0], agent[i].current_pos[1] };
    double curPos_j[2] = { agent[j].current_pos[0], agent[j].current_pos[1] };

    double radian_i = agent[i].actual_heading_tcpa[crs][spd];
    double speed_i = agent[i].actual_speed_tcpa[crs][spd];
    double newPos_i[2] = { agent[i].current_pos[0] + cos(radian_i) * speed_i, 
                           agent[i].current_pos[1] + sin(radian_i) * speed_i };

    double radian_j = agent[j].actual_heading_tcpa[agent[j].intention_heading_id][agent[j].intention_speed_id];
    double speed_j = agent[j].actual_speed_tcpa[agent[j].intention_heading_id][agent[j].intention_speed_id];
    double newPos_j[2] = { agent[j].current_pos[0] + cos(radian_j) * speed_j, 
                           agent[j].current_pos[1] + sin(radian_j) * speed_j };

    double a = curPos_i[0] - curPos_j[0];
    double b = newPos_i[0] - curPos_i[0] - newPos_j[0] + curPos_j[0];
    double c = curPos_i[1] - curPos_j[1];
    double d = newPos_i[1] - curPos_i[1] - newPos_j[1] + curPos_j[1];

    double derivativeAtStarPoint = a * b + c * d;
    double secondDerivative = b * b + d * d;
    double derivativeAtEndPoint = secondDerivative + derivativeAtStarPoint;
    double squareDistanceAtCPA;

    if (secondDerivative > 0) {
        if (derivativeAtEndPoint <= 0) {
            *tcpa_ptr = Agent::time_window;
            squareDistanceAtCPA = a * a + c * c + 2 * derivativeAtStarPoint + secondDerivative;
        }
        else if (derivativeAtStarPoint >= 0) {
            *tcpa_ptr = 0;
            squareDistanceAtCPA = a * a + c * c;
        }
        else if (derivativeAtStarPoint < 0 && derivativeAtEndPoint > 0) {
            *tcpa_ptr = -1.0 * Agent::time_window * derivativeAtStarPoint / secondDerivative;
            squareDistanceAtCPA = a * a + c * c - derivativeAtStarPoint * derivativeAtStarPoint / secondDerivative;
        }
    }
    else if (derivativeAtStarPoint > 0) {
        *tcpa_ptr = 0;
        squareDistanceAtCPA = a * a + c * c;
    }
    else {
        *tcpa_ptr = Agent::time_window;
        squareDistanceAtCPA = a * a + c * c + 2 * derivativeAtStarPoint;
    }

    if (squareDistanceAtCPA < 0) {
        if (fabs(squareDistanceAtCPA) < 0.00001) {
            *dcpa_ptr = 0.0;
        }
        else {
            cout << "ship" << i + 1 << "Value of squareDistanceAtCPA is too small:" << squareDistanceAtCPA << endl;
            isActive = false;
        }
    }
    else {
        *dcpa_ptr = sqrt(squareDistanceAtCPA);
    }
}

//コストによって各船舶が意思決定
inline void MoveAgents::compNextIntentionByDSSA(int i) {
    double minCost = 1.0e9;
    int argmin_crs = 0;
    int argmin_spd = 0;

    double currentCost = agent[i].cost[agent[i].intention_heading_id][agent[i].intention_speed_id];

    for (int crs = 0; crs < num_angles + 1; crs++) {
        for (int spd = 0; spd < num_speeds; spd++) {
            if (minCost - agent[i].cost[crs][spd] > 0.00000001) {
                minCost = agent[i].cost[crs][spd];
                argmin_crs = crs;
                argmin_spd = spd;
            }
        }
    }

    if (argmin_crs == agent[i].intention_heading_id && argmin_spd == agent[i].intention_speed_id) {
        agent[i].satisfied = true;
    }

    if (currentCost > minCost) {
        agent[i].next_intention_heading = agent[i].possible_heading[argmin_crs];
        agent[i].next_intention_speed = agent[i].possible_speed[argmin_spd];
        agent[i].intention_heading_id = argmin_crs;
        agent[i].intention_speed_id = argmin_spd;
    }
}