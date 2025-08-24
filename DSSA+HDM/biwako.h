#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include "config.h"

// Forward declaration
class Agent;

using namespace std;

//---------------���i�΂̊�-----------------

class biwako {

public:

	static const int cell_size = 500; //�Z���̈�ӂ̒���(m)
	static const int num_cell_x = 71; //x�����̃Z����
	static const int num_cell_y = 131; //y�����̃Z����
	static const int num_data = 400; //�擾���闬���̃f�[�^��

	static const double lon_west; //���[�̌o�x
	static const double lat_south; //���[�̈ܓx
	static const double horizontal_size_m; //x�����̋���(m)
	static double const vertical_size_m; //y�����̋���(m)

	static int agent_root[Agent::num_agents][2]; //�e�D���̔����n���[�g

	static vector<vector<pair<int, int>>> pattern; // �D���̏o���n�p�^�[�����i�[

    static double FS_data[num_data + 1][(num_cell_x - 2) * (num_cell_y - 2) + 1][2]; //�����̃f�[�^
    static int icover_data[num_cell_x + 1][num_cell_y + 1]; //�n�`�̃f�[�^
	
	//-------------�`------------------

	struct Port{
		static const int num_ports = 18; //�`�̐�
		int ID; //�`��ID
		string name; //�`�̖��O
		double lon; //�`�̌o�x
		double lat; //�`�̈ܓx
	};

	static vector<Port> port;

	//---------------------------------

	biwako(); //�R���X�g���N�^

	//�D���̈ʒu�ɂ����闬���̌v�Z
	static void cal_FS(int min, double x, double y, double* x_flow_ptr, double* y_flow_ptr); 

	//���ۂ̌o�H�̌v�Z
	static void cal_actual(int time_step, Agent& agent, int crs, int spd);

	//tcpa�̌v�Z�ɂ�������ۂ̌o�H�̌v�Z
	static void cal_actual_tcpa(int time_step, Agent& agent, int crs, int spd);

	static double cal_goal_intention(int time_step, double cur_x, double cur_y, double goal_heading, double intention_speed);

};

// Implementation
#include <chrono>
#include "csv.h"
using namespace std::chrono;

// Static member definitions
inline double biwako::FS_data[num_data + 1][(num_cell_x - 2) * (num_cell_y - 2) + 1][2] = { 0 };
inline int biwako::icover_data[num_cell_x + 1][num_cell_y + 1] = { 0 };
inline int biwako::agent_root[Agent::num_agents][2] = { {1,3}, {3,1} };
inline const double biwako::lon_west = 135.853125;
inline const double biwako::lat_south = 34.977083;
inline const double biwako::horizontal_size_m = 35500;
inline const double biwako::vertical_size_m = 65494.84;
inline vector<vector<pair<int, int>>> biwako::pattern;
inline vector<biwako::Port> biwako::port;

//コンストラクタ
inline biwako::biwako() {

	//-----流速データの取得-----------------------------------------------------------------------------

	std::cout << "--------------\nFSdata input...\n";

	auto start = high_resolution_clock::now();

	string line;
	string x_flow;
	string y_flow;
	string str;
	stringstream ss;
	int n;
	string filename;
	stringstream filename_ss;
	ifstream ifs;

	for (int min = 1; min < num_data; min++) {

		int n = 1;

		int time_hour = min / 60; //時間
		int time_min = min % 60; //分

		stringstream filename_ss;
		filename_ss << "../HDM_PR_change/output/" << dict.input_folder_name << "/ave" << dict.dates << setfill('0') << setw(2) << time_hour
			<< setw(2) << time_min << ".csv";
		string filename = filename_ss.str();

		ifstream ifs(filename);
		if (!ifs) {
			cerr << "Error opening file: " << filename << endl;
			return; // ファイルが開けなかった場合はリターン
		}

		while (getline(ifs, line)) {
			stringstream ss(line);
			double x, y;
			ss >> x;
			ss >> y;
			FS_data[min - 1][n][0] = x;
			FS_data[min - 1][n][1] = y;
			n++;
		}
	}

	auto end = high_resolution_clock::now();
	auto duration = duration_cast<seconds>(end - start);

	std::cout << "finished   erasped time: " << duration.count() << "(sec)\n";

	//-----地形データの取得---------------------------------

	std::cout << "--------------\nterrain data input...\n";
	auto st = high_resolution_clock::now();

	string value;
	string str1;
	string filename1 = "input/icover.csv";
	ifstream ifs1(filename1);

	if (!ifs1) {
		cerr << "Error opening file: " << filename1 << endl;
		return; // ファイルが開けなかった場合はリターン
	}

	int y = 0;
	while (getline(ifs1, str1)) {
		std::istringstream i_stream(str1);
		int x = 0;
		while (getline(i_stream, value, ',')) {
			icover_data[x][y] = stoi(value);
			x++;
		}
		y++;
	}

	auto en = high_resolution_clock::now();
	auto dur = duration_cast<seconds>(en - st);

	std::cout << "finished   erasped time: " << dur.count() << "(sec)\n--------------\n";

	//-----港データの取得---------------------------------

	std::cout << "--------------\nport data input...\n";

	io::CSVReader<4> in("input/port_data.csv");
	in.read_header(io::ignore_extra_column, "ID", "name", "lon", "lat");

	int id;
	std::string name;
	double lon;
	double lat;

	while (in.read_row(id, name, lon, lat)) {
		Port p = { id, name, lon, lat };
		port.push_back(p);
	}

	std::cout << "finished   erasped time: " << dur.count() << "(sec)\n--------------\n";

	//-----出着地パターンの取得-----------------------------

	std::cout << "--------------\npattern data input...\n";

	string f_name = "input/pattern//" + to_string(dict.num_agents) + "agents_" + to_string(dict.num_episodes) + "patterns.csv";

	ifstream f_pat(f_name);
	if (!f_pat.is_open()) {
		cerr << "ファイルを開けませんでした。" << endl;
	}

	string line_pat;

	while (getline(f_pat, line)) {

		stringstream ss_pat(line);
		string cell_pat;
		vector<string> tmp;

		while (getline(ss_pat, cell_pat, ',')) {
			tmp.push_back(cell_pat);
		}

		vector<pair<int, int>> pat;
		for (int i = 0; i + 1 < tmp.size(); i += 2) {
			pair<int, int> t;
			t.first = stoi(tmp[i]);
			t.second = stoi(tmp[i + 1]);
			pat.push_back(t);
		}
		pattern.push_back(pat);
	}

	f_pat.close();

	std::cout << "finished   erasped time: " << dur.count() << "(sec)\n--------------\n";
}

//-----船舶の位置における流速の計算--------------------------------------------------------------------------
inline void biwako::cal_FS(int min, double x, double y, double* x_flow_ptr, double* y_flow_ptr) {

	if (1000 <= x && x <= 34500 && 1000 <= y && y <= 64500) {

		y = vertical_size_m - y;
		int x_pos = (int)x / cell_size;
		int y_pos = (int)y / cell_size;
		double x_pos_in_cell = x - cell_size * (double)x_pos;
		double y_pos_in_cell = y - cell_size * (double)y_pos;

		double x_flow1 = FS_data[min - 1][(y_pos - 1) * (num_cell_x - 2) + x_pos][0];
		double x_flow2 = FS_data[min - 1][(y_pos - 1) * (num_cell_x - 2) + x_pos + 1][0];
		double y_flow1 = FS_data[min - 1][(y_pos - 1) * (num_cell_x - 2) + x_pos][1];
		double y_flow2 = FS_data[min - 1][(y_pos) * (num_cell_x - 2) + x_pos][1];

		double x_flow_s = ((x_pos_in_cell * x_flow2) + ((cell_size - x_pos_in_cell) * x_flow1)) / cell_size;
		double y_flow_s = ((y_pos_in_cell * y_flow2) + ((cell_size - y_pos_in_cell) * y_flow1)) / cell_size;

		*x_flow_ptr = dict.unit_change * x_flow_s;
		*y_flow_ptr = (-1) * dict.unit_change * y_flow_s;
	}
	else {
		*x_flow_ptr = 0;
		*y_flow_ptr = 0;
	}
}

//-----実際の経路の計算-------------------------------------------------------------------
inline void biwako::cal_actual(int time_step, Agent& agent, int crs, int spd) {

	double intention_speed = agent.possible_speed[spd];
	double intention_heading = agent.possible_heading[crs];
	double cur_x = agent.current_pos[0];
	double cur_y = agent.current_pos[1];
	intention_speed *= dict.unit_change;
	int min = time_step * 3 + 1;

	double x_flow;
	double y_flow;
	double* x_flow_ptr = &x_flow;
	double* y_flow_ptr = &y_flow;
	cal_FS(min, cur_x, cur_y, x_flow_ptr, y_flow_ptr);
	double new_x = x_flow + intention_speed * cos(intention_heading);
	double new_y = y_flow + intention_speed * sin(intention_heading);

	double new_radian = atan2(new_y, new_x);
	agent.actual_heading[crs][spd] = new_radian;
	agent.actual_speed[crs][spd] = sqrt(new_x * new_x + new_y * new_y);
}

//-----tcpaの計算における実際の経路の計算-------------------------------------------------
inline void biwako::cal_actual_tcpa(int time_step, Agent& agent, int crs, int spd) {

	double intention_speed = agent.possible_speed[spd];
	double intention_heading = agent.possible_heading[crs];
	double cur_x = agent.current_pos[0];
	double cur_y = agent.current_pos[1];
	intention_speed *= dict.unit_change;
	int min = time_step * 3 + 1;

	double x_flow;
	double y_flow;
	double* x_flow_ptr = &x_flow;
	double* y_flow_ptr = &y_flow;
	cal_FS(min, cur_x, cur_y, x_flow_ptr, y_flow_ptr);
	double new_x = (x_flow + intention_speed * cos(intention_heading)) * 5;
	double new_y = (y_flow + intention_speed * sin(intention_heading)) * 5;

	double new_radian = atan2(new_y, new_x);
	agent.actual_heading_tcpa[crs][spd] = new_radian;
	agent.actual_speed_tcpa[crs][spd] = sqrt(new_x * new_x + new_y * new_y);
}

inline double biwako::cal_goal_intention(int time_step, double cur_x, double cur_y, double goal_heading, double intention_speed) {

	intention_speed *= dict.unit_change;
	double change_rate = (M_PI / 180) / 10;
	double x_flow;
	double y_flow;
	double* x_flow_ptr = &x_flow;
	double* y_flow_ptr = &y_flow;
	int min = time_step * 3 + 1;
	cal_FS(min, cur_x, cur_y, x_flow_ptr, y_flow_ptr);

	double intention_heading_tmp = goal_heading;
	int iter = 0;
	while (true) {
		iter++;
		double goal_x = intention_speed * cos(intention_heading_tmp) + x_flow;
		double goal_y = intention_speed * sin(intention_heading_tmp) + y_flow;
		double goal_heading_cal = atan2(goal_y, goal_x);
		double diff = goal_heading - goal_heading_cal;
		if (abs(diff) <= change_rate) {
			double intention_heading = intention_heading_tmp;
			return intention_heading;
		}
		if (diff > M_PI) { diff -= 2 * M_PI; }
		else if (diff < -M_PI) { diff += 2 * M_PI; }
		if (diff < 0) { intention_heading_tmp -= change_rate; }
		else if (diff >= 0) { intention_heading_tmp += change_rate; }

		if (iter >= 1000) {
			break;
		}
	}
	return intention_heading_tmp;
}

// Agent implementation (needs to be after biwako definition)
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
#include <numbers>

inline Agent::Agent() {
    time_step = 0;

    // ドメイン初期化
    possible_angle_deg = { -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45 };
    for (int i = 0; i < Agent::num_angles; i++) {
        possible_angle_rad.push_back(possible_angle_deg[i] * (M_PI / 180));
    }
}

//-----各エピソード開始時の初期化----------------------------------------------------------------------------------------------
inline void Agent::init(int episode) {
    trajectory.clear();
    trajectory.reserve(200);
    start_port_ID = biwako::pattern[episode][ID].first - 1;
    goal_port_ID = biwako::pattern[episode][ID].second - 1;
    current_pos[0] = (biwako::port[start_port_ID].lon - biwako::lon_west) * dict.m_deg_change[0]; //初期位置の設定(x)
    current_pos[1] = biwako::vertical_size_m - (biwako::port[start_port_ID].lat - biwako::lat_south) * dict.m_deg_change[1]; //初期位置の設定(y)
    init_pos[0] = (biwako::port[start_port_ID].lon - biwako::lon_west) * dict.m_deg_change[0]; //初期位置の設定(x)
    init_pos[1] = biwako::vertical_size_m - (biwako::port[start_port_ID].lat - biwako::lat_south) * dict.m_deg_change[1]; //初期位置の設定(y)
    goal_pos[0] = (biwako::port[goal_port_ID].lon - biwako::lon_west) * dict.m_deg_change[0]; //目標位置の設定(x)
    goal_pos[1] = biwako::vertical_size_m - (biwako::port[goal_port_ID].lat - biwako::lat_south) * dict.m_deg_change[1]; //目標位置の設定(y)

    current_heading_id = 9;
    current_speed_id = 4;
    intention_heading_id = 9;
    intention_speed_id = 4;
    notAtGoal = true;
    time_step = 0;
    current_heading = get_goal_heading(); //現在角度を目標角度に初期化
    trajectory.clear(); //船舶の軌跡の初期化
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

