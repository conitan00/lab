#pragma once
using namespace std;
#include <string> 

struct Dict {

	static const int num_agents = 30;
	static const int num_episodes = 10000;
	string input_folder_name = "2009-04-04_PR10";
	string dates = "20090404";
	string WS = "1";
	string PR = "10";

	double probability_for_flip = 0.8;
	double xmax_m = 35500;
	double ymax_m = 65494.84;
	double m_deg_change[2] = { 80000.0, 119990.4 };
	double unit_change = 180;
	int time_space = 3;

	double cost = 4;
	int num_angles = 4;
	double space_angle_deg = 4;
	int num_steps = 5;
	double n_rate = 0.5;
	double a_rate = 0.5;

};

inline Dict dict;