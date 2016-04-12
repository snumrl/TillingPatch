// +-------------------------------------------------------------------------
// | scatter_patches.h
// | 
// | Author: Manmyung Kim
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Manmyung Kim 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the TilingMotionPatch.
// |    TilingMotionPatch is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with TilingMotionPatch.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------
#pragma once

#include "../lua_tinker/lua_tinker.h"
#include "patch.h"
#include "collision_detect.h"
#include <list>

class Lego;
class Environment;

struct Dist_info
{
	int pa1;		// index of result_patch
	int pa2;		//
	int g1;			// index of point group 
	int g2;			//
	int p1;			// index of point
	int p2;			//
	double dist;	// distance
}; 

struct Box
{
	double x1;
	double x2;
	double z1;
	double z2;
	int t1;
	int t2;
};

class ScatterPatches
{
public:
	ScatterPatches(Lego *lg_);
	~ScatterPatches(void);

	void scatter_patches();
	void simple_stitch();
	void scatter_patches_random();
	bool scatter_a_patch_random(double& cost_value, const char* name, const int th_n, const double th_v);

	void scatter_a_patch_env(Patch &pa, const char* name, Environment& env);
	void scatter_a_patch_box(Patch &pa, const char* name, Box& box);

	void scatter_patches_simple();

	void make_move();
	void make_move_rec();
	void make_wall();
	void make_wall2();
	double delta_energy_function_greedy(Patch& patch, int& thres_num, double& thres_value);
	double delta_energy_function_annealing(const int patch_idx, Patch& patch, int& thres_num, double& thres_value);
	double energy_function();
	double energy_function(const double c, const double g, const double d);
	double trial_energy_function_greedy(Patch& patch, int& thres_num, double& thres_value);
	double trial_energy_function_annealing(Patch& patch, int& thres_num, double& thres_value);

	bool metrop(const double de, const double t);

	bool test_stitch_ok( Patch &pa1, lua_tinker::table &con, CollisionDetect &collision_detect, int &collision_num, int thres_num, list<Dist_info> &dist_info_list, double& costValue );
	void stitch_all(Patch &pa1, list<Dist_info> &dist_info_list );
	bool test_and_stitch( Patch &pa1, CollisionDetect &collision_detect, int &collision_num, int thres_num, double thres_value );
	bool stitch_to_dead( int pa2_i, int g2_i, lua_tinker::table &con, CollisionDetect &collision_detect );
	bool collision_and_stitch( Patch &pa1, CollisionDetect &collision_detect, int &collision_num, list<Dist_info>& dist_info_list);

	//void print_unable_dead_end();
	//bool is_unable_dead_end( int pa2_i, int g2_i );
	void print_dead_end();

	bool delete_patch(int pa_i);
	void simple_random();

	int settle( Patch &pa, vector<Patch> &patches, CollisionDetect &collision_detect, int thres_num );
	void scatter_test();
	void make_circle();
	void simple_metro();
	void make_sink();
	void make_circle2();
	Lego *lg;
	double ef_alpha;
	double ef_beta;

	bool stop;

	Box box;
	vector<string> stitch_motion_name[2];
};


vector<int> get_end_type_num(std::vector<Patch> &patches, int time_end = -10);
std::vector<int> get_random_vec( int size);

bool shake( std::vector<Patch> &patches, int shake_index, CollisionDetect &collision_detect );
bool collision_resolve( std::vector<Patch> &patches, CollisionDetect &collision_detect, set<int> &near_indexes,bool only_self_shake = false , int near_num_thres = 4 );
vector<int> get_end_type_num_patch( Patch &patch, int time_end /*= -10*/ );