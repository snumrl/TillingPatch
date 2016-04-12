// +-------------------------------------------------------------------------
// | environment.h
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

#include "patch.h"



struct Box
{
	Box() {}
	Box(cml::transf &se3, cml::vector3 &length) { this->se3 = se3; this->length = length;}
	Box(cml::transf &se3, cml::vector3 &length, double time) { this->se3 = se3; this->length = length; this->time = time;}

	cml::transf se3;
	cml::vector3 length;
	double time;
};

struct Env
{
	Env();
	
	bool put_center(Patch &pa);
	void set_scatter_range( double x_min, double x_max, double z_min, double z_max, double time_min, double time_max );
	void set_scatter_range( double x_min_start, double x_max_start, double x_min_end, double x_max_end, double z_min_start, double z_max_start, double z_min_end, double z_max_end, double time_min, double time_max );

	bool rand_scatter(Patch &pa);
	bool scatter_range(Patch &pa, double x_min, double x_max, double z_min, double z_max, double t_min, double t_max );

	double get_x_min() const { return range.find("x_min")->second;}
	double get_x_max() const { return range.find("x_max")->second;}
	double get_z_min() const { return range.find("z_min")->second;}
	double get_z_max() const { return range.find("z_max")->second;}
	double get_t_min() const { return range.find("t_min")->second;}
	double get_t_max() const { return range.find("t_max")->second;}
	
	vector<Box> static_boxes;
	vector<vector<Box>> dynamic_boxes;
	map<string, double> range;
	map<string, bool> is_cyclic;
};

void write_patches_env( const vector<shared_ptr<Patch>> & patches, Env &env, const string &name);
void read_patches_env( vector<shared_ptr<Patch>> & patches, Env &env, const string &name);

