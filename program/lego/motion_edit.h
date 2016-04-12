// +-------------------------------------------------------------------------
// | motion_edit.h
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
#include "LineEdit2D.h"
#include "motion/ml.h"

double motion_edit(ml::Motion &motion, std::map<int, double> cons_time, std::map<int, cml::vector3d> &cons_pos, std::map<int, cml::vector3d> &cons_ori, double rigid_thres = 0.027);

double multi_motion_edit( vector<ml::Motion*> &motions, const vector<Constraint> &constraints, double rigid_thres = 0.027);
double multi_motion_edit(vector<ml::Motion> &motions, const vector<Constraint> &constraints, double rigid_thres = 0.027);

void add_cons_pos( vector<Constraint> &cons, int group, int index, cml::vector3d v );
void add_cons_dir( vector<Constraint> &cons, int group, int index, cml::vector3d v );
void add_cons_rel_pos_oneside(vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 );
void add_cons_rel_pos(vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 );
void add_cons_same_pos( vector<Constraint> &cons, int group1, int index1, int group2, int index2 );
void add_cons_same_dir( vector<Constraint> &cons, int group1, int index1, int group2, int index2);
void add_cons_same_pos_dir_time( vector<Constraint> &cons, int group1, int index1, int group2, int index2);
void add_cons_pin( vector<Constraint> &cons, int group, const ml::Motion & motion, bool last );
void add_cons_pin( vector<Constraint> &cons, int group, const ml::Motion & motion, int index );
void add_cons_time( vector<Constraint> &cons, int group, int index, double t );
void add_cons_same_time( vector<Constraint> &cons, int group1, int index1, int group2, int index2 );
void add_cons_rel_pos_same_time( vector<Constraint> &cons, const vector<ml::Motion> &motions, int group1, int index1, int group2, int index2 );
void add_phase1_info( vector<Constraint> &cons);