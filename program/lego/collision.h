// +-------------------------------------------------------------------------
// | collision.h
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
#include "VCollide.H"
#include "patch.h"
#include "environment.h"
#include "intersect_obj.h"

bool collision_posture_obb( const ml::Posture &p1, const ml::Posture &p2 );
bool collision_posture( const ml::Posture &p1, const ml::Posture &p2 );

int collision_motion( const ml::Motion &m1, const ml::Motion &m2 );
int collision_patch( const Patch &pa1, const Patch &pa2 );
bool collision_patches_nopreprocess( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set);
bool collision_patches_nopreprocess_twopart( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set );
bool is_exist(int time, const Patch &pa);
int collision_motion_obb( const ml::Motion &m1, const ml::Motion &m2 );
int collision_patches_obb( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set );
int collision_patch_obb( const Patch &pa1, const Patch &pa2 );
bool collision_postures_obb( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, double maximum_radius = 1.1 );
bool collision_postures_obb_nocull( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, double maximum_radius = 1.1);
int collision_patches_obb_nocull( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set );
bool collision_patches_obb_two( const vector<shared_ptr<Patch>> & patches, int pa1, int pa2 );
bool collision_postures_boxes_obb( const vector<const ml::Posture *> ps1, const vector<const ml::Posture *> ps2, const vector<const Box *> boxes, double maximum_radius /*= 1.1 */ );
int collision_patches_env_obb( const vector<shared_ptr<Patch>> & patches, const set<int> &check_set, const Env & env );

obb bound_obb_posture(const ml::Posture &); // ???

void add_collision_motion(int time, const ml::Motion & m, VCollide &vc);
