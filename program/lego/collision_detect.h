// +-------------------------------------------------------------------------
// | collision_detect.h
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

#include "RAPID.H"
#include "VCollide.H"

#include "patch.h"
#include <set>

class CollisionDetect
{
public:
	CollisionDetect(void);
	~CollisionDetect(void);

	void init_patches(vector<Patch>& patches);
	bool detect( Patch& p1 );

	void init_patches_rp(vector<Patch>& patches);
	void init_patches_vc(vector<Patch>& patches);
	bool detect_rp( Patch& p1 );
	bool detect_vc( Patch& p1 );
	bool detect_vc_multi(Patch p1, map<int, Patch> &patches_stitched);
	std::set<std::pair<int, int>> init_and_detect_vc(vector<Patch>& patches);
	bool detect_vc_index( std::vector<Patch> &patches, std::set<int> &indexes );
	void add_vc(Patch &pa);
	void add_vc(std::vector<Patch> &patches, std::set<int> &indexes );
	set<int> detect_vc_index_all( std::vector<Patch> &patches, std::set<int> &indexes );
	void delete_vc( std::vector<Patch> &patches, std::set<int> &indexes );
	RAPID_model occupied[1000];
	//std::vector<VCollide*> vcs;
	VCollide vcs[1000];

	bool enable;
	bool use_vc;

	double R1[3][3], R2[3][3], T1[3], T2[3];
};

void make_collison_model_rp( ml::Posture & p, RAPID_model & b );
void make_collison_model_vc( ml::Posture & p, VCollide & vc );
void make_collision_model_patch_rp(int time, Patch& p, RAPID_model &b);
bool make_collision_model_patch_vc(int time, Patch& p, VCollide &vc);