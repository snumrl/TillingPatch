// +-------------------------------------------------------------------------
// | get_patch.h
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


ml::Motion auto_motion(const char * fullname, int first, int last, int first_type = 1, int last_type = 1, int object_first = -1, int object_last = -1, double ground = 0.0);
void clip_copy_motion(ml::Motion& cliped_mot, const char * fullname, int first, int last, double ground = 0.0);

void get_patch( map<string,Patch> *patch_type);
shared_ptr<Patch> get_a_patch( const char* name, const map<string,Patch> &patch_type);
shared_ptr<Patch> get_a_patch( const char* name);
void get_patch_unary( vector<Patch> *patch_type_unary, map<string, Patch> * patch_type );
void get_patch_object( map<string,Patch> *patch_type);

void add_patch(map<string, Patch> * patch_type , const char * fullname, int first, int first_type, int last, int last_type, int object_first = -1, int object_last = -1, double ground_truth = 0.0);
void make_patch_kick( Patch & pa );
void make_patch_chicken(vector< shared_ptr<Patch> > &);