// +-------------------------------------------------------------------------
// | execute.h
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

void execute(map<string, Patch> * patch_type, vector<Patch> * patch_type_unary);
void execute_ys( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );
void execute_mm( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );
void execute_picture( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );
void execute_kl( map<string, Patch> * patch_type, vector<Patch> * patch_type_unary );

void all_phase_tiling(vector<shared_ptr<Patch>> &, Env &, map<string, Patch> * , vector<Patch> * , int , int , int , vector<string> &, double k, double alpha, string &);
void tiling_by_example(vector<shared_ptr<Patch>> &patches, Env &env, const vector<shared_ptr<Patch> > &input_patches, const string example_name, const int character_num);