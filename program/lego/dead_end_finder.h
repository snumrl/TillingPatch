// +-------------------------------------------------------------------------
// | dead_end_finder.h
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

class DeadEndFinder
{
public:
	DeadEndFinder(void);
	~DeadEndFinder(void);

	void set_endes(std::vector<Patch> &result_patches);
	pair<int,int> get_dead_end(std::vector<Patch> &result_patches);

	void update_result_patches( std::vector<Patch> & result_patches );


	std::vector<pair<int,int>> endes;
	int endes_index;
	bool first;

	int end_time;
};

