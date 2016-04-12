// +-------------------------------------------------------------------------
// | stdafx.h
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
// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <iterator>
#include <algorithm>
#include <numeric>
#include <fstream>

#include <cassert>
#ifdef _DEBUG		
#define ASSERT(x) assert(x)
#else
#define ASSERT(x) 
#endif

extern "C"
{
#include "../lua/lua.h"
#include "../lua/lualib.h"
#include "../lua/lauxlib.h"
};

#include <gtest/gtest.h>
#include "common.h"

using namespace std;