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

#include <cassert>
#ifdef _DEBUG		
#define ASSERT(x) assert(x)
#else
#define ASSERT(x) 
#endif

#include <gtest/gtest.h>

using namespace std;
