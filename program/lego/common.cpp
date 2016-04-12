#include "StdAfx.h"
#include "common.h"

std::vector<int> get_random_vec( int size )
{
	std::vector<int> rv;
	for (int i = 0; i < size; ++i)
		rv.push_back(i);
	random_shuffle(rv.begin(), rv.end());

	return rv;
}

double safe_double(double val)
{
	if (_isnan(val) || !_finite(val))
		return 0.0;
	else
		return val;
};	

int safe_int(int val)
{
	if (_isnan(val) || !_finite(val))
		return 0;
	else
		return val;
};	

