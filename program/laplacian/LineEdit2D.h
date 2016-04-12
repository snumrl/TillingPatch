#pragma once

#include "sqpsolver.h"
#include <map>

double edit2D( std::vector<cml::vector2d> &points, std::map<int, cml::vector2d> &cons_pos, std::map<int, cml::vector2d> &cons_ori, std::vector<bool> &cons_rigid = std::vector<bool>());
std::vector<bool> get_cons_rigid_2d(const std::vector<cml::vector2d> &points, double thres = 0.027);

struct Constraint
{
	string type;
	int m_int[4];
	double m_double[2];
	cml::vector2d m_vector2d[2];
};

double multi_edit2D( std::vector<std::vector<cml::vector2d>> &multi_points,const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ );
double multi_edit2D_time( std::vector<std::vector<cml::vector2d>> &multi_points,const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ );