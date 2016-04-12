#pragma once
#include "cml/cml.h"
#include "LineEdit2D.h"
#include "../include/KLIB/Polyline.h"
#include <map>
using namespace std;
using namespace KLIB;
std::vector<std::map<int,int>> get_cons_rigid_2d_for_hybrid(const std::vector<cml::vector2d> &points, double thres = 0.027);
void multi_cages_setup( const int groupN, const vector<int>& multi_pointN, const vector<vector<cml::vector2d>>& multi_points, vector<vector<cml::vector2d>>& multi_cageCoords );
double multi_hybrid_edit2D(std::vector<std::vector<cml::vector2d>> &multi_points, const std::vector<Constraint> &cons, double rigid_thres /*= 0.027*/ );