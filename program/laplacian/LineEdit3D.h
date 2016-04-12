#pragma once

#include "LineEdit2D.h"

class LineEdit3D
{
public:
	LineEdit3D(void);
	~LineEdit3D(void);

	void set_points(std::vector<cml::vector3d> &points_);
	void set_cons_pos(int i, cml::vector3d &position);
	void set_cons_stationary(std::vector<std::pair<int, int> > &constraint_stationary_) { cons_stationary = constraint_stationary_; }
	double edit();
	std::vector<cml::vector3d> & get_points() {return points;}

	double edit_xz();
	double edit_ly();
	void cons_rigid( double cons_rigid_value );
	std::vector<cml::vector3d> points;
	std::map<int, cml::vector3d> cons_pos;
	std::map<int, cml::vector3d> cons_ori;
	std::vector<std::pair<int, int> > cons_stationary;
	bool only2D;
};