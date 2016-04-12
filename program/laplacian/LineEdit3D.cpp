#include "LineEdit3D.h"

LineEdit3D::LineEdit3D(void)
{
	only2D = false;
}

LineEdit3D::~LineEdit3D(void)
{
}

void LineEdit3D::set_points( std::vector<cml::vector3d> &points_ )
{
	points = points_;
}

void LineEdit3D::set_cons_pos( int i, cml::vector3d &position )
{
	cons_pos[i] = position;
}

double LineEdit3D::edit_xz()
{
	double error;

	LineEdit2D le2d;
	std::vector<cml::vector2d> points_2d;
	for (int i = 0; i < points.size(); ++i)
	{
		points_2d.push_back(cml::vector2d(points[i][2], points[i][0]));
	}
	le2d.set_points(points_2d);

	for ( std::map<int, cml::vector3d>::iterator it=cons_pos.begin() ; it != cons_pos.end(); it++ )
	{	
		int i = (*it).first;
		cml::vector3d &p = (*it).second;

		cml::vector2d p2d(p[2], p[0]);
		le2d.set_cons_pos(i, p2d);
	}

	for ( std::map<int, cml::vector3d>::iterator it=cons_ori.begin() ; it != cons_ori.end(); it++ )
	{	
		int i = (*it).first;
		cml::vector3d &ori = (*it).second;

		cml::vector2d p2d(ori[2], ori[0]);
		le2d.set_cons_ori(i, p2d);
	}

	le2d.set_cons_stationary(cons_stationary);
	
	error = le2d.edit();
	points_2d = le2d.get_points();

	for (int i = 0; i < points.size(); ++i)
	{
		points[i][0] = points_2d[i][1];
		points[i][2] = points_2d[i][0];
	}
	return error;
}

#include <cassert>
int get_k(std::vector<double> &ls, double deformed_l)
{
	for (int a = 0; a < ls.size() - 1; ++a)
	{
		if (ls[a] <= deformed_l && deformed_l < ls[a+1])
			return a;
	}
	if (deformed_l < ls[0])
		return 0;
	if (ls[ls.size()-1] <= deformed_l)
		return ls.size()-2;

	assert(false);
}

double LineEdit3D::edit_ly()
{
	double error;
	std::vector<cml::vector3d> points_bar  = points;

	LineEdit2D le2d;

	std::vector<double> ls;
	ls.push_back(0.0);
	for (int i = 1; i < points.size(); ++i)
	{
		double l_part = sqrt(pow(points[i][0] - points[i-1][0], 2) + pow(points[i][2] - points[i-1][2], 2));
		ls.push_back(ls[i-1] + l_part);
	}

	std::vector<cml::vector2d> points_2d;
	for (int i = 0; i < points.size(); ++i)
	{
		points_2d.push_back(cml::vector2d(ls[i], points[i][1]));
	}
	le2d.set_points(points_2d);

	for ( std::map<int, cml::vector3d>::iterator it=cons_pos.begin() ; it != cons_pos.end(); it++ )
	{	
		int i = (*it).first;
		cml::vector3d &p = (*it).second;

		cml::vector2d p2d(ls[i], p[1]);
		le2d.set_cons_pos(i, p2d);
	}

	le2d.set_cons_stationary(cons_stationary);

	error = le2d.edit();
	points_2d = le2d.get_points();
		
	for (int i = 0; i < points.size(); ++i)
	{
		double l = points_2d[i][0];
		int k = get_k(ls, l);
		double a = l - ls[k];
		double b = ls[k+1] - l;
		points[i][0] = (a * points_bar[k+1][0] + b * points_bar[k][0]) / (a + b);
		points[i][2] = (a * points_bar[k+1][2] + b * points_bar[k][2]) / (a + b);

		points[i][1] = points_2d[i][1];
	}
	return error;
}

double LineEdit3D::edit()
{
	double error = 0.0;
	error += edit_xz();
	if (only2D == false)
		error += edit_ly();
	return error;
}

void LineEdit3D::cons_rigid( double cons_rigid_value )
{
	throw std::exception("The method or operation is not implemented.");
}
