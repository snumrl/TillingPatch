// +-------------------------------------------------------------------------
// | cml_wrapper.h
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

#include <cml/cml.h>

namespace cml
{
	typedef cml::matrix44d_c SE3;
	typedef cml::matrix33d_c SO3;
	typedef cml::matrix33d_c SE2;
	typedef cml::vector3d vector3;
	typedef cml::vector2d vector2;
	typedef cml::quaternion<double, fixed<>, scalar_first> quater;

	double pi();
}

double radian(cml::vector2 v);
double get_radian(const std::vector<cml::vector2> &vec2_v, int index, int width = 1);

double deg2Rad(double deg);
double rad2Deg(double rad);

cml::vector2 get_vector2(cml::vector3 v);
cml::vector3 get_vector3(cml::vector2 v);

cml::SE2 getSE2(cml::vector2 trans_v, double angle);

cml::SO3 getSO3_by_rotY(double angle);
cml::SO3 getSO3_by_rotX(double angle);
cml::SO3 getSO3_by_rotZ(double angle);

cml::SE3 getSE3ByTransV(double x, double y, double z);
cml::SE3 getSE3ByTransV(cml::vector3 v);
cml::SE3 getSE3ByRotX(double theta);
cml::SE3 getSE3ByRotY(double theta);
cml::vector3 get_vec3(cml::SE3 &se3);
cml::SE3 getSE3_interpolate(cml::SE3 &se3_1, cml::SE3 &se3_2, double ratio);

void setSE3(cml::SE3 &se3, cml::vector3 trans_v);

cml::SO3 SE3_to_SO3(cml::SE3 &se3);
cml::SE3 SO3_to_SE3(cml::SO3 &so3);

cml::vector3 between_vector(cml::vector3 a, cml::vector3 b);
cml::vector2 rot_vector2(cml::vector2 v, double theta);

double get_diff_angle(double to, double from);

cml::SE3 exp_SE3(cml::vector3 axis, double theta);
cml::SE3 exp_SE3(cml::vector3 axis);
cml::SO3 exp_SO3(cml::vector3 axis, double theta);
cml::SO3 exp_SO3(cml::vector3 axis);
cml::vector3 log_SO3(const cml::SO3 &so3);

double distance_line_to_point(const cml::vector3 &line_p, const cml::vector3 &line_v, const cml::vector3 &point);

double length_vector2(cml::vector2& v); //generic algorithm 에서 사용하기 위해
cml::SE3 getSE3(cml::SO3 &so3, cml::vector3 &pos);