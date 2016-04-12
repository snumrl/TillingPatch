#include "cml_wrapper.h"
#include <cmath>

double cml::pi()
{
	return cml::constants<double>::pi();
}

double get_radian(const std::vector<cml::vector2> &vec2_v, int index, int width)
{
	int last_index = vec2_v.size() -1;

	int p_index = index - width;
	int n_index = index + width;

	if (p_index < 0)
		p_index = 0;
	if (n_index > last_index)
		n_index = last_index;

	cml::vector2 p_vec2 = vec2_v[p_index];
	cml::vector2 n_vec2 = vec2_v[n_index];

	return radian(n_vec2-p_vec2);
}

double radian(cml::vector2 v)
{
	if (v[1] >= 0)
		return acos(v[0] / length(v));
	else
		return cml::constants<double>::two_pi() - acos(v[0] / length(v));
}

double deg2Rad(double deg)
{
	return double(deg) / 180.0 * cml::constants<double>::pi();
}

double rad2Deg(double rad)
{
	return double(rad) / cml::constants<double>::pi() * 180.0;
}

cml::vector2 get_vector2(cml::vector3 v)
{
	return cml::vector2(v[2], v[0]);
}

cml::vector3 get_vector3( cml::vector2 v )
{
	return cml::vector3(v[1], 0.0, v[0]);
}

cml::SE2 getSE2(cml::vector2 trans_v, double angle)
{
	double cosT = cos(angle);
	double sinT = sin(angle);
	return cml::SO3(
		cosT, -sinT, trans_v[0],
		sinT, cosT, trans_v[1],
		0, 0, 1
		);
}

cml::SO3 getSO3_by_rotY(double angle)
{
	double c = cos(angle);
	double s = sin(angle);
	return cml::SO3(
		c, 0, s,
		0, 1, 0, 
		-s, 0, c
		);
}

cml::SO3 getSO3_by_rotX(double theta)
{
	double c = cos(theta);
	double s = sin(theta);

	return cml::SO3(
		1, 0, 0,
		0, c, -s,
		0, s, c
		);
}

cml::SO3 getSO3_by_rotZ(double theta)
{
	double c = cos(theta);
	double s = sin(theta);

	return cml::SO3(
		c, -s, 0,
		s, c, 0,
		0, 0, 1
		);
}

cml::SE3 getSE3ByTransV(double x, double y, double z)
{
	return cml::SE3(
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1
		);
}

cml::SE3 getSE3ByTransV(cml::vector3 v)
{
	return cml::SE3(
		1, 0, 0, v[0],
		0, 1, 0, v[1],
		0, 0, 1, v[2],
		0, 0, 0, 1
		);
}

cml::SE3 getSE3ByRotX(double theta)
{
	double c = cos(theta);
	double s = sin(theta);

	return cml::SE3(
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1
		);
}

cml::SE3 getSE3ByRotY(double theta)
{
	double c = cos(theta);
	double s = sin(theta);

	return cml::SE3(
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1
		);
}

cml::vector3 get_vec3(cml::SE3 &se3)
{
	return cml::vector3(se3(0,3), se3(1,3), se3(2,3));
}

cml::SE3 getSE3_interpolate( cml::SE3 &se3_1, cml::SE3 &se3_2, double ratio )
{
	cml::vector3 vec1= get_vec3(se3_1);
	cml::vector3 vec2= get_vec3(se3_2);
	cml::vector3 vec = vec1 * (1-ratio) + vec2 * ratio;
	
	cml::SO3 rot1 = SE3_to_SO3(se3_1);
	cml::SO3 rot2 = SE3_to_SO3(se3_2);

	cml::SO3 new_rot = rot1 * exp_SO3(ratio * log_SO3(inverse(rot1) * rot2));

	/*cml::vector3 rot1 = log_SO3(SE3_to_SO3(se3_1));
	cml::vector3 rot2 = log_SO3(SE3_to_SO3(se3_2));
	cml::vector3 rot = rot1 * (1-ratio) + rot2 * ratio;

	cml::SE3 se3 = getSE3(exp_SO3(rot), vec);*/

	cml::SE3 se3 = getSE3(new_rot, vec);
	return se3;
}

cml::SO3 SE3_to_SO3( cml::SE3 &se3 )
{
	return cml::SO3(
		se3(0,0),	se3(0,1),	se3(0,2),
		se3(1,0),	se3(1,1),	se3(1,2),
		se3(2,0),	se3(2,1),	se3(2,2)
		);
}

cml::SE3 SO3_to_SE3( cml::SO3 &so3 )
{
	return cml::SE3(
		so3(0,0),	so3(0,1),	so3(0,2),	0,
		so3(1,0),	so3(1,1),	so3(1,2),	0,
		so3(2,0),	so3(2,1),	so3(2,2),	0,
		0		,	0		,	0		,	1
		);
}

cml::SE3 getSE3( cml::SO3 &so3, cml::vector3 &pos )
{
	return cml::SE3(
		so3(0,0),	so3(0,1),	so3(0,2),	pos[0],
		so3(1,0),	so3(1,1),	so3(1,2),	pos[1],
		so3(2,0),	so3(2,1),	so3(2,2),	pos[2],
		0		,	0		,	0		,	1
		);
}

cml::vector3 between_vector(cml::vector3 a, cml::vector3 b)
{
	cml::vector3 crossV = cml::cross(a,b);
	double len_crossV= cml::length(crossV);
	if (len_crossV != 0.0)
		return atan2(double(len_crossV), double(cml::dot(a,b))) * crossV / len_crossV;
	else
		return crossV;
}

cml::vector2 rot_vector2(cml::vector2 v, double theta)
{
	return cml::matrix22f_c(cos(theta), -sin(theta), sin(theta), cos(theta)) * v;
}

double get_diff_angle(double to, double from)
{
	//angle -> matrix
	double a[4] = {cos(to), -sin(to), sin(to), cos(to)};
	double b[4] = {cos(from), -sin(from), sin(from), cos(from)};
	// c = inverse b
	double c[4] = {b[3], -b[1], -b[2], b[0]}; 

	// z = a * inverse b
	double z[4] = {
		a[0] * c[0] + a[1] * c[2],
		a[0] * c[1] + a[1] * c[3],
		a[2] * c[0] + a[3] * c[2],
		a[2] * c[1] + a[3] * c[3]
	};

	//error handling
	if (z[0] > 1)
		z[0] = 1.;
	else if(z[0] < -1)
		z[0] = -1.;
	
	//matrix -> angle
	double theta;
	if (z[2] >= 0)
		theta = acos(z[0]);
	else
		theta = -acos(z[0]);

	return theta;
}

cml::SE3 exp_SE3(cml::vector3 axis, double theta)
{
	return exp_SE3(axis * theta);
}

cml::SE3 exp_SE3(cml::vector3 axis)
{
	return SO3_to_SE3(exp_SO3(axis));
}

cml::SO3 exp_SO3(cml::vector3 axis, double theta)
{
	return exp_SO3(axis * theta);
}

cml::SO3 exp_SO3(cml::vector3 axis)
{
	double theta = length(axis);
	if (theta != 0.0)
		axis = axis / theta;
	
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	double c = cos(theta);
	double s = sin(theta);

	return cml::SO3(
		c + (1.0-c)*x*x,	(1.0-c)*x*y - s*z,	(1-c)*x*z + s*y,
		(1.0-c)*x*y + s*z,	c + (1.0-c)*y*y,    (1.0-c)*y*z - s*x,
		(1.0-c)*z*x - s*y,	(1.0-c)*z*y + s*x,	c + (1.0-c)*z*z
		);
}

cml::vector3 log_SO3( const cml::SO3 &so3 )
{
	double cosTheta = 0.5 * (so3(0,0) + so3(1,1) + so3(2,2) - 1.0);
	if (fabs(cosTheta) > 1.0 - 1e-6)
		return cml::vector3(0., 0., 0.);
	
	double theta = acos(cosTheta);
	double cof = theta / (2.0 * sin(theta));

	return cml::vector3(
		cof * (so3(2,1) - so3(1,2)),
		cof * (so3(0,2) - so3(2,0)),
		cof * (so3(1,0) - so3(0,1))
	);
}

double distance_line_to_point( const cml::vector3 &line_p, const cml::vector3 &line_v, const cml::vector3 &point )
{
	double t = dot(point - line_p, line_v) / dot(line_v, line_v);
	return length((line_p + t * line_v) - point);
}

double length_vector2(cml::vector2& v)
{
	return cml::length(v);
}

void setSE3( cml::SE3 &se3, cml::vector3 trans_v )
{
	se3(0,3) = trans_v[0];
	se3(1,3) = trans_v[1];
	se3(2,3) = trans_v[2];
}