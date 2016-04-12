// +-------------------------------------------------------------------------
// | fl_gl_3d.h
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

#include "util/ccml.h"

#include <FL/glut.H>
#include <GL/GLU.h> 

#include <FL/Fl_Gl_Window.H>

class Camera
{
public:
	double centerX;
	double centerY;
	double centerZ;
	double rotateY;
	double rotateX;
	double distance;

	Camera::Camera()
	{
		centerX = 0.0;
		centerY = 0.8;
		centerZ = 0.0;
		rotateY = cml::deg2rad(0.0);
		rotateX = cml::deg2rad(-15.0);
		distance = 7.0;
	}

	void set_2d()
	{
		centerX = 0.0;
		centerY = 0.0;
		centerZ = 0.0;
		rotateY = cml::deg2rad(-90.0);
		rotateX = cml::deg2rad(-90.0);
		distance = 5.0;
	}
	cml::matrix44d_c getSE3() 
	{
		cml::matrix44d_c se3_1 = cml::trans_transf(centerX, centerY, centerZ);
		cml::matrix44d_c se3_2 = cml::roty_transf(rotateY);
		cml::matrix44d_c se3_3 = cml::rotx_transf(rotateX);
		cml::matrix44d_c se3_4 = cml::trans_transf(0, 0, distance);
		cml::matrix44d_c se3 = se3_1 * se3_2 * se3_3 *se3_4;

		return se3;
	}

	void transform() {
		cml::matrix44d_c se3 = getSE3();
		glMultMatrixd(inverse(se3).data());
	}
};


class Fl_gl_3d : public Fl_Gl_Window 
{
public:
	Fl_gl_3d(int x,int y,int w, int h, const char *l=0);

	Camera camera;

	bool Is_draw_ground() const { return is_draw_ground; }
	void Is_draw_ground(bool val) { is_draw_ground = val; }

	bool Is_draw_axis() const { return is_draw_axis; }
	void Is_draw_axis(bool val) { is_draw_axis = val; }

protected:
	virtual int handle(int event);
	void init_gl();
	void init_gl_color();
	void setupFloor();
	virtual void draw();

	bool is_draw_ground;
	bool is_draw_axis;
	
	int mousePrevX;
	int mousePrevY;

	bool press_1;
	bool press_2;
	bool press_3;
};
