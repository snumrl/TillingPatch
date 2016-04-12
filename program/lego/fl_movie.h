// +-------------------------------------------------------------------------
// | fl_movie.h
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

#include "fl_gl_3d.h"

#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

//Strategy 디자인 패턴 사용
class Movie
{
public:
	virtual void draw(int frame) = 0;
	virtual int num_frame() = 0;
};

class Fl_gl_3d_parent : public Fl_gl_3d
{
public:
	Fl_gl_3d_parent(int x,int y,int w, int h, void * const p_parent_, const char *l=0):			Fl_gl_3d(x, y, w, h, l), p_parent(p_parent_) {}

	void * p_parent;
	void draw();
	int handle(int event);
};

class Fl_movie : public Fl_Window 
{
public:
	Fl_movie(int x, int y, int w, int h, Movie * movie_, const char *l=0);
	Fl_movie(int x, int y, int w, int h, const char *l=0);
	Fl_movie();

	void init(int max_frame);
	void set_frame(int num_frame);

	virtual void draw_();
	virtual int handle_(int event);
	virtual void idle() {}
	void timer();

	void set_bound_frame(int start_frame, int end_frame);

	int Frame_rate() const { return frame_rate; }
	void Frame_rate(int val) { frame_rate = val; }

	Camera& get_camera() { return fl_gl_3d_parent->camera; }
	Fl_gl_3d_parent * fl_gl_3d_parent;
	
protected:
	static void staticCallBack_view(Fl_Widget* w, void* p);
	void CallBack(Fl_Widget* w);

	Fl_Button * view_button;
	Fl_Value_Slider *view_slider;
	Movie * movie;
	int frame_rate;

	int bound_frames[2];
};

class Fl_movies : public Fl_movie 
{
public:
	Fl_movies(int x, int y, int w, int h, std::vector<Movie *> movies_, const char *l=0);
	
	void draw_();

private:
	std::vector<Movie *> movies;
};

class Movie_vector2 : public Movie
{
public:
	Movie_vector2() {}

	void set_vector2(const std::vector<cml::vector2> &vec2) {
		m_vec2 = vec2;
	}

	void draw(int frame) {
		glColor3f(0., 0., 0.);
		glPointSize(6.0f);

		glBegin(GL_POINTS);\
			for (int i = 0 ; i< m_vec2.size() ; ++i)	{
				glVertex3d(m_vec2[i][1], 0.01, m_vec2[i][0]);
			}
			glEnd();
	}

	int num_frame() {
		return 1;
	}

private:
	std::vector<cml::vector2> m_vec2;
};