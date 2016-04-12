#pragma once

#include <FL/glut.H>
#include <GL/GLU.h> 

#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

#include <cml/cml.h>

class Camera
{
public:
	Camera();
	void transform();

	double centerX;
	double centerY;
	double centerZ;
	double rotateY;
	double rotateX;
	double distance;
};

class Fl_Gl_3D : public Fl_Gl_Window
{
public:
	Fl_Gl_3D(int x, int y, int w, int h, void * const p_parent_);

	void draw();
	int handle(int event);

	Camera camera;
	bool is_draw_ground;
	bool is_draw_axis;

	int mousePrevX;
	int mousePrevY;

	bool press_1;
	bool press_2;
	bool press_3;

	void *p_parent;
};

class Fl_Gl_Slider : public Fl_Window 
{
public:
	Fl_Gl_Slider(void);
	Fl_Gl_Slider(int x, int y, int w, int h, int min_frame, int max_frame, const char * name = 0);
	Fl_Gl_Slider(int x, int y, int w, int h, const char * name = 0);
	~Fl_Gl_Slider(void);

	virtual void draw_main(int frame);
	virtual int handle_main(int event);

	void set_bounds(int min, int max);
	Camera& get_camera();

	void draw_internal();
	
protected:
	void init(int min_frame = 0, int max_frame = 0);
	static void staticTimerSlider(void* p);
	void timer();
	static void staticCallBackSlider(Fl_Widget* w, void* p);
	void CallBack(Fl_Widget* w);

	Fl_Gl_3D * fl_gl_3d;
	Fl_Button * fl_button;
	Fl_Value_Slider *fl_slider;
	
	int frame_rate;
};

void setupShadow();
void unsetupShadow();