#include "stdafx.h"
#include "fl_gl_3d.h"

Fl_gl_3d::Fl_gl_3d(int x,int y,int w, int h,  const char *l) : Fl_Gl_Window(x, y, w, h, l)
{
	camera = Camera();

	is_draw_ground = true;
	is_draw_axis = true;

	press_2 = false;
	press_3 = false;
}

void Fl_gl_3d::init_gl()
{
	glViewport(0,0,w(),h());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective( 45., float(w())/float(h()), 0.1, 1000.);
	//gluPerspective( 45., 800./600., 0.1, 1000.);
	//gluPerspective( 45., 1600./900., 0.1, 1000.);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Fl_gl_3d::init_gl_color()
{
	GLfloat light_ambient[] =  {0.0, 0.0, 0.0, 1.0};
	GLfloat light_diffuse[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_specular[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_position[] =  {1.0, 1.0, 1.0, 0.0};

	GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat mat_shininess = 70.0;

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	GLfloat ambient[] = {0.7, 0.7, 0.7, 1};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	//glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_NORMALIZE);

	glEnable(GL_POINT_SMOOTH);
}

void draw_ground()
{
	int count = 0;
	glBegin(GL_QUADS);
	for (int i = -80; i < 81; ++i) {
		for (int j = -80; j < 81; ++j) {
			if (count % 2 == 0)
				glColor3d(0.88, 0.88, 0.88);
				//glColor3d(0.90, 0.90, 0.90);
			else
				glColor3d(0.93, 0.93, 0.93);
				//glColor3d(0.95, 0.95, 0.95);

			glNormal3d(0.,0., 1.);

			glVertex3f(j, 0, i);
			glVertex3f(j, 0, i+1);
			glVertex3f(j+1, 0, i+1);
			glVertex3f(j+1, 0, i);
			count += 1;
		}
	}
	glEnd();
}

void draw_axis()
{
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(1.0,0,0);
	glVertex3f(0,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,1.0,0);
	glVertex3f(0,0,0);
	glColor3f(0,0,1);
	glVertex3f(0,0,1.0);
	glVertex3f(0,0,0);
	glEnd();
}

void Fl_gl_3d::setupFloor()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS,0x1,0x1);
	glStencilOp(GL_REPLACE,GL_REPLACE,GL_REPLACE);
	glStencilMask(0x1);		// only deal with the 1st bit
}

void drawRect(double x1, double x2, double y1, double y2)
{
	glBegin(GL_QUADS);
	glVertex3f(x1, 0.01, y1);
	glVertex3f(x1, 0.01, y2);
	glVertex3f(x2, 0.01, y2);
	glVertex3f(x2, 0.01, y1);
	glEnd();
}
void Fl_gl_3d::draw()
{
	if (!valid()) {
		valid(1);
		init_gl_color();
	}

	init_gl();

	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	camera.transform();

	if (is_draw_ground == true) {
		setupFloor();
		draw_ground();
	}
	if (is_draw_axis == true) {
		draw_axis();	
	}
	glColor3d(0., 0., 0.);
}

int Fl_gl_3d::handle(int event)
{
	int returned = 0;
	int mouseX = Fl::event_x();
	int mouseY = Fl::event_y();
	int pushButton = Fl::event_button();

	if (event == FL_PUSH)
	{
		if (pushButton == 1) {
			press_1 = true;
		}
		else if (pushButton == 2) {
			press_2 = true;
		}
		else if (pushButton == 3){
			press_3 = true;
		}
		returned = 1;
	}
	else if (event == FL_DRAG)
	{	
		int mouseDeltaX = mouseX - mousePrevX;
		int mouseDeltaY = mouseY - mousePrevY;

		if (press_1 == true && press_3 == false) {
			camera.rotateY -= cml::deg2rad(double(mouseDeltaX));
			camera.rotateX -= cml::deg2rad(double(mouseDeltaY));
		}
		else if (press_2 == false && press_3 == true){
			camera.centerX -= cos(camera.rotateY) * double(mouseDeltaX) / 32.0;
			camera.centerZ -= -sin(camera.rotateY) * double(mouseDeltaX) / 32.0;

			camera.centerX -= sin(camera.rotateY) * double(mouseDeltaY) / 32.0;
			camera.centerZ -= cos(camera.rotateY) * double(mouseDeltaY) / 32.0;
		}
		else if (press_2 == true){
			camera.distance -= double(mouseDeltaY) / 48.0;
			if (camera.distance < 0.0) 
				camera.distance = 0.0;
		}
		returned = 1;
	}
	else if (event == FL_MOUSEWHEEL)
	{
		camera.distance -= Fl::event_dy() / 2.0;
		if (camera.distance < 0.0) 
			camera.distance = 0.0;
		returned = 1;
	}
	else if (event == FL_RELEASE)
	{
		if (pushButton == 1) {
			press_1 = false;
		}
		else if (pushButton == 2) {
			press_2 = false;
		}
		else if (pushButton == 3){
			press_3 = false;
		}
		returned = 1;
	}

	if (returned == 1) {
		mousePrevX = mouseX;
		mousePrevY = mouseY;
		//redraw(); //떨리는 현상이 있어서 주석처리하였다.
		return 1;
	}
	else {
		return Fl_Gl_Window::handle(event);
	}
}

