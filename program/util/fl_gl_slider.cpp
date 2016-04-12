#include "StdAfx.h"
#include "fl_gl_slider.h"

double deg2rad_( double deg )
{
	return deg / 180.0 * cml::constants<double>::pi();
}

cml::matrix44d_c trans_transf_( double x, double y, double z )
{
	return cml::matrix44d_c(1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1);
}

cml::matrix44d_c rotx_transf_( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix44d_c(
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1
		);
}

cml::matrix44d_c roty_transf_( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix44d_c(
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1
		);
}

void init_gl(int w, int h)
{
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective( 45., float(w)/float(h), 0.1, 1000.);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void init_gl_color()
{
	GLfloat light_ambient[] =  {0.0, 0.0, 0.0, 1.0};
	GLfloat light_diffuse[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_specular[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_position[] =  {1.0, 1.0, 1.0, 0.0};

	GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat mat_shininess = 40.0;

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	GLfloat ambient[] = {0.6, 0.6, 0.6, 1};
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

void setupShadow()
{
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_EQUAL,0x1,0x1);
	glStencilOp(GL_KEEP,GL_ZERO,GL_ZERO);
	glStencilMask(0x1);		// only deal with the 1st bit

	glPushMatrix();
	// a matrix that squishes things onto the floor
	//float sm[16] = {1,0,0,0, 0,0,0,0.0, 0,0,1,0, 0,0.0,0,1};
	float light1_x = 10.0;
	float light1_y = -10.0;
	float light1_z = 20.0;

	float sm[16] = {1,0,0,0, -(light1_x/light1_z) ,0,-(light1_y/light1_z),0, 0,0,1,0, 0,0,0,1};
	glMultMatrixf(sm);
	// draw in transparent black (to dim the floor)
}

void unsetupShadow()
{
	glPopMatrix();
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
}

void draw_ground()
{
	int count = 0;
	glBegin(GL_QUADS);
	for (int i = -80; i < 81; ++i) {
		for (int j = -80; j < 81; ++j) {
			if (count % 2 == 0)
				glColor3d(0.82, 0.82, 0.82);
			//glColor3d(0.90, 0.90, 0.90);
			else
				glColor3d(0.88, 0.88, 0.88);
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

void setupFloor()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS,0x1,0x1);
	glStencilOp(GL_REPLACE,GL_REPLACE,GL_REPLACE);
	glStencilMask(0x1);		// only deal with the 1st bit
}

Camera::Camera()
{
	centerX = 0.0;
	centerY = 0.8;
	centerZ = 0.0;
	rotateY = deg2rad_(0.0);
	rotateX = deg2rad_(-15.0);
	distance = 7.0;
}

void Camera::transform()
{

	cml::matrix44d_c se3_1 = trans_transf_(centerX, centerY, centerZ);
	cml::matrix44d_c se3_2 = roty_transf_(rotateY);
	cml::matrix44d_c se3_3 = rotx_transf_(rotateX);
	cml::matrix44d_c se3_4 = trans_transf_(0, 0, distance);
	cml::matrix44d_c se3 = se3_1 * se3_2 * se3_3 *se3_4;

	glMultMatrixd(inverse(se3).data());
}

Fl_Gl_3D::Fl_Gl_3D(int x,int y,int w, int h, void * const p_parent_) : Fl_Gl_Window(x, y, w, h), p_parent(p_parent_)
{
	camera = Camera();

	is_draw_ground = true;
	is_draw_axis = true;

	press_2 = false;
	press_3 = false;
}

void Fl_Gl_3D::draw()
{
	if (!valid()) {
		valid(1);
		init_gl_color();
	}

	init_gl(w(), h());

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
	
	glColor3d(0.3, 0.3, 0.3);
	glEnable(GL_LIGHTING);
	((Fl_Gl_Slider *) p_parent)->draw_internal();
	glDisable(GL_LIGHTING);
}

int Fl_Gl_3D::handle(int event)
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
	else if (event == FL_DRAG && (Fl::event_ctrl() == 0 || Fl::event_shift() == 0))
	{	
		int mouseDeltaX = mouseX - mousePrevX;
		int mouseDeltaY = mouseY - mousePrevY;

		if (press_1 == true && press_3 == false) {
			camera.rotateY -= deg2rad_(double(mouseDeltaX));
			camera.rotateX -= deg2rad_(double(mouseDeltaY));
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

	mousePrevX = mouseX;
	mousePrevY = mouseY;

	int returned_parent = ((Fl_Gl_Slider *) p_parent)->handle_main(event);
	if (returned == 1 || returned_parent == 1)
		return 1;
	else
		return Fl_Gl_Window::handle(event);
}

Fl_Gl_Slider::Fl_Gl_Slider(void) : Fl_Window(1024,768+40)
{
	init();
}

Fl_Gl_Slider::Fl_Gl_Slider( int x, int y, int w, int h, int min_frame, int max_frame, const char * name /*= 0*/ ) : Fl_Window(x,y,w,h+40,name)
{
	init(min_frame, max_frame);
}

Fl_Gl_Slider::Fl_Gl_Slider( int x, int y, int w, int h, const char * name /*= 0*/ ) : Fl_Window(x,y,w,h+40,name)
{
	init();
}

void Fl_Gl_Slider::init( int min_frame /*= 0*/, int max_frame /*= 0*/ )
{
	frame_rate = 30;

	fl_gl_3d = new Fl_Gl_3D(0, 0, w(), h()-40, this);
	this->resizable(fl_gl_3d);

	fl_button = new Fl_Button(0, h()-40, 60, 40);
	fl_button->label("@> / @||");
	fl_button->callback(staticCallBackSlider, (void*)this); 
	fl_button->value(1);

	fl_slider = new Fl_Value_Slider(60, h()-40, w()-60, 40);
	fl_slider->type(FL_HORIZONTAL);
	fl_slider->step(1);
	fl_slider->bounds(min_frame, max_frame);
	fl_slider->value(fl_slider->minimum());
	fl_slider->callback(staticCallBackSlider, (void*)this);

	staticTimerSlider(this);
	show();
}

Fl_Gl_Slider::~Fl_Gl_Slider(void)
{
	delete fl_gl_3d;
	delete fl_button;
	delete fl_slider;
}

void Fl_Gl_Slider::staticTimerSlider(void* p)
{
	((Fl_Gl_Slider*)p)->timer();
}

void Fl_Gl_Slider::timer()
{
	if (fl_button->value() == 1) {
		fl_slider->value(fl_slider->value()+1);
		
		if (fl_slider->value() > fl_slider->maximum() || fl_slider->value() < fl_slider->minimum()) 
			fl_slider->value(fl_slider->minimum());
	}
	fl_gl_3d->redraw();

	Fl::add_timeout(1.0 / double(frame_rate) * 0.83, (Fl_Timeout_Handler)staticTimerSlider, this);
}

void Fl_Gl_Slider::staticCallBackSlider(Fl_Widget* w, void* p)
{
	((Fl_Gl_Slider*)p)->CallBack(w);
}

void Fl_Gl_Slider::CallBack(Fl_Widget* w)
{
	if( w == fl_button)
	{
		if (fl_button->value() == 1)
			fl_button->value(0);
		else
			fl_button->value(1);
	}
	else if (w == fl_slider)
	{
		fl_gl_3d ->redraw();
	}
}

void Fl_Gl_Slider::draw_main( int frame )
{

}

int Fl_Gl_Slider::handle_main( int event )
{
	return 0;
}

void Fl_Gl_Slider::draw_internal()
{
	draw_main(int(fl_slider->value()));
}

void Fl_Gl_Slider::set_bounds( int min, int max )
{
	fl_slider->bounds(min, max);
	fl_slider->value(fl_slider->minimum());
}

Camera& Fl_Gl_Slider::get_camera()
{
	return fl_gl_3d->camera;
}
