#include "stdafx.h"
#include "fl_movie.h"

void Fl_gl_3d_parent::draw()
{
	Fl_gl_3d::draw();
	((Fl_movie *) p_parent)->draw_();
}

int Fl_gl_3d_parent::handle(int event)
{
	int return1 = Fl_gl_3d::handle(event);
	int return2 = ((Fl_movie *) p_parent)->handle_(event);

	//Motion_edit_win 안에서 flush를 하며 화면 업데이트가 안되서 이렇게 한다.라고 적혀있었다.
	if (return2 == 1)
		flush();

	return (return1 || return2);
}

void staticTimerSlider(void* p)
{
	((Fl_movie*)p)->timer();
}

Fl_movie::Fl_movie(int x, int y, int w, int h, Movie * movie_, const char *l) : frame_rate(30), Fl_Window(x,y,w,h,l)
{
	movie = movie_;
	init(movie->num_frame());
}

Fl_movie::Fl_movie(int x, int y, int w, int h, const char *l) : frame_rate(30), Fl_Window(x,y,w,h,l) 
{
	
}

Fl_movie::Fl_movie() : frame_rate(30), Fl_Window(100,100,1024,786+40,0) 
{
	init(0);
	show();
}

void Fl_movie::init(int num_frame)
{
	if (num_frame == 0)
		num_frame = 1;
	fl_gl_3d_parent = new Fl_gl_3d_parent(0, 0, w(), h()-40, this);
	this->resizable(fl_gl_3d_parent);
	
	view_button = new Fl_Button(0, h()-40, 60, 40);
	view_button->label("@> / @||");
	view_button->callback(staticCallBack_view, (void*)this); 
	view_button->value(1);

	view_slider = new Fl_Value_Slider(60, h()-40, w()-60, 40);
	view_slider->type(FL_HORIZONTAL);
	view_slider->value(0);
	view_slider->step(1);
	view_slider->bounds(0, num_frame-1);
	view_slider->callback(staticCallBack_view, (void*)this);

	set_bound_frame(0, num_frame-1);
	
	staticTimerSlider(this);
	show();	
}

void Fl_movie::set_frame(int num_frame)
{
	view_slider->bounds(0, num_frame-1);
}

void Fl_movie::staticCallBack_view(Fl_Widget* w, void* p)
{
	((Fl_movie*)p)->CallBack(w);
}

void Fl_movie::CallBack(Fl_Widget* w)
{
	if( w == view_button)
	{
		if (view_button->value() == 1)
			view_button->value(0);
		else
			view_button->value(1);
	}
	else if (w == view_slider)
	{
		fl_gl_3d_parent->redraw();
	}
}

void Fl_movie::draw_()
{
	int frame = int(view_slider->value());
	movie->draw(frame);
}

int Fl_movie::handle_(int event)
{
	return 0;
}

void Fl_movie::timer()
{
	if (view_button->value() == 1) {
		int frame = int(view_slider->value());
		view_slider->value(frame+1);

		if (view_slider->value() > view_slider->maximum() || view_slider->value() > bound_frames[1]) 
			view_slider->value(bound_frames[0]);
	}
	fl_gl_3d_parent->redraw();
	idle();

	Fl::add_timeout(1.0 / double(frame_rate) * 0.83, (Fl_Timeout_Handler)staticTimerSlider, this);
}

void Fl_movie::set_bound_frame( int begin_frame, int end_frame )
{
	bound_frames[0] = begin_frame; 
	bound_frames[1] = end_frame;
	view_slider->value(begin_frame);
}

Fl_movies::Fl_movies(int x, int y, int w, int h, std::vector<Movie *> movies_, const char *l) :Fl_movie(x,y,w,h,l)
{
	movies = movies_;

	int num_frame = 0;
	for (int i = 0; i < movies.size(); ++i)
		if (num_frame < movies[i]->num_frame())
			num_frame = movies[i]->num_frame();

	init(num_frame);
}

void Fl_movies::draw_()
{
	int frame = int(view_slider->value());
	for (int i = 0; i < movies.size(); ++i)
	{
		if (frame <= movies[i]->num_frame() -1)
			movies[i]->draw(frame);
	}
}
