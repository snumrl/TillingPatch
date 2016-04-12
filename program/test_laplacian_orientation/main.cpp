#include "LineEdit2D.h"
#include "gl_2d.h"

#include <cmath>

double pi = 3.1415925;
std::vector<cml::vector2d> points;
std::vector<cml::vector2d> points_edited;

void init()
{
	for (int i = 0; i <= 20; ++i)
	{
		double x = -pi + 2 * pi / 20 * i;
		points.push_back(cml::vector2d(x / 2,sin(x) / 5.0));
	}

	LineEdit2D le;
	le.set_points(points);
	le.set_cons_pos(0, points[0]);
	le.set_cons_pos(1, points[1]);
	le.set_cons_ori(19, cml::vector2d(0,-1));
	//le.set_constraint_position(20, cml::vector2d(0,1));
	le.edit();
	points_edited = le.get_points();
}

void draw()
{
	glPointSize(5);
	glColor3f (0.0, 0.0, 0.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); ++i)
		glVertex2d (points[i][0], points[i][1]);
	glEnd();
	glColor3f (0.0, 0.5, 1.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); ++i)
		glVertex2d (points_edited[i][0], points_edited[i][1]);
	glEnd();
}

int main(int argc, char** argv)
{
	init();
	glut_main();
	return 0;
}
