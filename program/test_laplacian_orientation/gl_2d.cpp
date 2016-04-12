#include "gl_2d.h"

static void draw_coordinates()
{
	glColor3f (0.6, 0.6, 0.6);
	int size = 10;
	glBegin(GL_LINES);
	for (int i = -size; i <= size; ++i) {
		glVertex2d(i, size);
		glVertex2d(i, -size);
		glVertex2d(size, i);
		glVertex2d(-size, i);
	}
	glEnd();
}

static void draw_axis()
{
	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(1,0,0);
	glVertex3f(0,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,1,0);
	glVertex3f(0,0,0);
	glEnd();
	glLineWidth(1);
}

static void display(void)
{
	/* clear all pixels  */
	glClear (GL_COLOR_BUFFER_BIT);

	draw_coordinates();
	draw_axis();
	draw();	

	/* don't wait!  
	* start processing buffered OpenGL routines 
	*/
	glFlush ();
}

static void init (void) 
{
	/* select clearing color 	*/
	glClearColor (1.0, 1.0, 1.0, 0.0);

	/* initialize viewing values  */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0);
}

static void keyboard (unsigned char key, int x, int y)
{
	if (key == 27) 
		exit(0);
}

void glut_main()
{
	glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize (800, 800); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow ("");
	init ();
	glutDisplayFunc(display); 
	glutKeyboardFunc(keyboard);
	glutMainLoop();
}