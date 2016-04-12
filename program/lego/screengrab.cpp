//-------------------------------------------------------------
/// \file	screengrab.cpp
/// \author	Rob Bateman
/// \date	9-feb-2005
/// \brief	some code to take a screen grab of an opengl window.
//-------------------------------------------------------------
#include "stdafx.h"
#include "screengrab.h"
#include "tgasave.h"
#include <GL/glut.h>
#include <stdio.h>
bool SaveScreenGrab(const char* filename) {

	// get some info about the screen size
	unsigned sw      =  800;
	unsigned sh      =  600;
	unsigned bpp     =  3;
	GLenum   format  =  (bpp==4) ? GL_RGBA : GL_RGB;


	// allocate memory to store image data
	unsigned char* pdata = new unsigned char[sw*sh*bpp];

	// read from front buffer
	glReadBuffer(GL_FRONT);

	// read pixel data
	glReadPixels(0,0,sw,sh,format,GL_UNSIGNED_BYTE,pdata);

	//printf("%f" ,pdata[0]); 

	// write data as a tga file
	//bool ret = WriteTga(filename,sw,sh,bpp,pdata);
	bool ret =  WriteBMP(filename,sw,sh,bpp,pdata);
	// clean up
	delete [] pdata;

	// done
	return ret;
}

