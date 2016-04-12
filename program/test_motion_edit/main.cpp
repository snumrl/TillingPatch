// test_motion_edit.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "motions_viewer.h"

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();

	return Fl::run();
}

