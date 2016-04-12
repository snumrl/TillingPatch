// +-------------------------------------------------------------------------
// | tgasave.h
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
//-------------------------------------------------------------
/// \file	tgasave.h
/// \date	9-feb-2005
/// \author	Rob Bateman
/// \brief	writes a 24 or 32 bit tga file to the disk.
//-------------------------------------------------------------

#ifndef TGA_SAVE__H__
#define TGA_SAVE__H__

	bool WriteTga(const char* filename,
				const unsigned w,
				const unsigned h,
				const unsigned bpp,
				const unsigned char* pixels);

	bool WriteBMP(const char* filename,
		const unsigned w,
		const unsigned h, 
		const unsigned bpp, 
		const unsigned char* pixels);

#endif
