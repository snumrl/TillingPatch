//-------------------------------------------------------------
/// \file	tgasave.cpp
/// \date	9-feb-2005
/// \author	Rob Bateman
/// \brief	writes a 24 or 32 bit tga file to the disk.
//-------------------------------------------------------------
#include "stdafx.h"
#include "tgasave.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

/// the tga header
#pragma pack(push,1)
struct TGAHeader
{
	// sometimes the tga file has a field with some custom info in. This 
	// just identifies the size of that field. If it is anything other
	// than zero, forget it.
	unsigned char m_iIdentificationFieldSize;

	// This field specifies if a colour map is present, 0-no, 1 yes...
	unsigned char m_iColourMapType;

	// only going to support RGB/RGBA/8bit - 2, colour mapped - 1
	unsigned char m_iImageTypeCode;

	// ignore this field....0
	unsigned short m_iColorMapOrigin;

	// size of the colour map
	unsigned short m_iColorMapLength;

	// bits per pixel of the colour map entries...
	unsigned char m_iColourMapEntrySize;

	// ignore this field..... 0
	unsigned short m_iX_Origin;

	// ignore this field..... 0
	unsigned short m_iY_Origin;

	// the image width....
	unsigned short m_iWidth;

	// the image height.... 
	unsigned short m_iHeight;

	// the bits per pixel of the image, 8,16,24 or 32
	unsigned char m_iBPP;

	// ignore this field.... 0
	unsigned char m_ImageDescriptorByte;
};
#pragma pack(pop)


// bmp header
struct BMPFILEHEADER {
	unsigned char     bfType[2];
	unsigned long     bfSize;
	unsigned short    bfReserved1;
	unsigned short    bfReserved2;
	unsigned long     bfOffBits;
};

struct BMPINFOHEADER{
	unsigned long        biSize;
	long                 biWidth;
	long                 biHeight;
	unsigned short       biPlanes;
	unsigned short       biBitCount;
	unsigned long        biCompression;
	unsigned long        biSizeImage;
	long                 biXPelsPerMeter;
	long                 biYPelsPerMeter;
	unsigned long        biClrUsed;
	unsigned long        biClrImportant;
};

bool WriteBMP(const char* filename,
			  const unsigned w,
			  const unsigned h, 
			  const unsigned bpp, 
			  const unsigned char* pixels){
	FILE* file; 
	unsigned int size; 

	BMPFILEHEADER fileHeader;
	BMPINFOHEADER infoHeader;

	/*  make sure the file is there. */
	if ((file = fopen(filename, "wb"))==NULL) {
		printf("Could not open file for writing : %s\n",filename);
		return false;
	}

	/* compute the image size */
	size = w*h*3; 

	/* set up header info */
	fileHeader.bfType[0] = 'B';
	fileHeader.bfType[1] = 'M';
	fileHeader.bfSize = size + 54;//sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER);
	fileHeader.bfReserved1 = 0;
	fileHeader.bfReserved2 = 0;
	fileHeader.bfOffBits = 54;// * 8;//(sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER)) * 8;

	infoHeader.biSize = 40;//sizeof(BMPINFOHEADER);
	infoHeader.biWidth = w;
	infoHeader.biHeight = h;
	infoHeader.biPlanes = 1;
	infoHeader.biBitCount = 24;
	infoHeader.biCompression = 0;
	infoHeader.biSizeImage = 0;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;
	infoHeader.biClrUsed = 0;
	infoHeader.biClrImportant = 0;

	/* write header info */
	//fwrite(&fileHeader, sizeof(fileHeader), 1, file);
	fwrite(&(fileHeader.bfType), 2, 1, file);
	fwrite(&(fileHeader.bfSize), 4, 1, file);
	fwrite(&(fileHeader.bfReserved1), 2, 1, file);
	fwrite(&(fileHeader.bfReserved2), 2, 1, file);
	fwrite(&(fileHeader.bfOffBits), 4, 1, file);

	//fwrite(&infoHeader, sizeof(infoHeader), 1, file);
	fwrite(&(infoHeader.biSize), 4, 1, file);
	fwrite(&(infoHeader.biWidth), 4, 1, file);
	fwrite(&(infoHeader.biHeight), 4, 1, file);
	fwrite(&(infoHeader.biPlanes), 2, 1, file);
	fwrite(&(infoHeader.biBitCount), 2, 1, file);
	fwrite(&(infoHeader.biCompression), 4, 1, file);
	fwrite(&(infoHeader.biSizeImage), 4, 1, file);
	fwrite(&(infoHeader.biXPelsPerMeter), 4, 1, file);
	fwrite(&(infoHeader.biYPelsPerMeter), 4, 1, file);
	fwrite(&(infoHeader.biClrUsed), 4, 1, file);
	fwrite(&(infoHeader.biClrImportant), 4, 1, file);

	/* write image data */
	for (int i = 0; i < size; i += 3) {
		/* reverse RGB */
		fwrite(&(pixels[i+2]), 1, 1, file);
		fwrite(&(pixels[i+1]), 1, 1, file);
		fwrite(&(pixels[i+0]), 1, 1, file);
	}

	fclose(file); /* Close the file */

	/*  we're done. */
	return true;
}


bool WriteTga(const char* filename,
			  const unsigned w,
			  const unsigned h,
			  const unsigned bpp,
			  const unsigned char* pixels) {

	// a flag to see if writing 32 bit image
	bool rgba=false;

	// make sure format is valid (allows bits or bytes per pixel)
	switch(bpp) {
	case 4: case 32:
		rgba = true;
		break;
	
	case 3: case 24:
		rgba = false;
		break;

	default:
		std::cerr << "[ERROR] Unsupported bit depth requested" << std::endl;
		return false;
	}

	FILE* fp = fopen(filename,"wb");

	if(!fp) {
		std::cerr << "[ERROR] could not save file \"" 
				  << filename << "\"" << std::endl;
		return false;
	}

	// fill the file header with correct info....
	TGAHeader header;

	// wipe to 0
	memset(&header,0,sizeof(TGAHeader));
	
	// rgb or rgba image
	header.m_iImageTypeCode = 2;

	// set image size
	header.m_iWidth = w;
	header.m_iHeight = h;

	// set bits per pixel
	header.m_iBPP = rgba ? 32 : 24;

	// write header as first 18 bytes of output file
	fwrite(&header,sizeof(TGAHeader),1,fp);

	// get num pixels
	unsigned int total_size = w * h * (3 + (rgba?1:0));
	unsigned int this_pixel_start = 0;

	// loop through each pixel
	for( ; this_pixel_start != total_size; this_pixel_start += 3 ) {

		// get address of pixel data
		const unsigned char* pixel = pixels + this_pixel_start;

		// write as BGR
		fputc(pixel[2],fp);
		fputc(pixel[1],fp);
		fputc(pixel[0],fp);

		// if RGBA also write alpha value
		if (rgba) {
			fputc(pixel[3],fp);
			++this_pixel_start;
		}
	}

	fclose(fp);

	return true;		  
}
