/*
 * ImgTransform.h
 *
 *  Created on: Nov 22, 2020
 *      Author: E_CAD
 */

#ifndef IMGTRANSFORM_H_
#define IMGTRANSFORM_H_

#include <Arduino.h>
#include <esp_camera.h>

#include "fb_gfx.h"
#include "fd_forward.h"
//#include "fr_forward.h"




class ImgTransform {

private:

	uint64_t lastOpTime;		// Last operation timer (ms)

	dl_matrix3du_t *image_matrix;

	fb_data_t pImage;

public:

	ImgTransform();
	~ImgTransform();
	dl_matrix3du_t *thisPtr;

	int 		width();
	int 		height();
	uint64_t 	lastOpMs();		// Return last operation time in milliseconds
	size_t 		len();

	esp_err_t 	FillFromFB(camera_fb_t pImage);		// Fills picture data from camerera's frame buffer
	esp_err_t 	FillFromImg(ImgTransform pImage);	// Fills picture data from another buffer
	esp_err_t 	Fill(uint32_t color);

	esp_err_t 	doBlur();							// Apply Gaussian blur to image

	esp_err_t	begin(framesize_t framesize, pixformat_t pixformat);	// init buffer
};


#endif /* IMGTRANSFORM_H_ */
