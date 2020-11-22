/*
 * ImgTransform.cpp
 *
 *  Created on: Nov 22, 2020
 *      Author: E_CAD
 */

#include "ImgTransform.h"

ImgTransform::ImgTransform(){

	lastOpTime 		= 0;
	image_matrix 	= NULL;
	thisPtr			= NULL;
}

ImgTransform::~ImgTransform(){
	dl_matrix3du_free(image_matrix);
	image_matrix = NULL;
}

esp_err_t ImgTransform::begin(framesize_t framesize, pixformat_t pixformat){
	int p_width 	= 0;
	int p_height 	= 0;
	int p_channels	= 0;

	/*
	FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_QXGA,     // 2048*1536
	 */
	switch(framesize)
	{
		case FRAMESIZE_QVGA:
			p_width		= 320;
			p_height	= 240;
			break;
		case FRAMESIZE_VGA:
			p_width		= 640;
			p_height	= 480;
			break;
		case FRAMESIZE_QQVGA:
			p_width		= 160;
			p_height	= 120;
			break;
		default:
			return ESP_ERR_INVALID_ARG;
	}

	switch(pixformat)
	{
		case PIXFORMAT_GRAYSCALE:
			p_channels				= 1;
			pImage.bytes_per_pixel 	= 1;
			pImage.format			= FB_RGB888;		// no grayscale
			break;

		case PIXFORMAT_RGB888:
			p_channels				= 3;
			pImage.bytes_per_pixel 	= 3;
			pImage.format 			= FB_RGB888;
			break;

		default:
			return ESP_ERR_INVALID_ARG;
	}

	image_matrix = dl_matrix3du_alloc(1, p_width, p_height, p_channels); // Allocate RGB888 memory area for drawing
	if (!image_matrix) {
		Serial.println(PSTR("[ImgTransform::begin] dl_matrix3du_alloc failed"));
		return ESP_ERR_NO_MEM;
	}

    pImage.width 	= p_width;
    pImage.height	= p_height;
    pImage.data 	= image_matrix->item;

	thisPtr = &image_matrix;

	return ESP_OK;
}

int ImgTransform::width(){
	return image_matrix->w;
}

int ImgTransform::height(){
	return image_matrix->h;
}

esp_err_t ImgTransform::Fill(uint32_t color){

	fb_gfx_fillRect(&pImage, 0, pImage.width, pImage.width, pImage.height, color);

	return ESP_OK;
}



