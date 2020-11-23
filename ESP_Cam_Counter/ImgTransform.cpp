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

	if(image_matrix == NULL) return ESP_ERR_INVALID_STATE;					// Object not initialized

	fb_gfx_fillRect(&pImage, 0, pImage.width, pImage.width, pImage.height, color);

	return ESP_OK;
}

//---------------------------------------------------------------------------------------------
// --------------------- Gaussian blur --------------------------------------------------------
//---------------------------------------------------------------------------------------------
esp_err_t ImgTransform::doBlur(){

	// Init operation timer
	lastOpTime = 0;
	uint64_t OpStartStamp = millis();

	if(image_matrix == NULL) return ESP_ERR_INVALID_STATE;					// Object not initialized

	// Allocate memory for initial image buffer
	dl_matrix3du_t *image_matrix_new;
	image_matrix_new = dl_matrix3du_alloc(1, image_matrix->w, image_matrix->h, image_matrix->c); // Allocate RGB888 memory area for drawing
	if (!image_matrix_new) {
		Serial.println(PSTR("[ImgTransform::doBlur] dl_matrix3du_alloc failed"));
		return ESP_ERR_NO_MEM;
	}
	// Copy initial image to temporary buffer for processing
	dl_matrix3du_slice_copy(image_matrix_new, image_matrix, 0, 0, image_matrix->w, image_matrix->h);

	// Definitions
	uint32_t blurpixel=0;
	uint32_t pixelweight = 0;
	uint32_t i = 0;
	uint32_t j = 0;

	// Define gaussian blur weightings array
	int weighting[5][5] =
	{
	{ 2, 4, 5, 4, 2},
	{ 4, 9,12, 9, 4},
	{ 5,12,15,12, 5},
	{ 4, 9,12, 9, 4},
	{ 2, 4, 5, 4, 2}
	};

	// Get each pixel and apply the Gaussian blur filter
	for (int x = 2; x <= image_matrix_new->w - 2; x++) {
		for (int y = 2; y <= image_matrix_new->h - 2; y++) {

			// Clear blurpixel
			blurpixel = 0;

			// +- 2 for each pixel and calculate the weighting
			for (int dx = -2; dx <= 2; dx++) {
				for (int dy = -2; dy <= 2; dy++) {
					pixelweight = weighting[dx+2][dy+2];

					i = ((y+dy) * image_matrix_new->stride + (x+dx)); // image_matrix Address from coordinates

					// Apply weighting
					blurpixel = blurpixel + image_matrix_new->item[i] * pixelweight;
				}
			}
			// Write pixel to blur image
			j = y * image_matrix->stride + x; // image_matrix Address from coordinates
			image_matrix->item[j] = (int)(blurpixel / 159);

		}
	}

	// release memory with buffered initial image
	dl_matrix3du_free(image_matrix_new);
	image_matrix_new = NULL;

	lastOpTime = millis() - OpStartStamp;

	return ESP_OK;
}
