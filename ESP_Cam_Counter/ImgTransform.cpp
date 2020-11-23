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
//---------------------- Gaussian blur --------------------------------------------------------
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


//---------------------------------------------------------------------------------------------
//---------------------- Gradient intensity non-maximal suppression ---------------------------
//---------------------------------------------------------------------------------------------
esp_err_t ImgTransform::doGradPass(int sensitivity){

	// Init operation timer
	lastOpTime = 0;
	uint64_t OpStartStamp = millis();

	if(image_matrix == NULL) return ESP_ERR_INVALID_STATE;					// Object not initialized

	// Allocate memory for temporary gradient buffers
	dl_matrix3du_t *image_matrix_grad_mag;									// Gradient magnitude buffer
	dl_matrix3du_t *image_matrix_grad_dir;									// Gradient direction buffer

	image_matrix_grad_mag = dl_matrix3du_alloc(1, image_matrix->w, image_matrix->h, 1);
	image_matrix_grad_dir = dl_matrix3du_alloc(1, image_matrix->w, image_matrix->h, 1);

	if (!image_matrix_grad_mag || !image_matrix_grad_dir) {
		Serial.println(PSTR("[ImgTransform::doGradPass] dl_matrix3du_alloc failed"));
		return ESP_ERR_NO_MEM;
	}



	int pix[3] = { 0,0,0 };
	int gradx = 0, grady = 0;
	int graddir = 0, grad = 0;
	double tempa = 0, temps = 0, tempr = 0;
	uint32_t i = 0;

	// Get pixels and calculate gradient and direction
	for (int x = 1; x <= image_matrix->w-1; x++) {
		for (int y = 1; y <= image_matrix->h-1; y++) {

			// Get source pixels to calculate the intensity and direction
			i = y * image_matrix->w + (x - 1); 	// imgblur address from coordinates
			pix[1] = image_matrix->item[i]; 		// pixel left

			i = (y - 1) * image_matrix->w + x; 	// imgblur address from coordinates
			pix[2] = image_matrix->item[i]; 		// pixel above

			i = y * image_matrix->w + x; 		// imgblur address from coordinates
			pix[0] = image_matrix->item[i]; 		// main pixel

			// get value for x gradient
			gradx = pix[0] - pix[1];

			// get value for y gradient
			grady = pix[0] - pix[2];

			// Calculate gradient direction
			// We want this rounded to 0,1,2,3 which represents 0, 45, 90, 135 degrees
			// graddir = (int)(abs(atan2(grady, gradx)) + 0.22) * 80;

				// atan2 approximation
				if (max(abs(gradx), abs(grady)) == 0) {
					tempa = 0;
				}
				else {
					tempa = min(abs(gradx), abs(grady)) / max(abs(gradx), abs(grady));
				}
				temps = tempa * tempa;

				tempr = ((-0.0464964749 * temps + 0.15931422) * temps - 0.327622764) * temps * tempa + tempa;

				// Now sort out quadrant (gradient direction)
				if (abs(grady) > abs(gradx)) tempr = 1.57079637 - tempr;
				if (gradx < 0) tempr = 3.14159274 - tempr;
				if (grady < 0) tempr = -tempr;
				graddir = (int)(abs(tempr) + 0.22) * 80;
				image_matrix_grad_dir->item[i] = graddir;

			// Calculate gradient magnitude
			// grad = (int)sqrt(gradx * gradx + grady * grady) * 2;
			// imggrad->item[i] = grad;

				// Get absolute values for both gradients
				gradx = abs(gradx);
				grady = abs(grady);
				// Calculate gradient length (hypotenuse=(short side * 0.414) + long side)
				if (gradx > grady) {
					grad = (grady * 414) / 1000 + gradx;
				}
				else {
					grad = (gradx * 414) / 1000 + grady;
				}
				image_matrix_grad_mag->item[i] = grad * 2;

		}
	}



	//---------------------------------------------------------------------------------------------
	// --------------------- Non-Maximal Suppression -----------------------
	// Definitions
	int graddir = 0, grad = 0;

	// Get each pixel
	for (int x = 2; x <= image_matrix->w - 2; x++) {
		for (int y = 2; y <= image_matrix->h - 2; y++) {

			i  = y * image_matrix_grad_mag->stride + x; 		// image_matrix_grad_mag address from coordinates

			int j0 = y     * image_matrix->stride + x; 			// image_matrix address from coordinates
			int j1 = 0;
			int j2 = 0;

			// First check that current pixel's gradient above the threshold
			if (image_matrix_grad_mag->item[i] >= sensitivity) {

				// Get gradient direction
				graddir = image_matrix_grad_dir->item[i];		// Remember this was multiplied by 80 for the sake of display



				// If angle = 0
				if (graddir == 0) {
					// Is pixel local maximal
					j1 = (y-1) * image_matrix->stride + x; 		// image_matrix address from coordinates
					j2 = (y+1) * image_matrix->stride + x; 		// image_matrix address from coordinates
				}

				// If angle = 45 degrees
				else if (graddir == 80) {
					// Is pixel local maximal
					j1 = (y-1) * image_matrix->stride + (x+1); 	// image_matrix address from coordinates
					j2 = (y+1) * image_matrix->stride + (x-1); 	// image_matrix address from coordinates
				}

				// If angle = 90 degrees
				else if (graddir == 160) {
					// Is pixel local maximal
					j1 = y * image_matrix->stride + (x-1); 		// image_matrix address from coordinates
					j2 = y * image_matrix->stride + (x+1); 		// image_matrix address from coordinates
				}

				// If angle = 135 degrees
				else if (graddir == 240) {
					// Is pixel local maximal
					j1 = (y-1) * image_matrix->stride + (x-1); 	// image_matrix address from coordinates
					j2 = (y+1) * image_matrix->stride + (x+1); 	// image_matrix address from coordinates
				}

				if (image_matrix_grad_mag->item[i] >= image_matrix_grad_mag->item[j1] && image_matrix_grad_mag->item[i] >= image_matrix_grad_mag->item[j2]) {
					// Write pixel to as max
					image_matrix->item[j0] = 255;
					// Suppress other two
					image_matrix->item[j1] = 0;
					image_matrix->item[j2] = 0;

				}
				else {
					// Suppress pixel
					image_matrix->item[j0] = 0;
				}

			}
			else {

				// Suppress pixel
				image_matrix->item[j0] = 0;
			}

		}
	}

	dl_matrix3du_free(image_matrix_grad_mag);		// Gradients will not be used any further
	dl_matrix3du_free(image_matrix_grad_dir);
	image_matrix_grad_mag = NULL;
	image_matrix_grad_dir = NULL;


	lastOpTime = millis() - OpStartStamp;

	return ESP_OK;
}



//---------------------------------------------------------------------------------------------
//---------------------- The Hough Transformation ---------------------------------------------
//------ By http://www.keymolen.com/2013/05/hough-transformation-c-implementation.html --------
//---------------------------------------------------------------------------------------------
esp_err_t ImgTransform::doHough(){

	#define DEG2RAD (314/180)
	trigonom trigonomFuncs(50);

	// Init operation timer
	lastOpTime = 0;
	uint64_t OpStartStamp = millis();

	if(image_matrix == NULL) return ESP_ERR_INVALID_STATE;					// Object not initialized

	// Allocate memory for result hough plane
	int _img_w = image_matrix->w;											// Source image dimensions
	int _img_h = image_matrix->h;

	int hough_h = round((sqrt(2.0) * (double)(_img_h>_img_w?_img_h:_img_w)) / 2.0);

	// Hough plane dimensions calculate from source image size:
	int _accu_h = hough_h * 2.0; 											// -ro -> +ro
	int _accu_w = 180;														// thetta 0..180


	dl_matrix3du_t *_accu 	= dl_matrix3du_alloc(1, _accu_w, _accu_h, 1);	// Hough space image transformation

	if (!_accu) {
		Serial.println(PSTR("[ImgTransform::doHough] dl_matrix3du_alloc failed"));
		return ESP_ERR_NO_MEM;
	}

	double center_x = _img_w/2;
	double center_y = _img_h/2;

	for(int y=0;y<_img_h;y++){
		for(int x=0;x<_img_w;x++){

			int i  = y * image_matrix->stride + x; 							// image_matrix address from coordinates

			if( image_matrix->item[i] > 250 ){								// if source pixel is bright
				for(int t=0;t<180;t++){										// thetta 0..180 in Hough plane

					int r = 0;
					r += (x - center_x) * trigonomFuncs.cos(t * DEG2RAD);
					r += (y - center_y) * trigonomFuncs.sin(t * DEG2RAD);

					int j =  (r + hough_h) * _accu->stride + t; 			// _accu Address from coordinates
					if(_accu->item[j]<252){
						_accu->item[j]++;									// add brightness to hough plane pixels
					}
					else{
						_accu->item[j]=255;									// limitate brightness on maximum level
					}
				}
			}
		}
	}

	// Hough space buffer will be the main image, and source image will be wiped out and memory released
	dl_matrix3du_free(image_matrix);
	image_matrix 	= _accu;
	_accu			= NULL;

	pImage.bytes_per_pixel 	= 1;
	pImage.format			= FB_RGB888;
	pImage.width 			= image_matrix->w;
	pImage.height			= image_matrix->h;
	pImage.data 			= image_matrix->item;

	thisPtr 				= &image_matrix;

	lastOpTime = millis() - OpStartStamp;

	return ESP_OK;
}
