/*
 * Bitmap_buffer.h
 *
 *  Created on: May 10, 2020
 *      Author: E_CAD
 */

#ifndef BITMAP_BUFFER_H_
#define BITMAP_BUFFER_H_

#include <Arduino.h>
#include <esp_camera.h>

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"


#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)


typedef struct {
	uint8_t  *buf;		// JPEG data pointer
	size_t 	 buf_len;	// Buffer size
	uint16_t width; 	// HDR image width
	uint16_t height;	// HDR image height
} JPEG;

typedef struct {
	unsigned short *buf;// Bitmap data pointer
	size_t 	 buf_len;	// Buffer size

	uint16_t max; 		//Max pixel brightness
	uint16_t min; 		//Min pixel brightness

	uint16_t width; 	//HDR image width
	uint16_t height;	//HDR image height
} HDR;

typedef struct {
	uint16_t X1;
	uint16_t Y1;
	uint16_t X2;
	uint16_t Y2;
} frame;


// Pixelformat is fixed to RGB888
class BitmapBuff {

private:

	int p_width;
	int p_height;
	size_t buf_len;

	fb_data_t pImage;
	dl_matrix3du_t *image_matrix;

public:

	BitmapBuff *thisPtr;


	BitmapBuff(uint16_t width, uint16_t height);
	~BitmapBuff();

	int width();
	int height();
	size_t len();

	esp_err_t HDR2Bitmap(HDR *HDR_buf, frame *area_frame);	// Draws HDR image to bitmap buffer in area_frame window (crops if window too small)
	esp_err_t Bitmap2JPEG(JPEG *output, uint8_t quality);
	esp_err_t Clear();
	esp_err_t Fill(frame *area_frame, uint32_t color);

};

//extern BitmapBuff Bitmap;




#endif /* BITMAP_BUFFER_H_ */
