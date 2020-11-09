/*
 * Bitmap_buffer.cpp
 *
 *  Created on: May 10, 2020
 *      Author: E_CAD
 */

#include "Bitmap_buffer.h"


BitmapBuff::BitmapBuff(uint16_t width, uint16_t height){
	p_width = width ;
	p_height = height ;



	image_matrix = dl_matrix3du_alloc(1, p_width, p_height, 3); // Allocate RGB888 memory area for drawing
	if (!image_matrix) {
		Serial.println(PSTR("[BitmapBuff] dl_matrix3du_alloc failed"));
	}

    pImage.width = p_width;
    pImage.height = p_height;
    pImage.bytes_per_pixel = 3;
    pImage.format = FB_RGB888;
    pImage.data = image_matrix->item;


    buf_len = p_width*p_height*pImage.bytes_per_pixel;

	thisPtr = &pImage;

}


BitmapBuff::~BitmapBuff(){
	pImage.data = NULL;
	dl_matrix3du_free(image_matrix);
	image_matrix = NULL;
}

int BitmapBuff::width(){
	return p_width;
}

int BitmapBuff::height(){
	return p_height;
}

esp_err_t BitmapBuff::Fill(frame *area_frame, uint32_t color){
	int32_t	w = area_frame->X2 - area_frame->X1;
	int32_t h = area_frame->Y2 - area_frame->Y1;

	fb_gfx_fillRect(&pImage, area_frame->X1, area_frame->Y1, w, h, color);

	return ESP_OK;
}

esp_err_t BitmapBuff::Clear(){
	frame all_bitmap;

	all_bitmap.X1 = 0;
	all_bitmap.Y1 = 0;
	all_bitmap.X2 = p_width;
	all_bitmap.Y2 = p_height;

	return this->Fill(&all_bitmap, FACE_COLOR_BLACK);
}

esp_err_t BitmapBuff::HDR2Bitmap(HDR *fr_buf, frame *area_frame){	// Draws grayscale HDR image to bitmap buffer in area_frame window (crops if window too small)

	uint16_t trans_w = (area_frame->X2 - area_frame->X1);
	uint16_t trans_h = (area_frame->Y2 - area_frame->Y1);

	trans_w = trans_w<fr_buf->width  ? trans_w : fr_buf->width; // min of area frame or fr_buf size to be transferred
	trans_h = trans_h<fr_buf->height ? trans_h : fr_buf->height;

	if(trans_w<1 || trans_h<1) return ESP_FAIL;

	// Rescale HDR to 8bit bitmap matrix
	for (uint16_t y = 0; y < trans_h; y++)		//x and y is HDR coordinates
	for (uint16_t x = 0; x < trans_w; x++) {

		uint32_t i = (y * fr_buf->width + x); // fr_buf->buf adress from coordinates
		uint32_t j = ((y+area_frame->Y1) * pImage.width + (x+area_frame->X1))*3; // pImage adress from coordinates

		// rescale brightness
		uint32_t t = fr_buf->buf[i]-fr_buf->min;
		t *= 256;
		t /= (fr_buf->max - fr_buf->min)+1;

		pImage.data[j+0] = (uint8_t) t;	//R
		pImage.data[j+1] = (uint8_t) t;	//G
		pImage.data[j+2] = (uint8_t) t;	//B
	}
	return ESP_OK;
}

esp_err_t BitmapBuff::Bitmap2JPEG(JPEG *jpeg_Out, uint8_t quality){


	// release buffers
	if(jpeg_Out->buf != NULL){
		free(jpeg_Out->buf);
		jpeg_Out->buf = NULL;
	}

	if(!fmt2jpg(image_matrix->item, buf_len, image_matrix->w, image_matrix->h, PIXFORMAT_RGB888, quality, &jpeg_Out->buf, &jpeg_Out->buf_len)){
		Serial.printf(PSTR("[BitmapBuff] JPEG compression failed\r\n"));
	}
	else{
		Serial.printf(PSTR("[BitmapBuff] JPEG compression done. Jpeg size is %i bytes\r\n"), jpeg_Out->buf_len);
	}


	return ESP_OK;
}


