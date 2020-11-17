/*
 * trigonometry.cpp
 *
 *  Created on: Nov 16, 2020
 *      Author: E_CAD
 */


#include "trigonometry.h"
//#include <math.h>


trigonom::trigonom(uint8_t precision){

	buf_len = precision;

	lookup = new int [precision];
	if (lookup == nullptr) {
	  // error assigning memory. Take measures.
	}

	for (int i = 1; i <= buf_len; i++)
	{
		lookup[i-1] = std::sin(PI * i/ buf_len / 2); // fill lookup table with values of sin() in range from 0 to PI/2
	}

}


trigonom::~trigonom(){

	delete[] lookup;

}


int trigonom::cos(int rad){
	return sin(rad+PI/2);
}

int trigonom::sin(int rad){
	return rad;
}
