/*
 * trigonometry.cpp
 *
 *  Created on: Nov 16, 2020
 *      Author: E_CAD
 */


#include "trigonometry.h"
//#include <math.h>

#define intPI	314

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

int trigonom::sin(int rad){

	// rad = PI = 314

	uint8_t i;					// lookup table index
	int sign = 1;
	int radtrunk = rad % intPI;

	if(radtrunk < 0){
		sign = -1;
	}

	radtrunk = abs(radtrunk);

	radtrunk = radtrunk>(intPI/2) ? intPI-radtrunk : radtrunk;

	i = radtrunk * buf_len / (intPI/2);

    return lookup[i]*sign;
}

int trigonom::cos(int rad){
	return sin(rad+(intPI/2));
}
