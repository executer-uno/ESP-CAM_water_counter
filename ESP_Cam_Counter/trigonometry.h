/*
 * trigonometry.h
 *
 *  Created on: Nov 16, 2020
 *      Author: PBAZH
 */

#ifndef TRIGONOMETRY_H_
#define TRIGONOMETRY_H_

#include <Arduino.h>


class trigonom {

private:

	uint8_t buf_len;

	int *lookup;

public:



	trigonom(uint8_t precision); // precision defines size of lookup table in ram
	~trigonom();

	int sin(int rad);	// argument is radiand multiplyed by 100. Pi=314. Return value is multiplyed by 100: cos(0)=100
	int cos(int rad);	// argument is radiand multiplyed by 100. Pi=314. Return value is multiplyed by 100: cos(0)=100


};



#endif /* TRIGONOMETRY_H_ */
