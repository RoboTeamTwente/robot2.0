/*
 * userIO.h
 *
 *  Created on: May 23, 2018
 *      Author: Leon
 */

#ifndef USERIO_USERIO_H_
#define USERIO_USERIO_H_

#include <stdint.h>

/*	reads the address from the slider switch
 *
 */
int ReadAddress();
/*
 * uint is the value to display
 * n_leds chooses bitwise which leds to show the uint on, 0 means no edit
 */
void Uint2Leds(uint8_t uint, uint8_t n_leds);
/*	Toggle a single led given by
 *
 */
void ToggleLD(uint8_t LD_);
/*	Sets a LD to value on
 *
 */
void SetLD(uint8_t LD_, uint8_t on);

#endif /* USERIO_USERIO_H_ */
