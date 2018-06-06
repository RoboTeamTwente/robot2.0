/*
 * ballsensor.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Gebruiker
 */

#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

#include "i2c.h"
#include "gpio.h"
#include "../PuttyInterface/PuttyInterface.h"
#include "../kickchip/kickchip.h"
#include "../Geneva/geneva.h"

#define NOBALL -1
#define NOBALL_TIMEOUT 100 //TODO: find appropriate value for this?


void I2CTx(uint8_t tosend[], uint8_t length);
void I2CRx();

void printRawData(uint8_t data[]);
void printPosition(uint8_t data[]);
uint8_t isBallCentered();
void ballHandler(uint16_t x, uint16_t y);
void parseMessage();
void ballsensorInit();
void ballsensorReset();
int8_t ballsensorMeasurementLoop(uint8_t kick_enable, uint8_t chip_enable, uint8_t power);

void noBall();
int32_t getBallPos();
void resetKickChipData();

#endif /* BALLSENSOR_H_ */
