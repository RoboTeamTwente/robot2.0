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

PuttyInterfaceTypeDef puttystruct;

#define ballsensor_i2caddr (uint16_t)(0x50 << 1)
#define MAX_DATA_SIZE 255
  uint8_t data[MAX_DATA_SIZE];
  uint8_t next_message_length;
  uint error;



uint8_t enable_command[13];
uint8_t enable_response[11];
uint8_t bootcomplete_response[19];
uint8_t config_command[30];
uint8_t set_freq_command[17];
uint8_t set_freq_response[10];
uint8_t measurement_rx[5];

void I2CTx(uint8_t tosend[]);
void I2CRx();

void printRawData(uint8_t data[]);
void printPosition(uint8_t data[]);
void ballHandler(uint16_t x, uint16_t y);
void parseMessage();
void ballsensorInit();
void ballsensorMeasurementLoop();

#endif /* BALLSENSOR_H_ */
