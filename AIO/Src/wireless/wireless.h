/*
 * wireless.h
 *
 *  Created on: Sep 29, 2018
 *      Author: pepijn
 */
#include <string.h>
#include "main.h"
#include "spi.h"
#include "packing.h"

#ifndef WIRELESS_H_
#define WIRELESS_H_

extern SPI_HandleTypeDef *spiHandle;
extern volatile roboData receivedRoboData;

int Wireless_newData();
void Wireless_newPacketHandler();
void Wireless_Init(char* ssid, char* pass, char* ip, int robot_id);
void Wireless_Send(roboAckData* data);


#endif /* WIRELESS_H_ */
