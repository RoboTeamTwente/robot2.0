/*
 * wireless.h
 *
 *  Created on: May 14, 2018
 *      Author: Gebruiker
 */

#ifndef WIRELESS_WIRELESS_H_
#define WIRELESS_WIRELESS_H_

#include <stdbool.h>
#include "../PuttyInterface/PuttyInterface.h"
#include "myNRF24.h"
#include "roboNRF24.h"
#include "debugNRF24.h"

uint8_t isNrfInitialized = 0;
uint8_t localRobotID = 0xff; //"uninitialized"
uint LastPackageTime = 0;
uint8_t wireless_irq_prev = 0;
uint8_t kickchip_command = 0;

bool Wireless_newData();
void Wireless_newPacketHandler();
void Wireless_Init();

#endif /* WIRELESS_WIRELESS_H_ */
