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

bool isNrfInitialized;
uint8_t localRobotID; //"uninitialized"
uint LastPackageTime;
bool wireless_irq_prev;
bool kickchip_command;

bool Wireless_newData();
void Wireless_newPacketHandler();
void Wireless_Init(uint8_t address, uint8_t RADIO_CHANNEL);
void Wireless_Deinit();

#endif /* WIRELESS_WIRELESS_H_ */
