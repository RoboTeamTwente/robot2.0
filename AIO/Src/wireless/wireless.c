/*
 * wireless.c
 *
 *  Created on: May 14, 2018
 *      Author: Gebruiker
 */

#include "wireless.h"

bool isNrfInitialized = 0;
uint8_t localRobotID = 0xff; //"uninitialized"
uint LastPackageTime = 0;
bool wireless_irq_prev = 0;
bool kickchip_command = 0;

//detect falling edges on IRQ pin
bool Wireless_newData() {
	uint8_t irq_pin = HAL_GPIO_ReadPin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin);
	if(!irq_pin && wireless_irq_prev) {
		wireless_irq_prev = irq_pin;
		return 1;
	}
	else {
		wireless_irq_prev = irq_pin;
		return 0;
	}
}

void Wireless_newPacketHandler() {
	if(isNrfInitialized) { //wait till module is fully initialized before calling roboCallback, otherwise everything fucks up
		LastPackageTime = HAL_GetTick();
		//uprintf("\n\nnew wireless message (interrupt fired)\n");

		int8_t error_code = roboCallback(localRobotID);
		if(error_code) {
//			uprintf("RoboCallback failed with error: %i\n", error_code);
		}
		clearInterrupts(); //should not be needed
	}
}

void Wireless_Init(uint8_t address, uint8_t RADIO_CHANNEL) {
	localRobotID = address;
	preparedAckData.roboID = localRobotID;
	while(initRobo(&hspi2, RADIO_CHANNEL, localRobotID) != 0) {
		uprintf("Error while initializing nRF wireless module. Check connections.\n");
	}
	if(nrfPrintStatus(readReg(STATUS)) == 0x0e) {
		uprintf("nRF module initialized with ID: %i\n", preparedAckData.roboID);
		isNrfInitialized = 1;
		return;
	}
	else {
		while(nrfPrintStatus(readReg(STATUS)) != 0x0e) {
			uprintf("nRF is fucking around, blame Ulf....trying to reinitialize\r\n");
			Wireless_Deinit();
			while(initRobo(&hspi2, RADIO_CHANNEL, localRobotID) != 0) {
				uprintf("Error while initializing nRF wireless module. Check connections.\n");
			}
		}
	}
}


void Wireless_Deinit(){
	flushTX();
	flushRX();
	clearInterrupts();
}
