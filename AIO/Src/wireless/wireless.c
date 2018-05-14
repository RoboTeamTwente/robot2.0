/*
 * wireless.c
 *
 *  Created on: May 14, 2018
 *      Author: Gebruiker
 */

#include "wireless.h"

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
	if(isNrfInitialized) {
			LastPackageTime = HAL_GetTick();
			uprintf("\n\nnew wireless message (interrupt fired)\n");

			int8_t error_code = roboCallback(localRobotID);
			if(error_code) {
				uprintf("RoboCallback failed with error: %i\n", error_code);
			}
			clearInterrupts(); //should not be needed



			//kicker
			if (receivedRoboData.kick_chip_forced) {
				uprintf("FORCE KICK\n\n");
				kickchip_command = 1;
			}
		}
}

void Wireless_Init() {
	localRobotID = ReadAddress();
	uprintf("Robot ID: %i\n", localRobotID);

	while(initRobo(&hspi2, RADIO_CHANNEL, localRobotID) != 0) {
		uprintf("Error while initializing nRF wireless module. Check connections.\n");
	}
	isNrfInitialized = 1;
	uprintf("nRF wireless module successfully initialized.\n");

	nrfPrintStatus();
}
