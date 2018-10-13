/*
 * wireless.c
 *
 *  Created on: Sep 29, 2018
 *      Author: pepijn
 */
#include "wireless.h"

SPI_HandleTypeDef *spiHandle = &hspi2;
volatile roboData receivedRoboData;


uint32_t readStatus() {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	//SPI.transfer(0x04);
	//uint32_t status = (SPI.transfer(0) | ((uint32_t)(SPI.transfer(0)) << 8) | ((uint32_t)(SPI.transfer(0)) << 16) | ((uint32_t)(SPI.transfer(0)) << 24));
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	return 0;
}

void writeStatus(uint32_t status) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	uint8_t data[] = {
			0x01,
			status & 0xFF,
			(status >> 8) & 0xFF,
			(status >> 16) & 0xFF,
			(status >> 24) & 0xFF
	};
	HAL_SPI_Transmit(spiHandle, data, 5, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void readData(uint8_t * data) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	uint8_t cmd[] = {0x03, 0x00};
	HAL_SPI_Transmit(spiHandle, cmd, 2, 100);
	HAL_SPI_Receive(spiHandle, data, 32, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void writeData(uint8_t * data, size_t len) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	uint8_t cmd[] = {0x02, 0x00};
	HAL_SPI_Transmit(spiHandle, cmd, 2, 100);
	HAL_SPI_Transmit(spiHandle, data, len, 100);
	// transmit remaining bytes
	uint8_t empty[32] = {0};
	HAL_SPI_Transmit(spiHandle, empty, 32-len, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void Wireless_Init(char* ssid, char* pass, char* ip, int robot_id) {
	writeStatus(robot_id);
	HAL_Delay(1);
	writeData((uint8_t*)ssid, strlen(ssid));
	HAL_Delay(1);
	writeData((uint8_t*)pass, strlen(pass));
	HAL_Delay(1);
	writeData((uint8_t*)ip, strlen(ip));
}

volatile uint8_t sndbuffer[2+32] = {0x02, 0x00};
void Wireless_Send(roboAckData* data) {
	roboAckDataToPacket(data, sndbuffer+2);
	if(HAL_SPI_GetState(spiHandle) == HAL_SPI_STATE_READY) {
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(spiHandle, sndbuffer, 2+32);
	}
}

int Wireless_newData() {
	return HAL_GPIO_ReadPin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin) == GPIO_PIN_SET;
}

volatile uint8_t cmdbuffer[2+32] = {0x03, 0x00};
volatile uint8_t rcvbuffer[2+32];
void Wireless_newPacketHandler() {
	if(HAL_SPI_GetState(spiHandle) == HAL_SPI_STATE_READY) {
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_IT(spiHandle, cmdbuffer, rcvbuffer, 2+32);
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	packetToRoboData(rcvbuffer+2, &receivedRoboData);
}
