/*
 * wireless.c
 *
 *  Created on: Sep 29, 2018
 *      Author: pepijn
 */
#include "wireless.h"
#include "../PuttyInterface/PuttyInterface.h"
#include "../userIO/userIO.h"

SPI_HandleTypeDef *spiHandle = &hspi2;
roboData receivedRoboData;
bool data_ready = false;
Wireless_state  Wstate = Wconfig;

// force allign the buffer to make it start on a 32 bits address (otherwise mem fault)
uint8_t sndbuffer[32]  __attribute__((aligned(4))) = {0x00};
uint8_t  rcvbuffer[32] __attribute__((aligned(4))) = {0x00};


uint32_t Wireless_readStatus() {
	uint8_t status_buf[4] = {0};
	uint8_t cmd = READ_STATUS;
	//request status register
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiHandle,&cmd,1,100);
	HAL_SPI_Receive(spiHandle,status_buf,4,100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	uint32_t status = *((uint32_t*)status_buf); //cast array to one number
	return status;
}

void Wireless_writeStatus(uint32_t status) {
	uint8_t cmd = WRITE_STATUS;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiHandle, &cmd, 1, 100);
	HAL_SPI_Transmit(spiHandle, (uint8_t*)&status, 4, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void readData(uint8_t * data) {
	// cmd and data always have to be 32 bytes
	uint8_t cmd[32] = {READ_DATA, 0x00};

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiHandle, cmd, 2, 100);
	HAL_SPI_Receive(spiHandle, data, 32, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void writeData(uint8_t * data, size_t len) {
	uint8_t cmd[] = {WRITE_DATA, 0x00};
	// data always has to be 32 bytes
	uint8_t data_buf[32] __attribute__((aligned(4))) = {0};
	memcpy(data_buf,data,len);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiHandle, cmd, 2, 100);
	HAL_SPI_Transmit(spiHandle, data_buf, 32, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void Wireless_Init(int robot_id) {
	Wireless_writeStatus(SET_ROB_ID);
	int rob_id = ReadAddress();
	writeData((uint8_t*)&rob_id,4);
	HAL_Delay(1);
	Wireless_writeStatus(SET_SSID);
	writeData((uint8_t*)SSID, strlen(SSID));
	HAL_Delay(1);
	Wireless_writeStatus(SET_PSWD);
	writeData((uint8_t*)PSWD, strlen(PSWD));
	HAL_Delay(1);
	Wireless_writeStatus(SET_HOST_IP);
	writeData((uint8_t*)HOST_IP, strlen(HOST_IP));
}

void Wireless_Send(roboAckData* data) {
	roboAckDataToPacket(data, sndbuffer);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	uint8_t cmd[2] = {WRITE_DATA, 0x00};
	Wireless_writeStatus(SET_FB_DATA);
	HAL_SPI_Transmit(spiHandle, cmd, 2,100);
	HAL_SPI_Transmit(spiHandle, sndbuffer,32,100);
	//Wstate = Wtransmitting;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	Wstate = Wready;
}

bool Wireless_newData() {
	return HAL_GPIO_ReadPin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin) == GPIO_PIN_RESET;
}

void Wireless_newPacketHandler() {
	Wstate = Wreceiving;
	while(HAL_SPI_GetState(spiHandle) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	uint8_t cmd[2] = {READ_DATA, 0x00};
	HAL_SPI_Transmit(spiHandle,cmd,2,100);
	HAL_SPI_Receive_IT(spiHandle, rcvbuffer, 32);
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	packetToRoboData(rcvbuffer, &receivedRoboData);
	Wstate = Wreceived;
	data_ready = true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	Wstate = Wready;
}

void Wireless_INT_Handler(){
	uint32_t status = Wireless_readStatus();
	switch(status){
	case WAIT_FOR_SETTINGS:
		Wireless_Init(ReadAddress());
		Wstate = Wconfig;
		break;
	case SET_ROB_ID:
		Wireless_writeStatus(SET_ROB_ID);
		int rob_id = ReadAddress();
		writeData((uint8_t*)&rob_id,4);
		//Wireless_writeStatus(ReadAddress());
		break;
	case SET_SSID:
		Wireless_writeStatus(SET_SSID);
		writeData((uint8_t*)SSID, strlen(SSID));
		break;
	case SET_PSWD:
		Wireless_writeStatus(SET_PSWD);
		writeData((uint8_t*)PSWD, strlen(PSWD));
		break;
	case SET_HOST_IP:
		Wireless_writeStatus(SET_HOST_IP);
		writeData((uint8_t*)HOST_IP, strlen(HOST_IP));
		break;
	case NEW_DATA_READY:
		if(Wstate == Wready){
			Wireless_newPacketHandler();
		}
		break;
	case WIFI_CONNECTED:
		Wstate = Wconnected;
		break;
	case MQTT_CONNECTED:
		Wstate = Wready;
		break;
	case LOST_CONNECTION:
		Wstate = Wdisconnected;
		break;
	default:
		break;
	}
}

void Wireless_Update(roboAckData* ack){
	switch(Wstate){
	case Wconfig:
	case Wconnected:
	case Wdisconnected:
	case Wready:
		//if(Wireless_newData()&&!data_ready)	Wireless_INT_Handler();
		break;
	case Wreceived:
		Wireless_Send(ack);
		break;
	case Wreceiving:
	case Wtransmitting:
	default:
		break;
	}
}
