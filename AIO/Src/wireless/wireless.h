/*
 * wireless.h
 *
 *  Created on: Sep 29, 2018
 *      Author: pepijn
 */
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "spi.h"
#include "packing.h"


#ifndef WIRELESS_H_
#define WIRELESS_H_

#define SSID	"RoboTeam"
#define PSWD	"oranjebal"
#define HOST_IP	"192.168.0.101"


#define READ_STATUS 	0x04
#define WRITE_STATUS 	0x01
#define READ_DATA 		0x03
#define WRITE_DATA 		0x02

//status register value
#define WAIT_FOR_SETTINGS 		0xFFFF
#define NEW_DATA_READY 			0xAAAA
#define LOST_CONNECTION			0xCCCC
#define WIFI_CONNECTED			0x3333
#define MQTT_CONNECTED			0x8888
#define SET_ROB_ID    		    0xFF00
#define SET_SSID            	0xFF01
#define SET_PSWD        	    0xFF02
#define SET_HOST_IP 	        0xFF03
#define SET_FB_DATA				0xFF04


typedef enum{
	Wconfig,
	Wconnected,
	Wdisconnected,
	Wreceiving,
	Wreceived,
	Wtransmitting,
	Wready,
	Widle
}Wireless_state;
Wireless_state Wstate;

extern SPI_HandleTypeDef *spiHandle;
extern volatile roboData receivedRoboData;
extern bool data_ready;

void Wireless_INT_Handler();
bool Wireless_newData();
void Wireless_newPacketHandler();
void Wireless_Init(int robot_id);
void Wireless_Update(roboAckData* ack);
void Wireless_Send(roboAckData* data);
uint32_t Wireless_readStatus();
void Wireless_writeStatus(uint32_t status);


#endif /* WIRELESS_H_ */
