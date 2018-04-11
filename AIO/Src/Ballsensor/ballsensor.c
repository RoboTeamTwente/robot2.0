/*
 * ballsensor.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Gebruiker
 */
#include "ballsensor.h"
#include "../kickchip/kickchip.h"

enum zForceStates{
	zForce_RST,
	zForce_WaitForDR,
	zForce_DecodeMessage,
	zForce_ReadMessage,
	zForce_EnableDevice
}zForceState = zForce_RST;

uint8_t enable_command[] = 	{0xEE,0x0B,	0xEE,0x09,0x40,0x02,0x02,0x00,0x65,0x03,0x81,0x01,0x00};
uint8_t enable_response[] = {			0xEF,0x09,0x40,0x02,0x02,0x00,0x65,0x03,0x81,0x01,0x00};
uint8_t bootcomplete_response[] = {0xF0, 0x11, 0x40, 0x02, 0x00, 0x00, 0x63, 0x0B, 0x80, 0x01, 0x00, 0x81, 0x02, 0x03, 0x00, 0x82, 0x02, 0x00, 0x00};

uint8_t next_message_length = 2;

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if(zForceState == zForce_WaitForDR) {
		 zForceState = zForce_DecodeMessage;
	}
	else if(zForceState == zForce_ReadMessage) {
		parseMessage();
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if(zForceState == zForce_EnableDevice) {
			zForceState = zForce_WaitForDR;
	}
}


void parseMessage() {
	uint16_t x;
	x = data[12] << 8;
	x |= data[13];
	uint16_t y;
	y = data[14] << 8;
	y |= data[15];
	//uprintf("BALLSENSOR - x:\t %d \t y:\t %d \n\r", x,y);
	kick_Kick();

	if(!memcmp( data, bootcomplete_response, sizeof(bootcomplete_response))) {
	  //uprintf("BootComplete response received, enabling device\n\r");
	  zForceState = zForce_EnableDevice;
	}
	else if(!memcmp(data, enable_response, sizeof(enable_response))) {
		//uprintf("Enable response received, going to waitfordr\n\r");
		zForceState = zForce_WaitForDR;
	}
	else {
	  zForceState = zForce_WaitForDR;
	  //uprintf("going to waitfordr\n\r");
	}
}

void ballsensorInit()
{
	  PuttyInterface_Init(&puttystruct);
	  uprintf("Initializing ball sensor\r\n");
	  HAL_I2C_Init(&hi2c1);
}

void ballsensorMeasurementLoop()
{
	  if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	  	{
		  	  return;
	  	}

	switch(zForceState){
		  case zForce_RST:// device to be kept in reset
			  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
			  uprintf("going to waitfordr\n\r");
			  zForceState = zForce_WaitForDR;
			  break;

		  case zForce_WaitForDR:// when DR(Data Ready) is high, message length needs to be read

			  next_message_length = 2;
			  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 1);
			  if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)){
				  //uprintf("data ready\n\r");
				  while(HAL_OK != (error = HAL_I2C_Master_Receive_IT(&hi2c1, ballsensor_i2caddr, data, next_message_length))){
					  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
					  uprintf("BALLSENSOR WaitforDR - i2c read failed with error [%d]!\n\rzForce stopped\n\r", error);

					  zForceState = zForce_RST;
				  }
			  }
			  break;
		  case zForce_DecodeMessage:// message is received and needs to be decoded
		  			  next_message_length = data[1];
		  			  zForceState = zForce_ReadMessage;
		  			  //uprintf("going to readmess state\n\r");
		  			  break;
		  case zForce_ReadMessage:// when message length is known it should be received
			  if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)){

				  while(HAL_OK != (error = HAL_I2C_Master_Receive_IT(&hi2c1, ballsensor_i2caddr, data, next_message_length))){// in case of error; put the device in reset
					  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
					  uprintf("BALLSENSOR ReadMessage - i2c read failed with error [%d]!\n\rzForce stopped\n\r", error);
				  }
			  }
			  break;
		  case zForce_EnableDevice:
			  while(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(&hi2c1, ballsensor_i2caddr, enable_command, 13))){// in case of error; put the device in reset
			  		HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
			  		uprintf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
			  		zForceState = zForce_RST;
			  }

			  break;
			  break;
		  }

		PuttyInterface_Update(&puttystruct);
}

