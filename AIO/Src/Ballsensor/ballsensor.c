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
	zForce_EnableDevice,
	zForce_setFreq
}zForceState = zForce_RST;

uint8_t enable_command[] = 	{0xEE,0x0B,	0xEE,0x09,0x40,0x02,0x02,0x00,0x65,0x03,0x81,0x01,0x00};
uint8_t enable_response[] = {			0xEF,0x09,0x40,0x02,0x02,0x00,0x65,0x03,0x81,0x01,0x00};
uint8_t bootcomplete_response[] = {0xF0, 0x11, 0x40, 0x02, 0x00, 0x00, 0x63, 0x0B, 0x80, 0x01, 0x00, 0x81, 0x02, 0x03, 0x00, 0x82, 0x02, 0x00, 0x00};

//from datasheet, not used atm
uint8_t config_command[] = {0xEE, 0x1E, 0xEE, 0x1A, 0x40, 0x02, 0x02, 0x00, 0x73, 0x14, 0xA2, 0x12, 0x80, 0x02, 0x00, 0xB5, 0x81, 0x01, 0x43, 0x82, 0x02, 0x06, 0x98, 0x83, 0x02, 0x04, 0x34, 0x85, 0x01, 0xFF};

//set scanning frequency to 900Hz (0x384) & idle frequency to 125 Hz (0x7D)
uint8_t set_freq_command[] = {0xEE,0x0F,0xEE,0x0D,0x40,0x02,0x00,0x00,0x68,0x07,0x80,0x02,0x03,0x84,0x82,0x01,0x7D};
uint8_t set_freq_response[] = {0xEF,0x0D,0x40,0x02,0x00,0x00,0x68,0x07,0x80,0x02};

uint8_t measurement_rx[] = {0xf0, 0x11, 0x40, 0x02, 0x02};

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
	zForceState = zForce_WaitForDR;
}

void I2CTx(uint8_t tosend[]) {
	//On "sizeof(tosend)": the compiler gives a warning that this will return the size of "uint8_t *".
	//So that means, it will probably just always return "1", because, in general, a pointer for uint8_t is 1 Byte large.
	//This agrees with what I have seen in C code so far. If you need the length of an array that you pass to a function,
	//then you need to give this length explicitly as a separate argument.
	//That means: define uint8_t arrayLength as another argument and pass it sizeof(originalArray) in the function call
	//in the scope where the size of the array is known (where the array was declared).
	//(I just checked and saw that the function isn't called anywhere, yet.)
	//However, if your solution actually works, then correct me. Best regards ~~~ Ulf
    while(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(&hi2c1, ballsensor_i2caddr, tosend, sizeof(tosend)))){// in case of error; put the device in reset
  	  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
        uprintf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
        zForceState = zForce_RST;
    }
}

void I2CRx() {
	while(HAL_OK != (error = HAL_I2C_Master_Receive_IT(&hi2c1, ballsensor_i2caddr, data, next_message_length))){
		HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
		uprintf("BALLSENSOR - i2c read failed with error [%d]!\n\rzForce stopped\n\r", error);

		zForceState = zForce_RST;
	}
}

void printRawData(uint8_t data[]) {
    uprintf("data = [");
    for(uint i = 0; i < next_message_length; i++){
      uprintf("%02x ", data[i]);
    }
    uprintf("]\n\r");
}

void printPosition(uint8_t data[]) {
  uint16_t x;
  x = data[12] << 8;
  x |= data[13];
  uint16_t y;
  y = data[14] << 8;
  y |= data[15];
  uint16_t ball_position = x;
  uprintf("BALLSENSOR - x:\t %d \t y:\t %d \n\r", ball_position,y);
  ballHandler(x,y);
}


void ballHandler(uint16_t x, uint16_t y) {
	kick_Kick(60);
}

void parseMessage() {


	if(!memcmp( data, bootcomplete_response, sizeof(bootcomplete_response))) {
	  //uprintf("BootComplete response received, enabling device\n\r");
	  zForceState = zForce_EnableDevice;
	}
	else if(!memcmp(data, enable_response, sizeof(enable_response))) {
		uprintf("Enable response received, going to waitfordr\n\r");
		zForceState = zForce_setFreq;
	}
	else if(!memcmp(data,measurement_rx, sizeof(measurement_rx))) {
		printPosition(data);
		zForceState = zForce_WaitForDR;
	}
	else if(!memcmp(data,set_freq_response, sizeof(set_freq_response))) {
		uprintf("Set frequency:\r\n");
		printRawData(data);
		zForceState = zForce_WaitForDR;
	}
	else {
		printRawData(data);
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
				  I2CRx();
			  }
			  break;
		  case zForce_DecodeMessage:// message is received and needs to be decoded
		  			  next_message_length = data[1];
		  			  zForceState = zForce_ReadMessage;
		  			  //uprintf("going to readmess state\n\r");
		  			  break;
		  case zForce_ReadMessage:// when message length is known it should be received
			  if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)){
				  I2CRx();
			  }
			  break;
		  case zForce_EnableDevice:
			  	  I2CTx(enable_command);
			  break;
		  case zForce_setFreq:
		          uprintf("Setting frequency\n\r");
		          I2CTx(set_freq_command);
		      break;
			  break;
		  }

		PuttyInterface_Update(&puttystruct);
}

