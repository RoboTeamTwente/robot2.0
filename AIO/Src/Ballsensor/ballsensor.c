/*
 * ballsensor.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Gebruiker
 */
#include "ballsensor.h"


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

uint8_t ballsensorInitialized = 0;

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if(zForceState == zForce_WaitForDR) {
		 zForceState = zForce_DecodeMessage;
		 uprintf("ballsensor decoding mesg\n");
	}
	else if(zForceState == zForce_ReadMessage) {
		parseMessage();
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	zForceState = zForce_WaitForDR;
}

void I2CTx(uint8_t tosend[], uint8_t length) {
    while(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(&hi2c1, ballsensor_i2caddr, tosend, length))){// in case of error; put the device in reset
  	  HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
        uprintf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
        zForceState = zForce_RST;
    }
}

void I2CRx() {
	while(HAL_OK != (error = HAL_I2C_Master_Receive_IT(&hi2c1, ballsensor_i2caddr, data, next_message_length))){
		uprintf("BALLSENSOR - i2c read failed with error [%d]!\n\rzForce stopped\n\r", error);
		HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
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

void printBallPosition() {
	uprintf("BALLSENSOR - x:\t %lu \t y:\t %lu \n\r", ballPosition.x, ballPosition.y);
}

void updatePosition(uint8_t data[]) {
  uint16_t x;
  x = data[12] << 8;
  x |= data[13];
  uint16_t y;
  y = data[14] << 8;
  y |= data[15];
  //uprintf("BALLSENSOR - x:\t %d \t y:\t %d \n\r", x,y);
	ballPosition.x = x;
	ballPosition.y = y;
}


void ballHandler(uint16_t x, uint16_t y) {
	//uprintf("ballHandler\n\r");
	if(kickWhenBall.enable) {
		kick_Shoot(kickWhenBall.power,KICK);
		noBall();
	}
	else if(chipWhenBall.enable) {
		kick_Shoot(chipWhenBall.power,CHIP);
		noBall();
	}
}

void parseMessage() {
	uprintf("parsemessage\n\r");

	if(!memcmp( data, bootcomplete_response, sizeof(bootcomplete_response)-1)) {
	  uprintf("BALLSENSOR - BootComplete response received, enabling device\n\r");
	  zForceState = zForce_EnableDevice;
	}
	else if(!memcmp(data, enable_response, sizeof(enable_response))) {
		zForceState = zForce_setFreq;
	}
	else if(!memcmp(data,measurement_rx, sizeof(measurement_rx))) { //ball detected
		//uprintf("ball detected\n\r");
		updatePosition(data);
		ballHandler(ballPosition.x,ballPosition.y);
		ballPosition.lastSeen = HAL_GetTick();
		zForceState = zForce_WaitForDR;
	}
	else if(!memcmp(data,set_freq_response, sizeof(set_freq_response))) {
//		uprintf("Set frequency:\r\n");
		//printRawData(data);
		ballsensorInitialized = 1;
		uprintf("BALLSENSOR - initialized successfully\n\r");
		zForceState = zForce_WaitForDR;
	}
	else { //ignore any other data
	  printRawData(data);
	  noBall();
	  zForceState = zForce_WaitForDR;
	  //uprintf("going to waitfordr\n\r");
	}
}

void ballsensorInit()
{
	//HAL_I2C_Init(&hi2c1);
	resetKickChipData();
	noBall();

	ballsensorReset();

	next_message_length = 2;
	HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 1);
	int currentTime = HAL_GetTick();
	zForceState = zForce_WaitForDR;
	while(  !HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)  &&  (HAL_GetTick()-currentTime < 100)  );
	uprintf("BALLSENSOR - data ready!\n");
	I2CRx();
}

void ballsensorReset() {
	ballsensorInitialized = 0;
	noBall();
	HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 0);
	//uprintf("going to waitfordr\n\r");
	zForceState = zForce_WaitForDR;
}

uint8_t ballsensorMeasurementLoop(uint8_t kick_enable, uint8_t chip_enable, uint8_t power)
{
	kickWhenBall.enable = kick_enable;
	chipWhenBall.enable = chip_enable;
	kickWhenBall.power = chipWhenBall.power = power;

	//uprintf("HAL_I2C_GetState = [%02x]\n\r", HAL_I2C_GetState(&hi2c1));
	if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		  	  return getBallPos();
	}

	//uprintf("zForceState = [%d]\n\r", zForceState);
	switch(zForceState){
	case zForce_RST:// device to be kept in reset
		uprintf("zForce_RST\n\r");
		ballsensorReset();
		break;
	case zForce_EnableDevice:
		//uprintf("zForce_EnableDevice\n\r");
		I2CTx(enable_command, sizeof(enable_command));
		break;
	case zForce_setFreq:
		//uprintf("zForce_setFreq\n\r");
		I2CTx(set_freq_command, sizeof(set_freq_command));
		break;
	case zForce_WaitForDR:// when DR(Data Ready) is high, message length needs to be read
		//uprintf("zForce_WaitForDR\n\r");
		next_message_length = 2;
		HAL_GPIO_WritePin(bs_nRST_GPIO_Port, bs_nRST_Pin, 1);
		if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)){
			//uprintf("data ready\n\r");
			I2CRx();
		}
		break;
	case zForce_DecodeMessage:// message is received and needs to be decoded
		uprintf("zForce_DecodeMessage\n\r");
		next_message_length = data[1];
		zForceState = zForce_ReadMessage;
		//uprintf("going to readmess state\n\r");
		break;
	case zForce_ReadMessage:// when message length is known it should be received
		uprintf("zForce_ReadMessage\n\r");
		if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port,bs_EXTI_Pin)){
			I2CRx();
		}
		break;
	}

	// if the ball hasn't been detected in a while, clear position data
	if(HAL_GetTick() - ballPosition.lastSeen > NOBALL_TIMEOUT) {
		noBall();
	}

		//PuttyInterface_Update(&puttystruct);
	//uprintf("ball: %i\n",getBallPos());
	//uprintf("ball: %i\n",getBallPos());
	return getBallPos();
}

void noBall() {
	ballPosition.x = ballPosition.y = NOBALL;
}

uint32_t getBallPos() {
	if(ballPosition.x != NOBALL) {
		return ballPosition.x/10;
	}
	return NOBALL;
}

void resetKickChipData() {
	kickWhenBall.enable = chipWhenBall.enable = 0;
	kickWhenBall.power = chipWhenBall.power = 0;
}
