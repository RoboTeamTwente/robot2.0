/*
 * roboNRF24.c
 *
 *  Created on: Mar 16, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Specific commands for using the nRF24 wireless module
 *   with our robot.
 */

#include "roboNRF24.h"
#include "../PuttyInterface/PuttyInterface.h" //should be removed after debugging

uint8_t roboid = 0xFF;
uint8_t dataArray[32];
uint8_t misalignOffset = 0; //sometimes we need to hack our way trough the received bytes. Sometimes we receive 3 Bytes of bullshit before the actual data.

uint32_t lastTX = 0;

/*
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi) {
	uprintf("rxcallback\n\n");
	if(state == readData_1) {
			state = readData_2;
		}
	if(state == readData_0) {
		state = readData_2;
	}
}

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi) {
	uprintf("txcallback\n\n");
	lastTX = HAL_GetTick();
	if(state == readData_0) {
		state = readData_1;
	}
}
*/

uint8_t counter = 0;


int8_t initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t roboID){
	roboid = roboID;
	nrf24nssHigh(); //moved from main, ulf hates me

	//reset and flush buffer
	if(NRFinit(spiHandle, nrf24nssHigh, nrf24nssLow, nrf24ceHigh, nrf24ceLow, nrf24irqRead ) != 0) {
		return -1; //error
	}

	//set interrupts
	uint8_t config_reg = readReg(CONFIG);
	config_reg &= ~MASK_RX_DR;   //enable for RX_DR
	config_reg &= ~MASK_TX_DS;  //enable for TX_DS
	config_reg &= ~MASK_MAX_RT; //enable for MAX_RT
	writeReg(CONFIG, config_reg);

	setFreqChannel(freqChannel);
	//setLowSpeed();


	//enable pipe 0 and 1, disable all other pipes
	setDataPipes(ERX_P1);

	//with respect to the following discussion: https://devzone.nordicsemi.com/f/nordic-q-a/20235/nrf24l01p-data-pipe-forbidden-numbers
	//the address was chosen to not start or end with a lot of zeros or a lot of ones or
	//an alternating series which could be mistaken for the preamble.
	//Therefore, 0x99 was chosen, as it represents the bit series: "10011001"
	uint8_t addressLong[5] = {0x99, 0xB0 + roboID, 0x34, 0x56, 0x99};
	//uint8_t addressLong[5] = {0xA8, 0xA8, 0xE1, 0xF0, 0xC6};
	//set the RX address of data pipe x
	setRXaddress(addressLong, 1);

	//enable dynamic packet length, ack payload, dynamic acks
	writeReg(FEATURE, EN_DPL | EN_ACK_PAY | EN_DYN_ACK);

	//enable Auto Acknowledgment for Pipe 1
	writeReg(EN_AA, ENAA_P1);


	//enable dynamic packet length for data pipe(s)
	writeReg(DYNPD, DPL_P1);

	//writeReg(RX_PW_P1, 32); //set static payload length........ SHOULD NOT BE NEEDED

	//go to RX mode and start listening
	powerUpRX();

	//preparing a dummy-payload which will be sent
	//when the very first packet was received
	uint8_t dummyvalues[32];
	//cold food means: we just booted up
	dummyvalues[0] = 0xc0;
	dummyvalues[1] = 0x1d;
	dummyvalues[2] = 0xf0;
	dummyvalues[3] = 0x0d;

	writeACKpayload(dummyvalues, 4, 1);


	//go to RX mode and start listening
	powerUpRX();

	state = callback_0;

	return 0;
}
/*
 * TODO
 * this needs to be extended that it will read payloads in a loop
 * as long as there is data in the FIFO.
 * Read the FIFO_STATUS and check RX_EMPTY.
 *
 */
//negative values on erros.
//0 on success

void checkSPIWirelessState() {
	if(state == readData_1) {
		if(recv_started && SPIready()) {
			state = readData_2;
			uprintf("checking state - A\n");
			//uprintf("rx started\n");
			readData_IT(dataArray, ROBOPKTLEN+misalignOffset+1);
		}
		if(!recv_started && SPIready()) {
			uprintf("checking state - B\n");
			//uprintf("rx started\n");
			readData_IT(dataArray, ROBOPKTLEN+misalignOffset+1);
		}
	}
	else if(state == readData_2) {
		//uprintf("checking state\n");
			readData_IT(dataArray, ROBOPKTLEN+misalignOffset+1);
	}
	else if(state == readData_3) {
		uprintf("checking state - readdata 3\n");
		if(SPIready() && ack_sent) {
			uprintf("checking state - readdata done\n");
			state = readData_done;
		}
		else {
			uprintf("checking state - robocallback\n");
			if(0 != roboCallback(4)) {
				uprintf("robocallback error\n");
				ack_sent = 1;
				state = callback_0;
			}
		}
	}
	else if(state == readData_done) {
		uprintf("checking state - callback 0\n");
		state = callback_0;
		clearInterrupts();
	}
}

int8_t roboCallback(uint8_t localRobotID){

	uint8_t verbose = 1;


	if(state == callback_0) {
		ack_sent = 0;
		//clear RX interrupt
		writeReg(STATUS, RX_DR);
		nrf24ceLow();
		//actually reading the payload
		//uprintf("callback0\n\n");
		state = readData_0;
	}
	if((state == readData_0) || (state == readData_1) || (state == readData_2)) {

		readData_IT(dataArray, ROBOPKTLEN+misalignOffset+1); //+3 for misalignment +1 for cheksum byte.
		//state = readData_done;
	}
	if(state == readData_3) {
		//uprintf("readdata done\n\n");
		ack_sent = 0;

			//calculate the checksum for what I received
			uint8_t calculated_checksum = 0;

			//start at 3 for the misaligment
			for(uint8_t i=misalignOffset; i<(ROBOPKTLEN+misalignOffset); i++) {
				calculated_checksum ^= dataArray[i];
			}

			//building a packet from the current roboAckData struct
			uint8_t txPacket[32];
			uint8_t received_checksum = dataArray[ROBOPKTLEN+misalignOffset];

			printRoboData(&receivedRoboData,dataArray+misalignOffset,0);
			printRoboAckData(&preparedAckData,txPacket,SHORTACKPKTLEN,0);

			//compare the calculated checksum with the received checksum
			if(calculated_checksum != received_checksum) {
				//checksums don't match.
				uprintf("Checksums don't match\n");
				flushRX();
				return -4;
			}

			uint8_t receivedRobotID = (dataArray+misalignOffset)[0]>>3; //see packet format

			//uprintf("la1\n");

			if(receivedRobotID != localRobotID) {
				//if(verbose)
					uprintf("Received RobotID was wrong. Local ID: %i (0x%02x); Rx'd: %i (0x%02x)\n", localRobotID, localRobotID, receivedRobotID, receivedRobotID);
				flushRX();
				return -3; //packet wasn't for me (or, more likely: we did not receive any packet and read bullshit from the buffer)
			}
			//uprintf("la2\n");
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			//putting the new data from the packet on the struct
			packetToRoboData(dataArray+misalignOffset, &receivedRoboData);
			//uprintf("la3\n");
			//if(verbose) uprintf("Clearing RX_DR interrupt.\n");
			nrf24ceHigh();

			flushRX();

			//uprintf("la4\n");

			uint8_t ackDataLength;
			if(receivedRoboData.debug_info)
				ackDataLength = FULLACKPKTLEN; //adding xsense data
			else
				ackDataLength = SHORTACKPKTLEN;

			//fillAckData(ackDataLength);
			roboAckDataToPacket(&preparedAckData, txPacket);
			//uprintf("la5\n");

			//robotDataToPacket(&receivedRoboData, txPacket); //sending back the packet we just received

			if(writeACKpayload(txPacket, ackDataLength, 1) != 0) { //just for testing sending a robot packet to the basestation.
				//if(verbose)
				uprintf("Error writing ACK payload. TX FIFO full?\n");
				return -2; //error while writing ACK payload to buffer
			}
			ack_sent = 1;

			//uprintf("la6\n");
	}
	return 0; //success
}

void fillDummyAckData(uint8_t ackDataLength) {

	preparedAckData.roboID = 5;
	preparedAckData.wheelLeftFront = 1;
	preparedAckData.wheelLeftFront = 1;
	preparedAckData.wheelRightFront = 1;
	preparedAckData.wheelLeftBack = 1;
	preparedAckData.wheelRightBack = 1;
	preparedAckData.genevaDriveState = 1;
	preparedAckData.batteryState = 1;
	preparedAckData.xPosRobot = counter;
	preparedAckData.yPosRobot = counter;
	preparedAckData.rho = counter;
	preparedAckData.theta = counter;
	preparedAckData.orientation = counter;
	preparedAckData.angularVelocity = counter;
	preparedAckData.ballSensor = counter;

	if(ackDataLength == FULLACKPKTLEN) {
		counter++;
		if(counter>10000)
			counter = 1;
		//extra fields (add 12 Bytes)
		preparedAckData.xAcceleration = counter;
		preparedAckData.yAcceleration = counter;
		preparedAckData.angularRate = counter;
	}

}

void fillXSensData(float data[3]) {


	union float_bytes {
	       float val;
	       uint32_t bytes;
	    } x,y,w;

	x.val = data[0];
	y.val = data[1];
	w.val = data[2];

	preparedAckData.xAcceleration = x.bytes;
	preparedAckData.yAcceleration = y.bytes;
	preparedAckData.angularRate = w.bytes;

}


/*
 * Pin setters and reader
 */


//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
	//NSS / CSN : chip select
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

//put the nss pin corresponding to the SPI used low
void nrf24nssLow(){
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh(){
	//CE: chip enable
	//In this board revision, this pin is directly connected to Vdd ("high").
}

//put the ce pin corresponding to the SPI used low
void nrf24ceLow(){
	//HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
	//In this board revision, this pin is directly connected to Vdd ("high").
}


//read the interrupt pin
uint8_t nrf24irqRead(){
	//return !HAL_GPIO_ReadPin(SPI2_IRQ_GPIO_Port, SPI2_IRQ_Pin);
	//has been changed to using Interrupts on that pin
	return 0;
}
