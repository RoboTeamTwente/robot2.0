/*
 * debugNRF24.c
 *
 *  Created on: May 10, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *		A couple of functions to debug the nRF wireless module.
 *		In this section you are encouraged to print debug messages.
 *		The original library, however, should not be using any print statements in the final version.
 */

#include "myNRF24.h"
#include "../PuttyInterface/PuttyInterface.h"


void printNRFregisters() {
	uprintf("Reading registers.\n");
	uprintf("CONFIG: 0x%02x\n", readReg(CONFIG));
	uprintf("EN_AA: 0x%02x\n", readReg(EN_AA));
	uprintf("EN_RXADDR: 0x%02x\n", readReg(EN_RXADDR));
	uprintf("SETUP_AW: 0x%02x\n", readReg(SETUP_AW));
	uprintf("SETUP_RETR: 0x%02x\n", readReg(SETUP_RETR));
	uprintf("RF_CH: 0x%02x\n", readReg(RF_CH));
	uprintf("RF_SETUP: 0x%02x\n", readReg(RF_SETUP));

	uint8_t status_reg = readReg(STATUS);
	uprintf("STATUS: 0x%02x ( ", status_reg);
	if(status_reg & RX_DR) uprintf("RX_DR ");
	if(status_reg & TX_DS) uprintf("TX_DS ");
	if(status_reg & MAX_RT) uprintf("MAX_RT ");
	uint8_t pipeNo = (status_reg >> 1)&7;
	if(pipeNo >= 0 && pipeNo <= 0b101) uprintf("PIPE:%i ", pipeNo);
	if(pipeNo == 0b110) uprintf("RX_FIFO:not_used ");
	if(pipeNo == 0b111) uprintf("RX_FIFO:empty ");
	if(status_reg & STATUS_TX_FULL) uprintf("TX_FULL ");
	uprintf(")\n");

	uprintf("OBSERVE_TX: 0x%02x\n", readReg(OBSERVE_TX));
	uprintf("RPD: 0x%02x\n", readReg(RPD));
	uint8_t buffer[5];
	readRegMulti(RX_ADDR_P0, buffer, 5);
	uprintf("RX_ADDR_P0: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	readRegMulti(RX_ADDR_P1, buffer, 5);
	uprintf("RX_ADDR_P1: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	readRegMulti(RX_ADDR_P2, buffer, 5);
	uprintf("RX_ADDR_P2: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	readRegMulti(RX_ADDR_P3, buffer, 5);
	uprintf("RX_ADDR_P3: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	readRegMulti(RX_ADDR_P4, buffer, 5);
	uprintf("RX_ADDR_P4: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	readRegMulti(RX_ADDR_P5, buffer, 5);
	uprintf("RX_ADDR_P5: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);

	readRegMulti(TX_ADDR, buffer, 5);
	uprintf("TX_ADDR: 0x%x%x%x%x%x\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	uprintf("RX_PW_P0: 0x%02x\n", readReg(RX_PW_P0));
	uprintf("RX_PW_P1: 0x%02x\n", readReg(RX_PW_P1));
	uprintf("RX_PW_P2: 0x%02x\n", readReg(RX_PW_P2));
	uprintf("RX_PW_P3: 0x%02x\n", readReg(RX_PW_P3));
	uprintf("RX_PW_P4: 0x%02x\n", readReg(RX_PW_P4));
	uprintf("RX_PW_P5: 0x%02x\n", readReg(RX_PW_P5));
	uint8_t fifo_status = readReg(FIFO_STATUS);
	uprintf("FIFO_STATUS: 0x%02x  ( ", fifo_status);
	if(fifo_status & TX_REUSE) uprintf("TX_REUSE ");
	if(fifo_status & FIFO_STATUS_TX_FULL) uprintf("TX_FULL ");
	if(fifo_status & TX_EMPTY) uprintf("TX_EMPTY ");
	if(fifo_status & RX_FULL) uprintf("RX_FULL ");
	if(fifo_status & RX_EMPTY) uprintf("RX_EMPTY ");
	uprintf(" )\n");

	uprintf("DYNPD: 0x%02x\n", readReg(DYNPD));
	uprintf("FEATURE: 0x%02x\n", readReg(FEATURE));
}

uint8_t nrfPrintStatus(uint8_t status_reg) {
	//uint8_t status_reg = readReg(STATUS);
	uprintf("STATUS: 0x%02x ( ", status_reg);
	if(status_reg & RX_DR) uprintf("RX_DR ");
	if(status_reg & TX_DS) uprintf("TX_DS ");
	if(status_reg & MAX_RT) uprintf("MAX_RT ");
	uint8_t pipeNo = (status_reg >> 1)&7;
	if(pipeNo >= 0 && pipeNo <= 0b101) uprintf("PIPE:%i ", pipeNo);
	if(pipeNo == 0b110) uprintf("RX_FIFO:not_used ");
	if(pipeNo == 0b111) uprintf("RX_FIFO:empty ");
	if(status_reg & STATUS_TX_FULL) uprintf("TX_FULL ");
	uprintf(")\n");
	return status_reg;
}
