/*
 * uart1_protocol.c
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"
#include "modbus.h"
#include "button_led.h"
#include "crc.h"

static buf tx;
static buf rx;

uint8_t net_addr = 0x01;

static uint8_t tx_buf[UART_BUF_SISE];

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if((rx_ptr[0]==net_addr || rx_ptr[0]==0x00) && (GetCRC16(rx_ptr,rx_cnt==0))) {
		rx.ptr = rx_ptr;
		rx.cnt = rx_cnt;
		modbusCmd* res = searchCmd(&rx);
		if(res->isCmd)
		{
			tx.ptr = tx_buf;
			tx.cnt = UART_BUF_SISE;
			sendAnswer(res,&tx);
		}
	}
	//send_data_to_uart1((uint8_t*)"ok\r\n",4);
}
