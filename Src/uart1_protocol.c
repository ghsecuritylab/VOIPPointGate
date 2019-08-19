/*
 * uart1_protocol.c
 *
 *  Created on: 24 мая 2019 г.
 *      Author: User
 */


#include "uart1_protocol.h"
#include "uart.h"
#include "main.h"

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	send_data_to_uart1((uint8_t*)"ok\r\n",4);
	if(rx_cnt==3) {
		switch(rx_ptr[0]) {
		case 0x41:
			break;
		}
	}
}
