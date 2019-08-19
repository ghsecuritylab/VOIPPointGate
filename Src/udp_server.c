/*
 * udp_server.c
 *
 *  Created on: 13 ���. 2019 �.
 *      Author: User
 */

#include "udp_server.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "crc.h"

#include "stm32f4xx_hal.h"
#include <string.h>
#include "button_led.h"
#include "frame_stack.h"
#include "can_tx_stack.h"

#define UDP_SERVER_PORT    12145


static char answer[1324];
static unsigned short reqID = 0;

unsigned short udp_tmr = 0;

uint8_t destination_group = 1;
uint8_t destination_point = 255;

extern uint8_t rx_group;
extern uint8_t rx_point;

extern uint16_t adc_data[3];


static void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void inline send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length);


void udp_server_init(void) {
	struct udp_pcb *upcb;
	err_t err;

	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb)
	{
	 /* Bind the upcb to the UDP_PORT port */
	 /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
	  err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

	  if(err == ERR_OK)
	  {

		/* Set a receive callback for the upcb */
		udp_recv(upcb, udp_server_receive_callback, NULL);
	  }
	  else
	  {
		udp_remove(upcb);
	  }
	}
}

void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	unsigned char *data;
	unsigned short crc;

	unsigned short length = 0;
	unsigned short offset = 0;
	unsigned short answer_offset = 0;
	unsigned char i=0;
	static unsigned char pckt_cnt = 0;
	static unsigned char pckt_length[20];

	data = (unsigned char*)(p->payload);
	crc = GetCRC16(data,p->len);
	if(crc==0)
	{
	  udp_tmr = 0;
	  reqID = (unsigned short)data[0]<<8;
	  reqID |= data[1];
	  switch(data[2]){
		  case 0xA0:
			  answer[0] = data[0];
			  answer[1] = data[1];
			  answer[2] = 0xA0;
			  answer[3] = 0x01;	// type of device identificator
			  answer[4] = 0xEF;
			  answer[5] = 0x35;
			  answer[6] = 0x7A;
			  crc = GetCRC16((unsigned char*)answer,7);
			  answer[7]=crc>>8;
			  answer[8]=crc&0xFF;
			  send_udp_data(upcb, addr, port,9);
			  break;
		  case 0xD0:
			  answer[0] = data[0];
			  answer[1] = data[1];
			  answer[2] = 0xD0;
			  answer[3] = adc_data[0]>>8;
			  answer[4] = adc_data[0] & 0xFF;
			  answer[5] = adc_data[1]>>8;
			  answer[6] = adc_data[1] & 0xFF;
			  answer[7] = adc_data[2]>>8;
			  answer[8] = adc_data[2] & 0xFF;
			  crc = GetCRC16((unsigned char*)answer,9);
			  answer[9]=crc>>8;
			  answer[10]=crc&0xFF;
			  send_udp_data(upcb, addr, port,11);
			  break;
		  case 0xEF:
			  SCB->AIRCR = 0x05FA0004;
			  break;
		  case 0x01:
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x01;		// cmd

			  destination_group = data[3];
			  destination_point = data[4];

			  answer[3] = rx_group;		// group from
			  answer[4] = rx_point;  	// point from

			  //device_id = data[3];
			  pckt_cnt = data[5];if(pckt_cnt>=20) pckt_cnt=0;
			  answer[5] = pckt_cnt;
			  for(i=0;i<pckt_cnt;i++) pckt_length[i] = data[6+i];
			  offset = 6 + pckt_cnt;
			  answer_offset = 6 + pckt_cnt;
			  toggle_first_led(GREEN);

			  for(i=0;i<pckt_cnt;i++) {
				  divide_to_packets_and_send_to_can(destination_group, destination_point, pckt_length[i], &data[offset]);
				  offset+=pckt_length[i];
				  length=0;//get_can_frame((unsigned char*)&answer[answer_offset]);
				  answer[6+i] = length;
				  if(length) { answer_offset+=length;}
			  }
			  toggle_second_led(GREEN);
			  //if(length) toggle_second_led(GREEN);
			  crc = GetCRC16((unsigned char*)answer,answer_offset);
			  answer[answer_offset++] = crc>>8;
			  answer[answer_offset++] = crc&0xFF;
			  send_udp_data(upcb, addr, port,answer_offset);
			  break;
		  case 0x02:
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x02;		// cmd

			  destination_group = data[3];
			  destination_point = data[4];

			  answer[3] = rx_group;		// group from
			  answer[4] = rx_point;  	// point from

			  //device_id = data[3];
			  pckt_cnt = data[5];if(pckt_cnt>=20) pckt_cnt=0;
			  answer[5] = pckt_cnt;
			  for(i=0;i<pckt_cnt;i++) pckt_length[i] = data[6+i];
			  offset = 6 + pckt_cnt;
			  answer_offset = 6 + pckt_cnt;
			  toggle_first_led(GREEN);

			  for(i=0;i<pckt_cnt;i++) {
				  offset+=pckt_length[i];
				  length=get_can_frame((unsigned char*)&answer[answer_offset]);
				  answer[6+i] = length;
				  if(length) { answer_offset+=length;}
			  }
			  toggle_second_led(GREEN);
			  //if(length) toggle_second_led(GREEN);
			  crc = GetCRC16((unsigned char*)answer,answer_offset);
			  answer[answer_offset++] = crc>>8;
			  answer[answer_offset++] = crc&0xFF;
			  send_udp_data(upcb, addr, port,answer_offset);
			  break;
	  }
	}

	/* Free the p buffer */
	pbuf_free(p);
}

void inline send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length) {
	struct pbuf *p_answer;
	udp_connect(upcb, addr, port);
	p_answer = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
	if (p_answer != NULL)
	{
	  pbuf_take(p_answer, answer, length);
	  udp_send(upcb, p_answer);
	  pbuf_free(p_answer);
	}
	udp_disconnect(upcb);
}
