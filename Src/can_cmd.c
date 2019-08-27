/*
 * can_cmd.c
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: User
 */

#include "can_cmd.h"
#include "can_tx_stack.h"

extern uint8_t current_group;
extern tx_stack can1_tx_stack;

void send_scan_cmd() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->unused_bits = 0;
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0x00;
	p_id->group_addr = current_group;
	p_id->cmd = SCAN_GROUP;
	p_id->param = 0;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 0;
	// добавить состояние входов выходов
	add_tx_can_packet(&can1_tx_stack,&packet);
}

void manage_relay(uint8_t point_num, uint8_t relay_num, uint8_t state) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->unused_bits = 0;
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0;
	p_id->group_addr = current_group;
	p_id->cmd = SET_OUTS;
	p_id->param = point_num;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 2;
	packet.data[0] = relay_num;
	packet.data[1] = state;
	// добавить состояние входов выходов
	add_tx_can_packet(&can1_tx_stack,&packet);
}
