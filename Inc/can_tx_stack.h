/*
 * can_tx_stack.h
 *
 *  Created on: 30 èþë. 2019 ã.
 *      Author: User
 */

#ifndef CAN_TX_STACK_H_
#define CAN_TX_STACK_H_

#include <stdint.h>

#define CAN_TX_DATA_SIZE		8
#define CAN_TX_STACK_LENGTH		64

#define EMPTY_PACKET	0
#define BUSY_PACKET		1
#define READY_PACKET	2

#define LOW_PACKET_PRIORITY		1
#define HIGH_PACKET_PRIORITY	2

#define		UNKNOWN_TYPE	0
#define		PC_TO_ALL		1
#define		PC_TO_GROUP		2
#define		PC_TO_POINT		3
#define		POINT_TO_ALL	4
#define		POINT_TO_PC		5

#define OPUS_PACKET_MAX_LENGTH	64

typedef struct
{
 uint32_t param: 8;
 uint32_t cmd: 4;
 uint32_t group_addr: 7;
 uint32_t point_addr: 7;
 uint32_t type: 3;
 uint32_t unused_bits: 3;
} id_field;

typedef struct {
	uint8_t data[CAN_TX_DATA_SIZE];
	uint32_t id;
	uint8_t length;
	uint8_t state;
	uint8_t priority;
} tx_stack_data;

typedef struct {
	tx_stack_data packet[CAN_TX_STACK_LENGTH];
	uint16_t read_position;
	uint16_t write_position;
} tx_stack;

void init_can_tx_stack(tx_stack *stack);
void add_tx_can_packet(tx_stack *stack,tx_stack_data *packet);
void divide_to_packets_and_send_to_can(uint8_t dest_group, uint8_t dest_point, uint8_t len, uint8_t *ptr);
uint8_t get_tx_can_packet(tx_stack *stack, tx_stack_data *packet);
void check_can_rx(uint8_t can_num);
void can_write_from_stack();

#endif /* CAN_TX_STACK_H_ */
