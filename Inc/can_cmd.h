/*
 * can_cmd.h
 *
 *  Created on: 23 рту. 2019 у.
 *      Author: User
 */

#ifndef CAN_CMD_H_
#define CAN_CMD_H_

#include <stdint.h>

#define		AUDIO_PACKET	1
#define		FIND_NEXT_POINT	2
#define		SCAN_GROUP		3
#define		POINT_STATE		4
#define		SET_OUTS		5
#define		LAST_POINT		6

#define		UNKNOWN_TYPE	0
#define		PC_TO_ALL		1
#define		PC_TO_GROUP		2
#define		PC_TO_POINT		3
#define		POINT_TO_ALL	4
#define		POINT_TO_PC		5
#define		UNUSED_TYPE		6

void send_scan_cmd();
void manage_relay(uint8_t point_num, uint8_t relay_num, uint8_t state);

#endif /* CAN_CMD_H_ */
