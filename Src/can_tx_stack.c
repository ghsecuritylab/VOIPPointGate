#include "can_tx_stack.h"
#include <cmsis_gcc.h>
#include "main.h"
#include "frame_stack.h"
#include "button_led.h"
#include "can_cmd.h"
#include "modbus.h"
#include "dyn_data.h"
#include "udp_server.h"

#define MODE_PC_TO_ALL		0
#define MODE_PC_TO_POINT	1
#define MODE_PC_TO_GROUP	2

extern unsigned short holdReg[HoldingRegistersLimit];
extern unsigned char discrInp[DiscreteInputsLimit];
extern unsigned short inpReg[InputRegistersLimit];
extern unsigned char coils[CoilsLimit];
extern uint16_t group_tmr[GROUP_CNT];

tx_stack can1_tx_stack;
tx_stack can2_tx_stack;

tx_stack can_rx1_stack;
tx_stack can_rx2_stack;


uint32_t	can_caught_id = UNKNOWN_TYPE;
uint8_t current_group = 1;
uint8_t rx_group = 0;
uint8_t rx_point = 0;
static uint8_t	can_priority_frame[OPUS_PACKET_MAX_LENGTH];
static uint8_t	can_frame[OPUS_PACKET_MAX_LENGTH];
uint8_t can_pckt_length = 0;
uint16_t packet_tmr = 0;

uint8_t p_cnt=0;
uint8_t prev_p_cnt=0;

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox1=0;
static uint32_t              TxMailbox2=0;
static uint8_t               TxData[8];
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];

uint8_t wr_stack_flag = 0;
uint16_t can_tmr = 0;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern uint8_t cur_mode;
extern uint8_t destination_group;
extern uint8_t destination_point;

static uint8_t inline is_cmd_through_up(uint8_t cmd) {
	if(cmd == AUDIO_PACKET || cmd == POINT_STATE || cmd==GATE_STATE || cmd==BOOT) return 1;
	return 0;
}

static uint8_t inline is_cmd_through_down(uint8_t cmd, uint8_t param) {
	if(cmd == AUDIO_PACKET  || ((cmd==SCAN_GROUP) && param) || cmd==BOOT || cmd==POINT_CONFIG) return 1;
	return 0;
}

// TX

static uint16_t atomicWrPosIncrement(uint16_t * ptr)
{
	uint16_t oldValue, newValue;
	do
	{
		oldValue = __LDREXH(ptr);
		newValue = oldValue + 1;
		if(newValue>=CAN_TX_STACK_LENGTH) newValue = 0;
	}while(__STREXH(newValue, ptr));
	return oldValue;
}

void init_can_tx_stack(tx_stack *stack) {
	uint8_t i = 0;
	stack->read_position = 0;
	stack->write_position = 0;
	for(i=0;i<CAN_TX_STACK_LENGTH;i++) {
		stack->packet[i].length = 0;
		stack->packet[i].state = EMPTY_PACKET;
	}
}

void add_tx_can_packet(tx_stack *stack,tx_stack_data *packet) {
	uint8_t i = 0;
	uint16_t wr_pos = atomicWrPosIncrement(&(stack->write_position));
	stack->packet[wr_pos].state = BUSY_PACKET;
	for(i=0;i<packet->length;++i) stack->packet[wr_pos].data[i] = packet->data[i];
	stack->packet[wr_pos].id = packet->id;
	stack->packet[wr_pos].priority = packet->priority;
	stack->packet[wr_pos].length = packet->length;
	stack->packet[wr_pos].state = READY_PACKET;
}

uint8_t get_tx_can_packet(tx_stack *stack, tx_stack_data *packet) {
	uint8_t i = 0;
	if(stack->packet[stack->read_position].state==READY_PACKET) {
		packet->id = stack->packet[stack->read_position].id;
		packet->length = stack->packet[stack->read_position].length;
		packet->priority = stack->packet[stack->read_position].priority;
		for(i=0;i<packet->length;++i) packet->data[i] = stack->packet[stack->read_position].data[i];
		stack->packet[stack->read_position].state = EMPTY_PACKET;
		stack->read_position++;
		if(stack->read_position>=CAN_TX_STACK_LENGTH) stack->read_position = 0;
		return 1;
	}
	return 0;
}

void divide_to_packets_and_send_to_can(uint8_t dest_group, uint8_t dest_point, uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);

	wr_stack_flag = 1;

	while(i) {
		pckt_cnt++;
		if(i<=8) {i=0;break;}
		i-=8;
	}
	i=1;
	while(cur_pckt<=pckt_cnt) {
		p_id->point_addr = dest_point&0x7F;
		p_id->group_addr = dest_group&0x7F;
		if(dest_point==0xFF) p_id->type = PC_TO_ALL;
		else if(dest_point==0x00) {p_id->type = PC_TO_GROUP;p_id->point_addr = 0;}
		else p_id->type = PC_TO_POINT;

		p_id->cmd = 1;
		p_id->param = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4);
		packet.priority = LOW_PACKET_PRIORITY;
		if(cur_pckt==pckt_cnt) { // last packet
			packet.length = len;
			for(i=0;i<len;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			//toggle_first_led(GREEN);
			cur_pckt++;
			len=0;
		}else {
			packet.length = 8;
			for(i=0;i<8;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			cur_pckt++;
			len-=8;
		}
	}
	wr_stack_flag = 0;
}

void can_write_from_stack() {
	//static uint8_t intern_tmr = 0;
	//static uint8_t start_flag = 0;
	tx_stack_data packet;
	uint8_t i = 0;
	if(wr_stack_flag) return;
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=0) {
			if(get_tx_can_packet(&can1_tx_stack,&packet)) {
				TxHeader.StdId = 0;
				TxHeader.ExtId = packet.id;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.TransmitGlobalTime = DISABLE;
				TxHeader.DLC = packet.length;
				for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
				//toggle_first_led(GREEN);
			}else break;
		}

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=0) {
		if(get_tx_can_packet(&can2_tx_stack,&packet)) {
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox2);
		}else break;
	}
}

// RX

uint8_t static check_id_priority(uint32_t packet_id) {
	id_field *input_id = (id_field*)(&packet_id);
	id_field *cur_id = (id_field*)(&can_caught_id);
	if(cur_mode == MODE_PC_TO_ALL) {
		if(input_id->type==POINT_TO_PC) {
			if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
				*cur_id = *input_id;
				return 1;
			}
			// ����� ����������� ��������
			if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
				*cur_id = *input_id;
				return 1;
			}
			// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
			if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
				*cur_id = *input_id;
				return 1;

			}
			return 1;
		}
		if(input_id->type==POINT_TO_ALL) { // ����� ���
			if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
				*cur_id = *input_id;
				return 1;
			}
			// ����� ����������� ��������
			if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
				*cur_id = *input_id;
				return 1;
			}
			// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
			if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
				*cur_id = *input_id;
				return 1;
			}
			return 0;
		}
	}else if(cur_mode == MODE_PC_TO_GROUP) {
		if(input_id->group_addr == destination_group) {
			if(input_id->type==POINT_TO_PC) {
				if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
					*cur_id = *input_id;
					return 1;
				}
				// ����� ����������� ��������
				if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
					*cur_id = *input_id;
					return 1;
				}
				// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
				if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
					*cur_id = *input_id;
					return 1;

				}
				return 1;
			}
			if(input_id->type==POINT_TO_ALL) { // ����� ���
				if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
					*cur_id = *input_id;
					return 1;
				}
				// ����� ����������� ��������
				if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
					*cur_id = *input_id;
					return 1;
				}
				// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
				if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
					*cur_id = *input_id;
					return 1;
				}
				return 0;
			}
		}
	}else {
		if(input_id->group_addr == destination_group && input_id->point_addr == destination_point) {
			if(input_id->type==POINT_TO_PC) {
				if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
					*cur_id = *input_id;
					return 1;
				}
				// ����� ����������� ��������
				if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
					*cur_id = *input_id;
					return 1;
				}
				// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
				if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
					*cur_id = *input_id;
					return 1;

				}
				return 1;
			}
			if(input_id->type==POINT_TO_ALL) { // ����� ���
				if(cur_id->type==UNKNOWN_TYPE) {	// ������ �� ���������
					*cur_id = *input_id;
					return 1;
				}
				// ����� ����������� ��������
				if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
					*cur_id = *input_id;
					return 1;
				}
				// �������� ����� �� �� ������ ������, ����� ������ �� ������ ������, �����������
				if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
					*cur_id = *input_id;
					return 1;
				}
				return 0;
			}
		}
	}


	return 0;
}

void check_can_rx(uint8_t can_num) {
	CAN_HandleTypeDef *hcan;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t cur_num = 0;
	uint8_t cnt = 0;
	id_field *p_id;
	tx_stack_data packet;
	struct point_data point;
	struct group_data group;
	if(can_num==1) hcan = &hcan1; else hcan = &hcan2;
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

			p_id = (id_field*)(&(RxHeader.ExtId));
			if(p_id->group_addr==0) return;
			//if(p_id->cmd==BOOT_ERASE_PAGE && p_id->type==0) toggle_first_led(GREEN);
			if(p_id->cmd==AUDIO_PACKET) { // ����������
				if(check_id_priority(RxHeader.ExtId))
				{
					packet_tmr = 0;
					cur_num = p_id->param & 0x0F;
					cnt = (p_id->param & 0xFF)>> 4;
					if(cur_num) {
						if(cur_num==cnt) {
						  if(p_id->type==POINT_TO_ALL || p_id->type==POINT_TO_PC) { // ����� ���
							  j = (cur_num-1)*8;
							  for(i=0;i<RxHeader.DLC;i++) {
								  if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i];
							  }
							  can_pckt_length = (cnt-1)*8+RxHeader.DLC;
							  if(can_pckt_length>OPUS_PACKET_MAX_LENGTH) can_pckt_length = OPUS_PACKET_MAX_LENGTH;
							  for(i=0;i<can_pckt_length;i++) {
								  can_frame[i]=can_priority_frame[i];
							  }
							  add_can_frame(&can_frame[0],can_pckt_length);
							  rx_group = p_id->group_addr;
							  rx_point = p_id->point_addr;
						  }
						}else {
						  j = (cur_num-1)*8;
						  for(i=0;i<8;i++) { if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i]; }
						}
					}
				}
			}else if(p_id->cmd==POINT_STATE) {
				if(p_id->point_addr>0 && p_id->point_addr<=100) {
					if(p_id->group_addr==current_group) {
						discrInp[16+(p_id->point_addr-1)*10] = RxData[0]&0x01;		// ����������� ���������/��������
						discrInp[16+(p_id->point_addr-1)*10+1] = RxData[1]&0x01;	// di1
						discrInp[16+(p_id->point_addr-1)*10+2] = RxData[1]&0x02;	// �����
						discrInp[16+(p_id->point_addr-1)*10+3] = RxData[1]&0x04;	// ��
						discrInp[16+(p_id->point_addr-1)*10+4] = RxData[1]&0x08;	// di2
						discrInp[16+(p_id->point_addr-1)*10+5] = RxData[1]&0x10;	// �����
						discrInp[16+(p_id->point_addr-1)*10+6] = RxData[1]&0x20;	// ��
						discrInp[16+(p_id->point_addr-1)*10+7] = RxData[1]&0x40;		// do1
						discrInp[16+(p_id->point_addr-1)*10+8] = RxData[1]&0x80;		// do2
						discrInp[16+(p_id->point_addr-1)*10+9] = RxData[0]&0x02;		// �������� ������ ���������
						inpReg[16+(p_id->point_addr-1)] = (((uint16_t)RxData[2])<<8) | RxData[3];
					}
					point.battery = RxData[2];
					point.power = RxData[3];
					point.gr_num = p_id->group_addr-1;
					point.point_num = p_id->point_addr-1;
					point.version = RxData[4];
					point.bits = (((uint16_t)RxData[0])<<8) | RxData[1];
					point.gain = RxData[5];
					add_point_data(&point);
				}
			}else if(p_id->cmd==LAST_POINT) {
				p_cnt = inpReg[0] = p_id->point_addr;
				if(prev_p_cnt!=p_cnt) send_get_state();
				prev_p_cnt = p_cnt;
				if(p_id->group_addr==current_group) can_tmr = 0;
			}else if(p_id->cmd==GATE_STATE) {
				group.num = p_id->group_addr;
				group.bits=(((uint16_t)RxData[1])<<8) | RxData[2];;
				group.point_cnt=RxData[0];
				group.version=1;
				group_tmr[p_id->group_addr-1]=0;
				add_group_data(p_id->group_addr-1,&group);
			}else if(p_id->cmd==BOOT) {
				if(p_id->type==BOOT_WRITE_ACK) {
					send_boot_ack(RxData[0]);
				}else if(p_id->type==BOOT_ERASE_PAGE_ACK) {
					send_erase_ack(p_id->param);
					//toggle_first_led(GREEN);
				}
			}


			packet.id = RxHeader.ExtId;
			packet.priority = LOW_PACKET_PRIORITY;
			packet.length = RxHeader.DLC;
			for(i=0;i<packet.length;++i) packet.data[i] = RxData[i];
			if(can_num==1 && is_cmd_through_up(p_id->cmd)) {
				add_tx_can_packet(&can2_tx_stack,&packet);
				//if((p_id->group_addr==current_group) && (p_id->cmd==POINT_STATE)) toggle_first_led(GREEN);
			}
			else if(can_num==2 && is_cmd_through_down(p_id->cmd,p_id->param)){add_tx_can_packet(&can1_tx_stack,&packet);}
		}
	}
}
