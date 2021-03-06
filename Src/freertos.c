/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include "button_led.h"
#include "udp_server.h"
#include "uart.h"
#include "modbus.h"
#include "can_cmd.h"
#include "dyn_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define		DI_BREAK_LIMIT		200
#define		DI_OPEN_LIMIT		800
#define		DI_CLOSED_LIMIT		2000

#define		START_STATE			0
#define		CHECK_DI1			1
#define		CHECK_RELAY2		2
#define		CHECK_DI2			3
#define		CHECK_AUDIO			4
#define		CHECK_DI3			5
#define		CHECK_DI2_2			6
#define		CHECK_RELAY2_2		7
#define		CHECK_DI1_2			8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t		gate_state = START_STATE;
uint16_t	gate_tmr = 0;
uint16_t	sec_cnt = 0;
uint16_t	point_cnt = 2;
static uint16_t	i = 0;
static uint8_t	relay2_test = 0;
static uint8_t	audio_test = 0;

static uint16_t err_dec = 0;
static uint16_t err_point = 0;
static uint16_t err_num = 0;

extern uint16_t adc_data[3];

extern unsigned short inpReg[InputRegistersLimit];
extern unsigned char discrInp[DiscreteInputsLimit];
extern uint16_t group_tmr[GROUP_CNT];
extern struct group_data groups[GROUP_CNT];

uint8_t modbus_di_array[DiscreteInputsLimit/8];

uint16_t group_bits=0;

extern uint16_t can_tmr;

extern uint8_t p_cnt;
extern uint8_t current_group;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId canTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// �������� ������� ��� � �����
static void bytes_to_bits(uint8_t *inp, uint8_t *out, uint16_t cnt) {
	uint16_t i = 0;
	uint8_t bit_num = 0;
	uint16_t byte_num = 0;
	uint8_t byte_value = 0;
	for(i=0;i<cnt;i++) {
		if(inp[i]) byte_value |= 1<<bit_num;
		bit_num++;
		if(bit_num==8) {
			bit_num = 0;
			out[byte_num++] = byte_value;
			byte_value = 0;
		}
	}
}
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCanTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCanTask, osPriorityNormal, 0, 1024);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
                 
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  static uint8_t gate_update_tmr=0;
  struct group_data group;

  uint16_t j=0;

  led_init();
  udp_server_init();

  osDelay(500);
  get_points_state();
  osDelay(500);
  get_points_state();

  for(;;)
  {
    osDelay(100);
    for(j=0;j<GROUP_CNT;j++){
    	if(groups[j].num) {
    		if(group_tmr[j]<50) group_tmr[j]++;else {
				if(group_tmr[j]==50) {	// ������ �� ������ ����� �� �����������, ����� ������
					group_tmr[j]++;
					group.num = j+1;
					group.point_cnt=0;
					group.version=0;
					group.bits=(uint16_t)1<<11;
					add_group_data(j,&group);
				}
			}
    	}
    }

    // ������������� �������� ��������� �����
    gate_update_tmr++;
    if(gate_update_tmr>=10) {
    	gate_update_tmr=0;
    	send_get_state();
    }

    inpReg[1] = gate_state;


    // ������������ ������ ��� ������� ������
    group.bits=0;
    for(i=0;i<3;i++) {
    	if(adc_data[i]<DI_BREAK_LIMIT) {	// �����
    		discrInp[3*i+0] = 0;
    		discrInp[3*i+1] = 1;
    		discrInp[3*i+2] = 0;
    		group.bits |= ((uint16_t)1<<(1+i*3));
    	}else if(adc_data[i]<DI_OPEN_LIMIT) {	// ����
    		discrInp[3*i+0] = 0;
    		discrInp[3*i+1] = 0;
    		discrInp[3*i+2] = 0;
    	}else if(adc_data[i]<DI_CLOSED_LIMIT) {	// ���
    		discrInp[3*i+0] = 1;
    		discrInp[3*i+1] = 0;
    		discrInp[3*i+2] = 0;
    		group.bits |= ((uint16_t)1<<(0+i*3));
    	}else {	//	��
    		discrInp[3*i+0] = 0;
    		discrInp[3*i+1] = 0;
    		discrInp[3*i+2] = 1;
    		group.bits |= ((uint16_t)1<<(2+i*3));
    	}
    }
    if(HAL_GPIO_ReadPin(RELAY1_GPIO_Port,RELAY1_Pin)==GPIO_PIN_SET) {
    	discrInp[9]=1;group.bits |= ((uint16_t)1<<9);
    }else discrInp[9]=0;
    if(HAL_GPIO_ReadPin(RELAY2_GPIO_Port,RELAY2_Pin)==GPIO_PIN_SET) {
    	discrInp[10]=1;group.bits |= ((uint16_t)1<<10);
    }else discrInp[10]=0;

    group.num = current_group;
	group.point_cnt=p_cnt;
	group.version=1;
	add_group_data(current_group-1,&group);
	group_tmr[current_group-1]=0;
	group_bits = group.bits;

    // manage relay 2
    relay2_test = 1;
    if(inpReg[0]>=point_cnt) {
    	for(i=0;i<point_cnt;++i) {
    		if(discrInp[16+i*10+1]==0) {relay2_test = 0;break;}
    	}
    }else relay2_test = 0;
    if(relay2_test) {HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,GPIO_PIN_SET);}
    else HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,GPIO_PIN_RESET);

    gate_tmr++;
    if(gate_tmr>=10) {
    	gate_tmr=0;
    	sec_cnt++;
    }

    // ������������ ������ DAC ��� ������������ ����� ������������ �����
    err_num = 0;
    if(inpReg[0]>=point_cnt) {
		for(i=0;i<point_cnt;++i) {
			if(discrInp[16+i*10+1]==0) {err_num = i;break;}
		}
	}else {
		if(inpReg[0]) err_num = inpReg[0];else err_num=99;
	}
    if(err_num) {
    	err_dec = err_num/10;
    	err_point = err_num%10;
    }else {
    	err_dec = 0;
    	err_point = 0;
    }
    can_tmr++;
    if(can_tmr>=30) {
    	err_dec = 9;
    	err_point = 9;
    	inpReg[0] = 0;
    	p_cnt=0;
    }

    TIM1->CCR2=(65535/9)*err_dec;
    TIM1->CCR3=(65535/9)*err_point;

    // �������� ���������� �������� ����� � �����
    switch(gate_state) {
		case START_STATE:
			if(gate_tmr==0) {	// ��������� ���� �� ���� �����������������
				manage_all_relays(1,0);
				manage_all_relays(2,0);
				gate_state = CHECK_DI1;
			}
			HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,GPIO_PIN_RESET);
			break;
		case CHECK_DI1:
			if(discrInp[0]) gate_state = CHECK_RELAY2;
			else gate_state = START_STATE;
			break;
		case CHECK_RELAY2:
			if(HAL_GPIO_ReadPin(RELAY2_GPIO_Port,RELAY2_Pin)==GPIO_PIN_SET) gate_state = CHECK_DI2;
			else gate_state = START_STATE;
			break;
		case CHECK_DI2:
			if(discrInp[3]) {
				HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,GPIO_PIN_SET);
				send_scan_cmd_from_gate();
				gate_tmr = 0;sec_cnt = 0;
				manage_all_relays(2,1);
				gate_state = CHECK_AUDIO;
			}else gate_state = START_STATE;
			break;
		case CHECK_AUDIO:
			if(gate_tmr==0) send_scan_cmd_from_gate();
			if(sec_cnt>=2) {
				audio_test = 1;
				for(i=0;i<point_cnt;i++) {
					if(discrInp[16+i*10]==0) {audio_test = 0; break;}
				}
				if(audio_test) gate_state = CHECK_DI3;
				if(sec_cnt>=4) gate_state = START_STATE;
			}
			break;
		case CHECK_DI3:
			if(discrInp[6]) {
				gate_state = CHECK_DI2_2;
			}else gate_state = CHECK_DI1;
			break;
		case CHECK_DI2_2:
			if(discrInp[3]) {
				if(discrInp[0]==0) gate_state = START_STATE;
				//if(gate_tmr==0) send_scan_cmd_from_gate();
				if(gate_tmr==0) gate_state = CHECK_DI2;
			}else gate_state = CHECK_RELAY2_2;
			break;
		case CHECK_RELAY2_2:
			if(HAL_GPIO_ReadPin(RELAY2_GPIO_Port,RELAY2_Pin)==GPIO_PIN_SET) gate_state = CHECK_DI1_2;
			else gate_state = START_STATE;
			break;
		case CHECK_DI1_2:
			if(discrInp[0]) {
				HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,GPIO_PIN_RESET);
				manage_all_relays(2,0);
				manage_all_relays(1,1);
				gate_state = CHECK_DI3;
			}
			else gate_state = START_STATE;
			break;
    }
    toggle_first_led(RED);
    toggle_second_led(RED);
    bytes_to_bits(discrInp,modbus_di_array,DiscreteInputsLimit);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
