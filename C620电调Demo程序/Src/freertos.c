/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "tim.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

#define CAN_CONTROL	//const current control 
//#define PWM_CONTROL	//const speed control
int set_v,set_spd[4];
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
//	PID_struct_init(&pid_omg, POSITION_PID, 20000, 20000,
//									1.5f,	0.1f,	0.0f	);  //angular rate closeloop.
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
	
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1000);	//ppm must be 1000 at first time.
	HAL_Delay(100);
  /* Infinite loop */
  for(;;)
  {
		
		#if defined CAN_CONTROL
		  
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			set_moto_current(&hcan1, pid_spd[0].pos_out, 
									pid_spd[1].pos_out,
									pid_spd[2].pos_out,
									pid_spd[3].pos_out);
		  
		#elif defined PWM_CONTROL
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000+set_spd[0]);//spd range[0,999]
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1000+set_spd[1]);
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1000+set_spd[2]);
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1000+set_spd[3]);
			//TIM5->CCR1 = set_spd[0];
		#endif
	
	static int key_sta,key_cnt;
	switch(key_sta)
	{
		case 0:	//no key
			if( 0 == HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) )
			{
				key_sta = 1;
			}
			break;
		case 1: //key down wait release.
			if( 0 == HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) )
			{
				key_sta = 2;
				key_cnt++;
			}
			else
			{
				key_sta = 0;
			}
			break;
		case 2: 
			if( 0 != HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) )
			{
				key_sta = 0;
			}
			break;
	}
	if(key_cnt>10)
		key_cnt = 0;
#if defined CAN_CONTROL
	set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = key_cnt*
	500;
#elif defined PWM_CONTROL
    set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = key_cnt*50;
#endif    
	
	osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
