/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      *一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角*
  *             工程车现用与GPIO及PWM的配置与调用
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "User_Task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"

#include "Detect_Task.h"
#include "INS_Task.h"


//自定义
#include "gimbal_task.h"
#include "filter.h"
#include "CAN_Receive.h"
#include "chassis_task.h"
#include "gpio.h"

//软件复位Trigger
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "core_cmFunc.h"


#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//姿态角 单位度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};


//软件复位Trigger
extern void SoftReset(void)
{
	__set_FAULTMASK(1);//关闭所有中断，以防复位被打断
	NVIC_SystemReset();//复位
}





void UserTask(void *pvParameters)
{

		const volatile fp32 *angle;
		//获取姿态角指针
		angle = get_INS_angle_point();
	
    while (1)
    {

        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        if (!user_is_error())
        {
            led_green_on();
        }
				
				//开启GPIO
				GPIO_ID_E GPIO_ID_LIST[17]={I1,I2,J1,J2,K1,K2,L1,L2,M1,M2,N1,N2,O1,O2,P1,P2,Q2};//输入想要开启的端口
				int i;
				for (i=0;i<17;i++)
				{	
					if(GPIO_ID_LIST[i]==NULL)//若端口未指定
					{
						;//则什么也不做，跳过
					}
					else
					{
					user_gpio.GPIO_ID=GPIO_ID_LIST[i];//设置GPIO_ID为指定端口
					Set_User_GPIO(&user_gpio,ENABLE);//ENABLE开启GPIO端口
					}
				}
								
				vTaskDelay(1);//每隔1ms循环一次
//        led_green_off();
//        vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
