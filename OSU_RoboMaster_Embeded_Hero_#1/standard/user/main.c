/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
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
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"


#include "gpio.h"

void BSP_init(void);
User_GPIO_X user_gpio;
User_PWM_X user_pwm;

int main(void)
{
    BSP_init();
    delay_ms(100);
    startTask();
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    led_configuration();
    //stm32 板载温度传感器初始化
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 随机数发生器初始化
    RNG_init();
#endif
    //24输出控制口 初始化
    power_ctrl_configuration();

		
///////////////////////////////////////////////////////////////////////////////
		//初始化配置GPIO
		GPIO_ID_E GPIO_ID_LIST[17]={I1,I2,J1,J2,K1,K2,L1,L2,M1,M2,N1,N2,O1,O2,P1,P2,Q2};//输入想要初始化配置的端口
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
				User_GPIO_Init(&user_gpio);//初始化配置GPIO
			}
		}
///////////////////////////////////////////////////////////////////////////////
		
///////////////////////////////////////////////////////////////////////////////
		//初始化配置PWM
		PWM_ID_E PWM_ID_LIST[16]={A,B,C,D,E,F,G,H,S,T,U,V,W,X,Y,Z};//输入想要初始化配置的端口
		int k;
		for (k=0;k<16;k++)
		{	
			if(PWM_ID_LIST[k]==NULL)//若端口未指定
			{
				;//则什么也不做，跳过
			}
			else
			{
				user_pwm.PWM_ID=PWM_ID_LIST[k];//设置PWM_ID为指定端口
				User_PWM_Init(&user_pwm);//初始化配置PWM
			}
		}
////////////////////////////////////////////////////////////////////////////////

		//17mm摩擦轮电机PWM初始化
    fric_PWM_configuration();
    //蜂鸣器初始化
    buzzer_init(30000, 90);
    //激光IO初始化
    laser_configuration();
    //定时器6 初始化
    TIM6_Init(60000, 90);
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN1-正常模式CAN_Mode_Normal，回环模式CAN_Mode_LoopBack
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//CAN2-正常模式CAN_Mode_Normal，回环模式CAN_Mode_LoopBack

    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //遥控器初始化
    remote_control_init();
    //flash读取函数，把校准值放回对应参数
    cali_param_init();
}
