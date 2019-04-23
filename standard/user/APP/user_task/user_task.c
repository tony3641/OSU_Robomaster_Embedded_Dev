/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角
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



#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//姿态角 单位度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};


////自定义内容
//定义gimbal结构体，获取数据
Gimbal_Control_t gimbal_control;

//定义filter类型
FIR_Filter_t group_delay;
//FIR_Filter_t double_group_delay;

//定义filter
double Group_Delay_Filter(FIR_Filter_t *F);
//extern group delay后的编码器，用于其他文件
extern fp32 delayed_ecd;
//fp32 double_delayed_value;

static void Filter_running(Gimbal_Control_t *gimbal_data)
	{
		delayed_ecd=Group_Delay_Filter(&group_delay);//经过x ms delay后的编码器的值, 根据需求参考GroupDelayTable
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
				
				
				
				
				
				group_delay.fir_raw_value=gimbal_control.gimbal_yaw_motor.relative_angle;//group delay fir filter输入为编码器相对ecd_offset的角度(-pi/2,pi/2)

				//double_group_delay.fir_raw_value=delayed_value;
				//double_delayed_value=Group_Delay_Filter(&double_group_delay);

				
				
				
				Filter_running(&gimbal_control);//运行filter
				
				
				
				
				
				
				
				
				vTaskDelay(1);//每隔1ms循环一次
//        led_green_off();
//        vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
