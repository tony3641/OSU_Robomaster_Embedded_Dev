/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      *一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角*
  *             现用与filter计算线程
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


#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//姿态角 单位度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};


////自定义内容
//定义结构体
Gimbal_Control_t gimbal_control;
tx2_aim_package_t tx2;

//底盘运动数据
chassis_move_t chassis_move;

//声明filter类型
Group_Delay_t group_delay_ecd_aim;
Group_Delay_t group_delay_gyro_aim;
Group_Delay_Chassis_t group_delay_chassis_speed;

Blackman_Filter_t blackman;

IIR_Filter_t butterworth_aim_yaw;
IIR_Filter_t butterworth_aim_pitch;
IIR_Filter_t butterworth_final_angle;

kalman_filter_t kalman;
kalman_filter_init_t kalman_initial;


//定义filter
double Group_Delay(Group_Delay_t *GD);
double Group_Delay_Chassis(Group_Delay_Chassis_t *GD);
double Blackman_Filter(Blackman_Filter_t *F);
double Butterworth_Filter(IIR_Filter_t *F);
float *kalman_filter_calc(kalman_filter_t *F, float x, float y, float vx, float vy);
//初始化Kalman
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);

//传出数据，用于其他文件
//extern 变量定义类型必须一致
extern fp32 delayed_yaw_relative_angle;
extern fp32 delayed_yaw_absolute_angle;
extern fp32 delayed_pitch_relative_angle;
extern fp32 delayed_wz_set;
extern fp32 *filtered_aim_data[4];
extern fp32 filtered_vertical_pixel;
extern fp32 *filtered_final_angle_set[4];

//传入数据
int32_t final_yaw_angle_set;
int32_t final_pitch_angle_set;


//声明flag
static int filter_aim_data_flag;
static int filter_final_angle_set_flag;



static void Filter_Running(Gimbal_Control_t *gimbal_data)
{

		
		filter_aim_data_flag=1;
		filter_final_angle_set_flag=0;
	
		
		//是否对视觉发来的自瞄数据进行滤波
		if(filter_aim_data_flag==1)
		{
			*filtered_aim_data=kalman_filter_calc(&kalman,
																						tx2.horizontal_pixel,
																						tx2.vertical_pixel,
																						gimbal_control.gimbal_yaw_motor.motor_gyro,
																						gimbal_control.gimbal_pitch_motor.motor_gyro);
			
//			filtered_horizontal_pixel=Butterworth_Filter(&butterworth_aim_yaw);//Blackman_Filter(&blackman);
		}
		else if(filter_aim_data_flag==0)
		{
//			*filtered_horizontal_pixel=tx2.horizontal_pixel;
		}
		
		
		//是否对最终角度进行滤波
		if(filter_final_angle_set_flag==1)
		{	
			*filtered_final_angle_set=kalman_filter_calc(&kalman,
																									 final_yaw_angle_set,
																									 final_pitch_angle_set,
																									 gimbal_control.gimbal_yaw_motor.motor_gyro,
																									 gimbal_control.gimbal_pitch_motor.motor_gyro);
		}
		else if(filter_final_angle_set_flag==0)
		{	static fp32 temp[4];
			temp[0]=final_yaw_angle_set;
			temp[1]=final_pitch_angle_set;
			temp[2]=0;
			temp[3]=0;
			*filtered_final_angle_set=temp;
		}		
		
		
		delayed_pitch_relative_angle=Group_Delay(&group_delay_ecd_aim);//经过x ms delay后的编码器的值
		delayed_yaw_absolute_angle=Group_Delay(&group_delay_gyro_aim);//经过x ms delay后的陀螺仪的值
		delayed_wz_set=Group_Delay_Chassis(&group_delay_chassis_speed);//Group_Delay(&group_delay_chassis_speed);
	}


void UserTask(void *pvParameters)
{

		const volatile fp32 *angle;
		const fp32 dt=0;//运行频率？
		//获取姿态角指针
		angle = get_INS_angle_point();
	
		//初始化Kalman Filter矩阵
		//Covariance of the Process Noise
		kalman_initial.Q_data[0]	= 1;			kalman_initial.Q_data[1]	=	0;			kalman_initial.Q_data[2]	=	0;			kalman_initial.Q_data[3]	=	0;
		kalman_initial.Q_data[4]	= 0;			kalman_initial.Q_data[5]	=	1;			kalman_initial.Q_data[6]	=	0;			kalman_initial.Q_data[7]	=	0;
		kalman_initial.Q_data[8]	= 0;			kalman_initial.Q_data[9]	=	0;			kalman_initial.Q_data[10]	=	1;			kalman_initial.Q_data[11]	=	0;
		kalman_initial.Q_data[12]	=	0;			kalman_initial.Q_data[13]	=	0;			kalman_initial.Q_data[14]	=	0;			kalman_initial.Q_data[15]	=	1;
		
		//Covariance of the Measurement Noise
		kalman_initial.R_data[0]	= 2000;		kalman_initial.R_data[1]	=	0;			kalman_initial.R_data[2]	=	0;			kalman_initial.R_data[3]	=	0;
		kalman_initial.R_data[4]	= 0;			kalman_initial.R_data[5]	=	2000;		kalman_initial.R_data[6]	=	0;			kalman_initial.R_data[7]	=	0;
		kalman_initial.R_data[8]	= 0;			kalman_initial.R_data[9]	=	0;			kalman_initial.R_data[10]	=	10000;	kalman_initial.R_data[11]	=	0;
		kalman_initial.R_data[12]	=	0;			kalman_initial.R_data[13]	=	0;			kalman_initial.R_data[14]	=	0;			kalman_initial.R_data[15]	=	10000;
		
		//System Term
		kalman_initial.A_data[0]	= 1;			kalman_initial.A_data[1]	=	0;			kalman_initial.A_data[2]	=	0;			kalman_initial.A_data[3]	=	0;
		kalman_initial.A_data[4]	= 0;			kalman_initial.A_data[5]	=	1;			kalman_initial.A_data[6]	=	0;			kalman_initial.A_data[7]	=	0;
		kalman_initial.A_data[8]	= 0;			kalman_initial.A_data[9]	=	0;			kalman_initial.A_data[10]	=	1;			kalman_initial.A_data[11]	=	0;
		kalman_initial.A_data[12]	=	0;			kalman_initial.A_data[13]	=	0;			kalman_initial.A_data[14]	=	0;			kalman_initial.A_data[15]	=	1;
		
		kalman_initial.AT_data[0]	= 0;			kalman_initial.AT_data[1]	=	0;			kalman_initial.AT_data[2]	=	0;			kalman_initial.AT_data[3]	=	0;
		kalman_initial.AT_data[4]	= 0;			kalman_initial.AT_data[5]	=	0;			kalman_initial.AT_data[6]	=	0;			kalman_initial.AT_data[7]	=	0;
		kalman_initial.AT_data[8]	= 0;			kalman_initial.AT_data[9]	=	0;			kalman_initial.AT_data[10]=	0;			kalman_initial.AT_data[11]=	0;
		kalman_initial.AT_data[12]=	0;			kalman_initial.AT_data[13]=	0;			kalman_initial.AT_data[14]=	0;			kalman_initial.AT_data[15]=	0;
	
	
		//Observation Model
		kalman_initial.H_data[0]	= 1;			kalman_initial.H_data[1]	=	0;			kalman_initial.H_data[2]	=	0;			kalman_initial.H_data[3]	=	0;
		kalman_initial.H_data[4]	= 0;			kalman_initial.H_data[5]	=	1;			kalman_initial.H_data[6]	=	0;			kalman_initial.H_data[7]	=	0;
		kalman_initial.H_data[8]	= 0;			kalman_initial.H_data[9]	=	0;			kalman_initial.H_data[10]	=	1;			kalman_initial.H_data[11]	=	0;
		kalman_initial.H_data[12]	=	0;			kalman_initial.H_data[13]	=	0;			kalman_initial.H_data[14]	=	0;			kalman_initial.H_data[15]	=	1;
		
		kalman_initial.HT_data[0]	= 0;			kalman_initial.HT_data[1]	=	0;			kalman_initial.HT_data[2]	=	0;			kalman_initial.HT_data[3]	=	0;
		kalman_initial.HT_data[4]	= 0;			kalman_initial.HT_data[5]	=	0;			kalman_initial.HT_data[6]	=	0;			kalman_initial.HT_data[7]	=	0;
		kalman_initial.HT_data[8]	= 0;			kalman_initial.HT_data[9]	=	0;			kalman_initial.HT_data[10]=	0;			kalman_initial.HT_data[11]=	0;
		kalman_initial.HT_data[12]=	0;			kalman_initial.HT_data[13]=	0;			kalman_initial.HT_data[14]=	0;			kalman_initial.HT_data[15]=	0;

		//Prior Estimate
		kalman_initial.xhat_data[0]=0;
		kalman_initial.xhat_data[1]=0;
		kalman_initial.xhat_data[2]=0;
		kalman_initial.xhat_data[3]=0;
		
		//Last State of Prior Estimate
		kalman_initial.xhatminus_data[0]=0;			
		kalman_initial.xhatminus_data[1]=0;
		kalman_initial.xhatminus_data[2]=0;			
		kalman_initial.xhatminus_data[3]=0;
		
		//Actual Measurement of x
		kalman_initial.z_data[0]=0;
		kalman_initial.z_data[1]=0;
		kalman_initial.z_data[2]=0;
		kalman_initial.z_data[3]=0;
	
		//Covariance of the Estimation-Error
		kalman_initial.Pminus_data[0]	= 0;	kalman_initial.Pminus_data[1]	=	0;	kalman_initial.Pminus_data[2]	=	0;		kalman_initial.Pminus_data[3]	=	0;
		kalman_initial.Pminus_data[4]	= 0;	kalman_initial.Pminus_data[5]	=	0;	kalman_initial.Pminus_data[6]	=	0;		kalman_initial.Pminus_data[7]	=	0;
		kalman_initial.Pminus_data[8]	= 0;	kalman_initial.Pminus_data[9]	=	0;	kalman_initial.Pminus_data[10]=	0;		kalman_initial.Pminus_data[11]=	0;
		kalman_initial.Pminus_data[12]=	0;	kalman_initial.Pminus_data[13]=	0;	kalman_initial.Pminus_data[14]=	0;		kalman_initial.Pminus_data[15]=	0;
		
		//Kalman Gain
		kalman_initial.K_data[0]	= 0;			kalman_initial.K_data[1]	=	0;			kalman_initial.K_data[2]	=	0;				kalman_initial.K_data[3]	=	0;
		kalman_initial.K_data[4]	= 0;			kalman_initial.K_data[5]	=	0;			kalman_initial.K_data[6]	=	0;				kalman_initial.K_data[7]	=	0;
		kalman_initial.K_data[8]	= 0;			kalman_initial.K_data[9]	=	0;			kalman_initial.K_data[10] =	0;				kalman_initial.K_data[11] =	0;
		kalman_initial.K_data[12] =	0;			kalman_initial.K_data[13] =	0;			kalman_initial.K_data[14] =	0;				kalman_initial.K_data[15] =	0;

		//Initial Covariance of the Estimation-Error
		kalman_initial.P_data[0]	= 0;			kalman_initial.P_data[1]	=	0;			kalman_initial.P_data[2]	=	0;				kalman_initial.P_data[3]	=	0;
		kalman_initial.P_data[4]	= 0;			kalman_initial.P_data[5]	=	0;			kalman_initial.P_data[6]	=	0;				kalman_initial.P_data[7]	=	0;
		kalman_initial.P_data[8]	= 0;			kalman_initial.P_data[9]	=	0;			kalman_initial.P_data[10] =	0;				kalman_initial.P_data[11] =	0;
		kalman_initial.P_data[12] =	0;			kalman_initial.P_data[13] =	0;			kalman_initial.P_data[14] =	0;				kalman_initial.P_data[15] =	0;
	
		//初始化Kalman Filter
		kalman_filter_init(&kalman,&kalman_initial);
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
				
				
				
				
				//filter的输入
				//group_delay_ecd_aim.group_delay_raw_value=gimbal_control.gimbal_yaw_motor.relative_angle;//group delay 输入为编码器相对ecd_offset的角度(-pi/2,pi/2)
				group_delay_gyro_aim.group_delay_raw_value=gimbal_control.gimbal_yaw_motor.absolute_angle;//group delay 输入为陀螺仪YAW轴的角度 
				group_delay_ecd_aim.group_delay_raw_value=gimbal_control.gimbal_pitch_motor.relative_angle;//group delay 输入为编码器PITCH轴相对于的ecd_offset的角度(-pi/2,pi/2)
				//group_delay_chassis_speed.group_delay_chassis_raw_value=chassis_move.wz_set;
//				butterworth_aim_yaw.raw_value=tx2.horizontal_pixel;
//				butterworth_final_angle.raw_value=final_angle_set;

				
				Filter_Running(&gimbal_control);//filter进行计算
				
				
				
				
				
				
				
				
				vTaskDelay(1);//每隔1ms循环一次
//        led_green_off();
//        vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
