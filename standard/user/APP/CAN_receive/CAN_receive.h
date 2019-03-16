/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      can device transmit and recevice function，receive via CAN interrupt
  * @note       This is NOT a freeRTOS TASK
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              Complete
  *  V1.0.1     Feb-17-2019     Tony-OSU        Add tx2 can bus config
	*  V1.1.0     Feb-21-2019     Tony-OSU        Finish Custom CAN Bus, fully functional
	*  V1.2.0     Mar-01-2019     Tony-OSU        Package ID modified 
	*																							@note TX2 package is now 0x111 for higher priority
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  **************Modifid by Ohio State University Robomaster Team****************

  */

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN2
#define GIMBAL_CAN CAN1
#define TX2_CAN CAN2
#define PID_TUNING_CAN CAN2


/* Enumerate CAN send and receive ID */
/* 枚举声明CAN收发ID*/
typedef enum
{
	  CAN_AIM_DATA_ID = 0x300,//现在用这个
	
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
		
	
		GYRO_DATA_TX2_ID=0x212,//陀螺仪绝对角度数据ID
    CAN_TX2_ID=0x111, //TX2 ID
	  CAN_PID_TUNING_ID=0x209, //PID tuning config ID
	  CAN_GIMBAL_YAW_INTER_TRANSFER_ID=0x210, //Transfer Gimbal data to CAN2
	  CAN_GIMBAL_PITCH_INTER_TRANSFER_ID=0x211
} can_msg_id_e;

//RM electrical motor unified data struct
//RM电机统一数据结构体
typedef struct
{
    uint16_t ecd;//encoder
    int16_t speed_rpm;//round per minute
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;



//Enumerate TX2 data package type
//枚举声明TX2通信数据包类型
typedef enum{
	pitch_package=1,
	yaw_package=2,
	aim_package=3,
} tx2_package_type_e;

//TX2 to Gimbal Motor PID data package
//TX2到云台电机PID数据包
typedef struct{
	uint8_t kp;
	uint8_t ki;
	uint8_t kd;
	uint16_t error;
	uint16_t err_last;
	uint16_t power;
} tx2_gimbal_package_t;

//TX2 to Gimbal Motor aim coordinate location data package
//TX2到云台电机瞄准坐标数据包
typedef struct{
	uint16_t horizontal_pixel_buffer;
	uint16_t vertical_pixel_buffer;
	int32_t horizontal_pixel;
	int32_t vertical_pixel;
} tx2_aim_package_t;

//TX2 data receive struct
//TX2数据接收结构体
typedef struct
{
	uint16_t package_type;
  tx2_gimbal_package_t yaw_pid_package;
	tx2_gimbal_package_t pitch_pid_package;
	tx2_aim_package_t aim_data_package;
} tx2_measure_t;

//发送重设底盘电机ID命令
extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//Send TX2 Data CMD
extern void CAN_GIMBAL_GYRO_DATA_TX2(int16_t yaw, int16_t pitch);//-32767-32768
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
