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
#define ARM_CAN CAN1


/* Enumerate CAN send and receive ID */
/* 枚举声明CAN收发ID*/
typedef enum
{
	  CAN_AIM_DATA_ID = 0x300,//自瞄数据ID
	
		CAN_CHASSIS_ALL_ID = 0x200,//发送ID
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_ARM1_MOTOR_ID = 0x205,
    CAN_ARM2_MOTOR_ID = 0x206,
    CAN_ARM3_MOTOR_ID = 0x207,
		CAN_ARM4_MOTOR_ID = 0x208,//自定义3508电机
		CAN_ARM_ALL_ID = 0x1FF,//发送ID 3508
		
	
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


//发送重设底盘电机ID命令
extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_ARM(int16_t arm1, int16_t arm2, int16_t arm3, int16_t arm4);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//返回机械臂1电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Arm1_Motor_Measure_Point(void);
//返回机械臂2电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Arm2_Motor_Measure_Point(void);
//返回机械臂3电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Arm3_Motor_Measure_Point(void);
//返回机械臂4电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Arm4_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
////自瞄数据
//extern tx2_aim_package_t tx2;
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
