/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成机械臂控制任务，ARM1和ARM2一同运行，ARM3和ARM4一同运行
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
	*  V2.0.0			May-18-2019			OSU-RM					2. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Gimbal_Task.h"

#include "main.h"

#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "buzzer.h"//加入蜂鸣器用于测试
#include "filter.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//软件复位Trigger
void SoftReset(void);

//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_arm1_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_arm1_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_arm1_motor.gimbal_motor_gyro_pid);                    \
			\
			PID_clear(&(gimbal_clear)->gimbal_arm2_motor.gimbal_motor_rpm_pid);  	\
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_arm2_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_arm2_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_arm2_motor.gimbal_motor_gyro_pid);                  \
			\
			PID_clear(&(gimbal_clear)->gimbal_arm2_motor.gimbal_motor_rpm_pid);  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control;


//发送的can 指令
static int16_t Arm1_Can_Set_Current = 0, Arm2_Can_Set_Current = 0, Arm3_Can_Set_Current = 0, Arm4_Can_Set_Current = 0;

//云台初始化
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//云台pid清零
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//云台状态设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//云台状态切换保存数据，例如从陀螺仪状态切换到编码器状态保存目标值
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//云台控制pid计算
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);



static void gimbal_motor_relative_angle_control_arm1(Gimbal_Motor_t *gimbal_motor);//机械臂1电机
static void gimbal_motor_relative_angle_control_arm2(Gimbal_Motor_t *gimbal_motor);//机械臂2电机
static void gimbal_motor_relative_angle_control_arm3(Gimbal_Motor_t *gimbal_motor);//机械臂3电机
static void gimbal_motor_relative_angle_control_arm4(Gimbal_Motor_t *gimbal_motor);//机械臂4电机

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);


static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);



static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

void GIMBAL_task(void *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    GIMBAL_Init(&gimbal_control);

//关闭自检    //判断电机是否都上线
//    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) || toe_is_error(TriggerMotorTOE) )
//    {
        vTaskDelay(10*GIMBAL_CONTROL_TIME);
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
//    }

    while (1)	
    {
				
				
				
        GIMBAL_Set_Mode(&gimbal_control);                    //设置云台控制模式

        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
						
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
				
        GIMBAL_Control_loop(&gimbal_control);                //云台控制PID计算


			
			
#if ARM1_TURN
        Arm1_Can_Set_Current = -gimbal_control.gimbal_arm1_motor.given_current;
#else
        Arm1_Can_Set_Current = gimbal_control.gimbal_arm1_motor.given_current;
#endif

#if ARM2_TURN
        Arm2_Can_Set_Current = -gimbal_control.gimbal_arm2_motor.given_current;
#else
        Arm2_Can_Set_Current = gimbal_control.gimbal_arm2_motor.given_current;
#endif

#if ARM3_TURN
        Arm3_Can_Set_Current = -gimbal_control.gimbal_arm3_motor.given_current;
#else
        Arm3_Can_Set_Current = gimbal_control.gimbal_arm3_motor.given_current;
#endif
				
#if ARM4_TURN
        Arm4_Can_Set_Current = -gimbal_control.gimbal_arm4_motor.given_current;
#else
        Arm4_Can_Set_Current = gimbal_control.gimbal_arm4_motor.given_current;
#endif
				
				
        //机械臂在遥控器掉线状态即relax 状态，can指令为0，不使用current设置为零的方法，是保证遥控器掉线一定使得云台停止
        if (!(toe_is_error(Arm1MotorTOE) && toe_is_error(Arm2MotorTOE) && toe_is_error(Arm3MotorTOE) && toe_is_error(Arm4MotorTOE)))
        {
            if (toe_is_error(DBUSTOE))
            {
                CAN_CMD_ARM(0, 0, 0, 0);
            }
            else
            {
                CAN_CMD_ARM(Arm1_Can_Set_Current, Arm2_Can_Set_Current, Arm3_Can_Set_Current, Arm4_Can_Set_Current);
            }
        }

				
				
#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @author         RM
  * @param[in]      yaw 中值
  * @param[in]      pitch 中值
  * @param[in]      yaw 最大相对角度
  * @param[in]      yaw 最小相对角度
  * @param[in]      pitch 最大相对角度
  * @param[in]      pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_arm1_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_arm1_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_arm1_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_arm2_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_arm2_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_arm2_motor.min_relative_angle = min_pitch;
}


/**
  * @brief          云台校准计算，将校准记录的最大 最小值 来计算云台 中值和最大最小机械角度
  * @author         RM
  * @param[in]      yaw 中值 指针
  * @param[in]      pitch 中值 指针
  * @param[in]      yaw 最大相对角度 指针
  * @param[in]      yaw 最小相对角度 指针
  * @param[in]      pitch 最大相对角度 指针
  * @param[in]      pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_arm2_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_arm2_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_arm1_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_arm1_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_arm2_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_arm2_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_arm1_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_arm1_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_arm1_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_arm1_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_arm1_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_arm2_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_arm2_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_arm2_motor.min_relative_angle = *min_pitch;

        return 1;
    }
    else
    {
        return 0;
    }
}

//校准计算，相对最大角度，云台中值
static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

const Gimbal_Motor_t *get_arm1_motor_point(void)
{
    return &gimbal_control.gimbal_arm1_motor;
}

const Gimbal_Motor_t *get_arm2_motor_point(void)
{
    return &gimbal_control.gimbal_arm2_motor;
}

const Gimbal_Motor_t *get_arm3_motor_point(void)
{
    return &gimbal_control.gimbal_arm3_motor;
}

const Gimbal_Motor_t *get_arm4_motor_point(void)
{
    return &gimbal_control.gimbal_arm4_motor;
}

//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{

		static const fp32 Arm_speed_pid[3] = {ARM_SPEED_PID_KP, ARM_SPEED_PID_KI, ARM_SPEED_PID_KD};
	
	
    //电机数据指针获取
    gimbal_init->gimbal_arm1_motor.gimbal_motor_measure = get_Arm1_Motor_Measure_Point();
    gimbal_init->gimbal_arm2_motor.gimbal_motor_measure = get_Arm2_Motor_Measure_Point();
    gimbal_init->gimbal_arm3_motor.gimbal_motor_measure = get_Arm3_Motor_Measure_Point();
    gimbal_init->gimbal_arm4_motor.gimbal_motor_measure = get_Arm4_Motor_Measure_Point();
    //陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    gimbal_init->gimbal_arm1_motor.gimbal_motor_mode = gimbal_init->gimbal_arm1_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_arm2_motor.gimbal_motor_mode = gimbal_init->gimbal_arm2_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
		gimbal_init->gimbal_arm3_motor.gimbal_motor_mode = gimbal_init->gimbal_arm3_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_arm4_motor.gimbal_motor_mode = gimbal_init->gimbal_arm4_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    
		//初始化ARM1电机pid
//    GIMBAL_PID_Init(&gimbal_init->gimbal_arm1_motor.gimbal_motor_relative_angle_pid, ARM_ENCODE_RELATIVE_PID_MAX_OUT, ARM_ENCODE_RELATIVE_PID_MAX_IOUT, ARM_ENCODE_RELATIVE_PID_KP, ARM_ENCODE_RELATIVE_PID_KI, ARM_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_arm1_motor.gimbal_motor_rpm_pid, PID_POSITION, Arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
    //初始化ARM2电机pid
//    GIMBAL_PID_Init(&gimbal_init->gimbal_arm2_motor.gimbal_motor_relative_angle_pid, ARM_ENCODE_RELATIVE_PID_MAX_OUT, ARM_ENCODE_RELATIVE_PID_MAX_IOUT, ARM_ENCODE_RELATIVE_PID_KP, ARM_ENCODE_RELATIVE_PID_KI, ARM_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_init->gimbal_arm2_motor.gimbal_motor_rpm_pid, PID_POSITION, Arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
    //初始化ARM3电机pid
//    GIMBAL_PID_Init(&gimbal_init->gimbal_arm3_motor.gimbal_motor_relative_angle_pid, ARM_ENCODE_RELATIVE_PID_MAX_OUT, ARM_ENCODE_RELATIVE_PID_MAX_IOUT, ARM_ENCODE_RELATIVE_PID_KP, ARM_ENCODE_RELATIVE_PID_KI, ARM_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_init->gimbal_arm3_motor.gimbal_motor_rpm_pid, PID_POSITION, Arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);
    //初始化ARM4电机pid
//    GIMBAL_PID_Init(&gimbal_init->gimbal_arm2_motor.gimbal_motor_relative_angle_pid, ARM_ENCODE_RELATIVE_PID_MAX_OUT, ARM_ENCODE_RELATIVE_PID_MAX_IOUT, ARM_ENCODE_RELATIVE_PID_KP, ARM_ENCODE_RELATIVE_PID_KI, ARM_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_init->gimbal_arm4_motor.gimbal_motor_rpm_pid, PID_POSITION, Arm_speed_pid, ARM_SPEED_PID_MAX_OUT, ARM_SPEED_PID_MAX_IOUT);

    //清除所有PID
    gimbal_total_pid_clear(gimbal_init);

    gimbal_init->gimbal_arm1_motor.relative_angle_set = gimbal_init->gimbal_arm1_motor.relative_angle;
    gimbal_init->gimbal_arm2_motor.relative_angle_set = gimbal_init->gimbal_arm2_motor.relative_angle;
    gimbal_init->gimbal_arm3_motor.relative_angle_set = gimbal_init->gimbal_arm3_motor.relative_angle;
    gimbal_init->gimbal_arm4_motor.relative_angle_set = gimbal_init->gimbal_arm4_motor.relative_angle;


}



#if GIMBAL_TEST_MODE

//jscope观察数据
int32_t arm1_ins_int_1000, arm2_ins_int_1000, arm1_ins_raw_int_1000;
int32_t arm1_ins_set_1000, arm2_ins_set_1000;
int32_t arm2_relative_set_1000, arm2_relative_angle_1000;
int32_t arm1_relative_set_1000, arm1_relative_angle_1000;
int32_t arm1_speed_int_1000, arm2_speed_int_1000;
int32_t arm1_speed_set_int_1000, arm2_speed_set_int_1000;


static void J_scope_gimbal_test(void)
{
    arm1_ins_int_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.absolute_angle * 1000);
    arm1_ins_set_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.absolute_angle_set * 1000);
    arm1_speed_int_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.motor_gyro * 1000);
    arm1_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.motor_gyro_set * 1000);
		arm1_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.relative_angle * 1000);
    arm1_relative_set_1000 = (int32_t)(gimbal_control.gimbal_arm1_motor.relative_angle_set * 1000);

    arm2_ins_int_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.absolute_angle * 1000);
    arm2_ins_set_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.absolute_angle_set * 1000);
    arm2_speed_int_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.motor_rpm * 1000);
    arm2_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.motor_rpm_set * 1000);
    arm2_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.relative_angle * -1000);
    arm2_relative_set_1000 = (int32_t)(gimbal_control.gimbal_arm2_motor.relative_angle_set * 1000);
	
}

#endif



static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
		gimbal_feedback_update->gimbal_arm1_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_arm1_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_arm1_motor.offset_ecd);
		gimbal_feedback_update->gimbal_arm1_motor.motor_rpm=0.25f*0.000415809748903494517209f*gimbal_feedback_update->gimbal_arm1_motor.gimbal_motor_measure->speed_rpm;//ARM1电机转速，3508全部相同
		
		gimbal_feedback_update->gimbal_arm2_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_arm2_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_arm2_motor.offset_ecd);
		gimbal_feedback_update->gimbal_arm2_motor.motor_rpm=0.25f*0.000415809748903494517209f*gimbal_feedback_update->gimbal_arm2_motor.gimbal_motor_measure->speed_rpm;//ARM2电机转速

		gimbal_feedback_update->gimbal_arm3_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_arm3_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_arm3_motor.offset_ecd);
		gimbal_feedback_update->gimbal_arm3_motor.motor_rpm=0.25f*0.000415809748903494517209f*gimbal_feedback_update->gimbal_arm3_motor.gimbal_motor_measure->speed_rpm;//ARM1电机转速，3508全部相同

		gimbal_feedback_update->gimbal_arm4_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_arm4_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_arm4_motor.offset_ecd);
		gimbal_feedback_update->gimbal_arm4_motor.motor_rpm=0.25f*0.000415809748903494517209f*gimbal_feedback_update->gimbal_arm4_motor.gimbal_motor_measure->speed_rpm;//ARM1电机转速，3508全部相同
}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

//云台状态切换保存，用于状态切换过渡

static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //机械臂1电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_arm1_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_arm1_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_arm1_motor.raw_cmd_current = gimbal_mode_change->gimbal_arm1_motor.current_set = gimbal_mode_change->gimbal_arm1_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_arm1_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODE && gimbal_mode_change->gimbal_arm1_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_mode_change->gimbal_arm1_motor.relative_angle_set = gimbal_mode_change->gimbal_arm1_motor.relative_angle;
		}
    gimbal_mode_change->gimbal_arm1_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_arm1_motor.gimbal_motor_mode;

		
    //机械臂2电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_arm2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_arm2_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_arm2_motor.raw_cmd_current = gimbal_mode_change->gimbal_arm2_motor.current_set = gimbal_mode_change->gimbal_arm2_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_arm2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODE && gimbal_mode_change->gimbal_arm2_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_mode_change->gimbal_arm2_motor.relative_angle_set = gimbal_mode_change->gimbal_arm2_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_arm2_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_arm2_motor.gimbal_motor_mode;
		
		 //机械臂3电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_arm3_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_arm3_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_arm3_motor.raw_cmd_current = gimbal_mode_change->gimbal_arm3_motor.current_set = gimbal_mode_change->gimbal_arm3_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_arm3_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODE && gimbal_mode_change->gimbal_arm3_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_mode_change->gimbal_arm3_motor.relative_angle_set = gimbal_mode_change->gimbal_arm3_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_arm3_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_arm3_motor.gimbal_motor_mode;
		
		//机械臂44电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_arm4_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_arm4_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_arm4_motor.raw_cmd_current = gimbal_mode_change->gimbal_arm4_motor.current_set = gimbal_mode_change->gimbal_arm4_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_arm4_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODE && gimbal_mode_change->gimbal_arm4_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_mode_change->gimbal_arm4_motor.relative_angle_set = gimbal_mode_change->gimbal_arm4_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_arm4_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_arm4_motor.gimbal_motor_mode;
}


//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //ARM1不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_arm1_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_arm1_motor);
    }
    else if (gimbal_control_loop->gimbal_arm1_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_arm1(&gimbal_control_loop->gimbal_arm1_motor);//机械臂1
			
			//软件复位程序
			if (switch_is_down(gimbal_control_loop->gimbal_rc_ctrl->rc.s[1]))
			{
        SoftReset();
			}
			//软件复位程序
		
    }
		
		
    //ARM2不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_arm2_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_arm2_motor);
    }
    else if (gimbal_control_loop->gimbal_arm2_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_arm2(&gimbal_control_loop->gimbal_arm2_motor);
    }
		
		//ARM3不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_arm3_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_arm3_motor);
    }
    else if (gimbal_control_loop->gimbal_arm3_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_arm3(&gimbal_control_loop->gimbal_arm3_motor);
    }
		
		//ARM4不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_arm4_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_arm4_motor);
    }
    else if (gimbal_control_loop->gimbal_arm4_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_arm4(&gimbal_control_loop->gimbal_arm4_motor);
    }
}



static void gimbal_motor_relative_angle_control_arm1(Gimbal_Motor_t *gimbal_motor)//机械臂1电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }
				
		static fp32 delta_arm1;//ARM1电机变化量
//		static fp32 const data_to_deg_ratio=0.001745329252f;//2*pi弧度/360角度/10精度

		//限速
		gimbal_control.gimbal_arm1_motor.motor_rpm=fp32_constrain(gimbal_control.gimbal_arm1_motor.motor_rpm, -ARM_MAX_SPEED, ARM_MAX_SPEED);
		
		delta_arm1=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[Arm12Channel])*ARM_RC_SEN
						 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.y)*ARM_MOUSE_SEN;
		

		//更改motor_rpm_set的值来达到锁定速度环
		gimbal_motor->motor_rpm_set=delta_arm1;
		//速度环PID
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_rpm_pid, gimbal_motor->motor_rpm, gimbal_motor->motor_rpm_set);
		//控制值赋值
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
}

static void gimbal_motor_relative_angle_control_arm2(Gimbal_Motor_t *gimbal_motor)//机械臂2电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		static fp32 delta_arm2;//ARM2电机变化量
//		static fp32 const data_to_deg_ratio=0.001745329252f;//2*pi弧度/360角度/10精度
		
		
		//限速
		gimbal_control.gimbal_arm2_motor.motor_rpm=fp32_constrain(gimbal_control.gimbal_arm2_motor.motor_rpm, -ARM_MAX_SPEED, ARM_MAX_SPEED);

		delta_arm2=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[Arm12Channel])*ARM_RC_SEN
							 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.y)*ARM_MOUSE_SEN;
		
		//更改motor_rpm_set的值来达到锁定速度环
		gimbal_motor->motor_rpm_set=delta_arm2;
		//速度环PID
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_rpm_pid, gimbal_motor->motor_rpm, gimbal_motor->motor_rpm_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control_arm3(Gimbal_Motor_t *gimbal_motor)//机械臂3电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		static fp32 delta_arm3;//ARM2电机变化量
//		static fp32 const data_to_deg_ratio=0.001745329252f;//2*pi弧度/360角度/10精度
		
		
		//限速
		gimbal_control.gimbal_arm3_motor.motor_rpm=fp32_constrain(gimbal_control.gimbal_arm3_motor.motor_rpm, -ARM_MAX_SPEED, ARM_MAX_SPEED);

		delta_arm3=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[Arm34Channel])*ARM_RC_SEN
							 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.x)*ARM_MOUSE_SEN;
		
		//更改motor_rpm_set的值来达到锁定速度环
		gimbal_motor->motor_rpm_set=delta_arm3;
		//速度环PID
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_rpm_pid, gimbal_motor->motor_rpm, gimbal_motor->motor_rpm_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control_arm4(Gimbal_Motor_t *gimbal_motor)//机械臂4电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		static fp32 delta_arm4;//ARM2电机变化量
//		static fp32 const data_to_deg_ratio=0.001745329252f;//2*pi弧度/360角度/10精度
		
		
		//限速
		gimbal_control.gimbal_arm4_motor.motor_rpm=fp32_constrain(gimbal_control.gimbal_arm4_motor.motor_rpm, -ARM_MAX_SPEED, ARM_MAX_SPEED);

		delta_arm4=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[Arm34Channel])*ARM_RC_SEN
							 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.x)*ARM_MOUSE_SEN;
		
		//更改motor_rpm_set的值来达到锁定速度环
		gimbal_motor->motor_rpm_set=delta_arm4;
		//速度环PID
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_rpm_pid, gimbal_motor->motor_rpm, gimbal_motor->motor_rpm_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
