/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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

#include "Gimbal_Task.h"
#include "stdlib.h"
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
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

//云台控制所有相关数据
extern Gimbal_Control_t gimbal_control;


//发送的can 指令
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;

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
//设置云台控制量
//static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制pid计算
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);


//static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_absolute_angle_control_yaw(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control_yaw(Gimbal_Motor_t *gimbal_motor);//普通模式下YAW电机
static void gimbal_motor_relative_angle_control_pitch(Gimbal_Motor_t *gimbal_motor);//普通模式下PITCH电机
static void gimbal_motor_aim_control_gyro_yaw(Gimbal_Motor_t *gimbal_motor);//自瞄模式-陀螺仪绝对位置-YAW电机
static void gimbal_motor_aim_control_pitch(Gimbal_Motor_t *gimbal_motor);//自瞄模式下PITCH电机

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

//在陀螺仪角度控制下，对控制的目标值进限制以防超最大相对角度
//static void GIMBAL_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
//static void GIMBAL_relative_angle_limit_yaw(Gimbal_Motor_t *gimbal_motor, fp32 add);
//static void GIMBAL_relative_angle_limit_pitch(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

//陀螺仪绝对角度临时参照值
fp32 temp_absolute_angle_reference;



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
    //射击初始化
    shoot_init();
    //判断电机是否都上线
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) || toe_is_error(TriggerMotorTOE))
    {
        vTaskDelay(10*GIMBAL_CONTROL_TIME);
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
    }


    while (1)	
    {
				
				
				
        GIMBAL_Set_Mode(&gimbal_control);                    //设置云台控制模式

        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
						
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
				
//        GIMBAL_Set_Contorl(&gimbal_control);                 //设置云台控制量
			
        GIMBAL_Control_loop(&gimbal_control);                //云台控制PID计算

        Shoot_Can_Set_Current = shoot_control_loop();        //射击任务控制循环
				
	
			
#if YAW_TURN
        Yaw_Can_Set_Current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        Pitch_Can_Set_Current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

				
				
				
				
        //云台在遥控器掉线状态即relax 状态，can指令为0，不使用current设置为零的方法，是保证遥控器掉线一定使得云台停止
        if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE)&& toe_is_error(TriggerMotorTOE)))// 
        {
            if (toe_is_error(DBUSTOE))
            {
                CAN_CMD_GIMBAL(0, 0, 0, 0);
            }
            else
            {
                CAN_CMD_GIMBAL(Yaw_Can_Set_Current, Pitch_Can_Set_Current, Shoot_Can_Set_Current, 0);
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
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
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
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;

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

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    //陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    //清除所有PID
    gimbal_total_pid_clear(gimbal_init);

    GIMBAL_Feedback_Update(gimbal_init);

    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;


    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;


}



#if GIMBAL_TEST_MODE

//jscope观察数据
int32_t yaw_ins_int_1000, pitch_ins_int_1000, yaw_ins_raw_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_relative_set_1000, yaw_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;


//jscope自定义观察数据
int32_t filtered_final_yaw_angle_set_jscope;
int32_t filtered_final_pitch_angle_set_jscope;
int32_t delayed_yaw_absolute_angle_jscope;
int32_t delayed_pitch_relative_angle_jscope;
int32_t filtered_horizontal_pixel_jscope;
int32_t filtered_yaw_motor_speed_jscope;
int32_t filtered_vertical_pixel_jscope;
int32_t filtered_pitch_motor_speed_jscope;

int32_t temp_absolute_angle_reference_jscope;

int32_t prediciton_filtered_final_yaw_angle_set_jscope;

//传出数据
extern int32_t final_yaw_angle_set;//最终改变的YAW轴角度值
extern int32_t final_pitch_angle_set;//最终改变的PITCH轴角度值

/////////////////////////////////////////////////
//外部文件传入数据
fp32 *filtered_final_angle_set;
fp32 *filtered_aim_data;
fp32 delayed_yaw_absolute_angle;
fp32 delayed_pitch_relative_angle;//定义从user_task.c extern的group delay后的relative angle的值
/////////////////////////////////////////////////

static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);
		yaw_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.relative_angle * 1000);
    yaw_relative_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.relative_angle_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * -1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
				
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		filtered_horizontal_pixel_jscope=(int32_t)(filtered_aim_data[0]-YAW_MID);//相对 相机坐标系x轴 中点的角度 = 滤波后的x轴自瞄数据 - x轴中心点
		filtered_vertical_pixel_jscope=(int32_t)(filtered_aim_data[1]-PITCH_MID);//相对 相机坐标系y轴 中点的角度 = 滤波后的y轴自瞄数据 - y轴中心点	
		filtered_yaw_motor_speed_jscope=(int32_t)(filtered_aim_data[2]*1000);//滤波后的YAW轴电机速度
		filtered_pitch_motor_speed_jscope=(int32_t)(filtered_aim_data[3]*1000);//滤波后的PITCH轴电机速度
	
		delayed_yaw_absolute_angle_jscope=(int32_t)(delayed_yaw_absolute_angle*RAD_TO_DEGREE);
		delayed_pitch_relative_angle_jscope=(int32_t)(delayed_pitch_relative_angle*-RAD_TO_DEGREE);
		
		//YAW陀螺仪最终角度 是个定值
		final_yaw_angle_set=(int32_t)(filtered_horizontal_pixel_jscope-(delayed_yaw_absolute_angle*RAD_TO_DEGREE));
		//PITCH编码器最终角度 是个定值
		final_pitch_angle_set=(int32_t)(-filtered_vertical_pixel_jscope-gimbal_control.gimbal_pitch_motor.relative_angle*RAD_TO_DEGREE);//不需要使用delay后的编码器，别问，我也不知道为什么
				
		filtered_final_yaw_angle_set_jscope=(int32_t)(filtered_final_angle_set[0]);
		filtered_final_pitch_angle_set_jscope=(int32_t)(filtered_final_angle_set[1]);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		temp_absolute_angle_reference_jscope=(int32_t)(temp_absolute_angle_reference*1000);
		yaw_ins_raw_int_1000=(int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_raw * 1000);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		prediciton_filtered_final_yaw_angle_set_jscope=(int32_t)(filtered_final_angle_set[0]-PREDICTION_TIME*RAD_TO_DEGREE*filtered_aim_data[2]);
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
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

	
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    
		//将从陀螺仪中直接读取的数值重命名为absolute_angle_raw
		gimbal_feedback_update->gimbal_yaw_motor.absolute_angle_raw = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
			
		
		///////////////////////////////////////////////////////////////////////////////////////////////
		//重新计算absolute_angle，使得每次开启自瞄时当前位置的数值为0//不稳定，开启自瞄时会抖动
		gimbal_feedback_update->gimbal_yaw_motor.absolute_angle=gimbal_feedback_update->gimbal_yaw_motor.absolute_angle_raw-temp_absolute_angle_reference;
		if(gimbal_feedback_update->gimbal_yaw_motor.absolute_angle<=(temp_absolute_angle_reference-PI))
		{
			gimbal_feedback_update->gimbal_yaw_motor.absolute_angle+=2*PI;
		}
		if(gimbal_feedback_update->gimbal_yaw_motor.absolute_angle>=PI)
		{
			gimbal_feedback_update->gimbal_yaw_motor.absolute_angle-=2*PI;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////
    
		
		
		gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);

    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
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
    //YAW电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
				
			
//				temp_absolute_angle_reference=gimbal_mode_change->gimbal_yaw_motor.absolute_angle_raw;
		}
		//新加入自瞄控制模式
		else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_AIM && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIM)
    {

				//陀螺仪自瞄-状态切换
				temp_absolute_angle_reference=gimbal_mode_change->gimbal_yaw_motor.absolute_angle_raw;//将absolute_angle_raw作为参照值
				vTaskDelay(10);//以防开启自瞄时抖动
				
		
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

		
    //PITCH电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
		//新加入自瞄控制模式
		else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_AIM && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIM)
    {
        //编码器自瞄-状态切换
				gimbal_mode_change->gimbal_pitch_motor.relative_angle=0;
				gimbal_mode_change->gimbal_pitch_motor.offset_ecd=gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_measure->ecd;
				gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = 0;
//			gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //YAW不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
			//gyro角度控制
        gimbal_motor_absolute_angle_control_yaw(&gimbal_control_loop->gimbal_yaw_motor);//跟随模式-云台控制
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_yaw(&gimbal_control_loop->gimbal_yaw_motor);//普通模式-云台控制
			
			
			
			
			//软件复位程序
			if (switch_is_down(gimbal_control_loop->gimbal_rc_ctrl->rc.s[1]))
			{
        SoftReset();
			}
			//软件复位程序
			
			
			
			

    }
		//新加入自瞄控制
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIM)
    {
        //自瞄模式-编码器
        //gimbal_motor_aim_control_yaw(&gimbal_control_loop->gimbal_yaw_motor);
				//自瞄模式-陀螺仪
				gimbal_motor_aim_control_gyro_yaw(&gimbal_control_loop->gimbal_yaw_motor);
    }
		
				
		
    //PITCH不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
				//enconde角度控制
        gimbal_motor_relative_angle_control_pitch(&gimbal_control_loop->gimbal_pitch_motor);   	
//				//gyro角度控制
//        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde角度控制
        gimbal_motor_relative_angle_control_pitch(&gimbal_control_loop->gimbal_pitch_motor);
    }
		//新加入自瞄控制
		else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AIM)
    {
        //enconde角度控制
        gimbal_motor_aim_control_pitch(&gimbal_control_loop->gimbal_pitch_motor);
    }
}


//弃用
//static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }
//    //角度环，速度环串级pid调试
//    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
//    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
//    //控制值赋值
//    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//}
//弃用

//跟随模式下无法自瞄，仅为驾驶车辆用
static void gimbal_motor_absolute_angle_control_yaw(Gimbal_Motor_t *gimbal_motor)//跟随模式下YAW电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }
				
		static fp32 delta_yaw;//yaw电机角度目标变量
		delta_yaw=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[2])*-0.00005f+(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.x)*-0.0075f;

		//角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle+delta_yaw, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


//自瞄模式-陀螺仪-YAW电机
static void gimbal_motor_aim_control_gyro_yaw(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		static fp32 delta_yaw;//yaw电机角度目标变量
		static fp32 final_absolute_yaw_angle_set; //yaw电机最终相对目标角度

		
		delta_yaw=(fp32)((filtered_final_angle_set[0]-PREDICTION_TIME*RAD_TO_DEGREE*filtered_aim_data[2])//预测
										*-DEGREE_TO_RAD*1.0f);

		//用摇杆/鼠标更改absolute_angle_set的值，在一定范围内临时手动调整准心
		gimbal_motor->absolute_angle_set=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[2])*-0.0005f+(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.x)*-0.0035f;


		
		//判断是否跟丢
		if (tx2.raw_horizontal_pixel==9999 || tx2.raw_horizontal_pixel==0)//如果原始自瞄数据返回跟丢
		{		
			final_absolute_yaw_angle_set=gimbal_motor->absolute_angle;//set设为当前角度使云台停止移动
		}
		else
		{
			tx2.horizontal_pixel=tx2.raw_horizontal_pixel;//赋值自瞄数据
			
			//改变最终绝对角度
			
			final_absolute_yaw_angle_set=gimbal_motor->absolute_angle_set+delta_yaw;
			
		}
		
		



//		//限制云台yaw电机左右幅度
//		fp32 yaw_limit=data_to_deg_ratio*900;//增大扩展左右界，减小收缩左右界
//		if(final_absolute_yaw_angle_set>yaw_limit)//左界
//		{
//			final_absolute_yaw_angle_set=yaw_limit;
//			buzzer_on(60,5000);
//		}
//		else if(final_absolute_yaw_angle_set<-yaw_limit)//右界
//		{
//			final_absolute_yaw_angle_set=-yaw_limit;
//			buzzer_on(60,5000);
//		}
//		else
//		{
//			buzzer_off();
//		}
		
	
    //角度环，速度环串级pid调试

/*******************************************************************/
/*************当error大时手动target超前 已测试 效果好***************/
/*******************************************************************/
    if(abs((int)delta_yaw)>15){
			if(delta_yaw>15){//If left to right, need faster?
				gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, final_absolute_yaw_angle_set, gimbal_motor->motor_gyro*0.85);
			}
			else if (delta_yaw<-15){//If right to left
				gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, final_absolute_yaw_angle_set, gimbal_motor->motor_gyro*0.85);
			}
		}
		else{//If the pixel error is small(close to target)
			gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, final_absolute_yaw_angle_set, gimbal_motor->motor_gyro);
		}
/********************************************************************/
/*************************不是玄学勿删！*****************************/
/********************************************************************/
		
			gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
		//控制值赋值
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
}
//自瞄模式-通用PITCH电机
static void gimbal_motor_aim_control_pitch(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		static fp32 delta_pitch;//pitch电机角度目标变量
		static fp32 final_relative_pitch_angle_set;

		

		delta_pitch=(fp32)(filtered_final_angle_set[1])
											*-DEGREE_TO_RAD*1.0f;
    
//		//更改relative_angle_set的值来达到锁定位置环
//		gimbal_motor->relative_angle_set+=delta_pitch;
		
//		/******************I THINK HERE IS A PROBLEM***********************/
		//改变最终绝对角度
		final_relative_pitch_angle_set=gimbal_motor->relative_angle_set+delta_pitch;
//		/******************************************************************/
//		//Micro adjustment by mousing when aiming assistant is on.
//		final_relative_pitch_angle_set=gimbal_control.gimbal_rc_ctrl->mouse.y*-0.0015;
		
		//判断是否跟丢
		if (tx2.raw_vertical_pixel==9999 || tx2.raw_vertical_pixel==0)//如果原始自瞄数据返回跟丢
		{		
			final_relative_pitch_angle_set=gimbal_motor->relative_angle;//set设为当前角度使云台停止移动
		}
		else
		{
			tx2.vertical_pixel=tx2.raw_vertical_pixel;//赋值自瞄数据
			//改变最终绝对角度
			final_relative_pitch_angle_set=gimbal_motor->relative_angle_set+delta_pitch;
		}
		
	
		
//		//限制云台pitch电机上下幅度
//		fp32 pitch_limit=data_to_deg_ratio*300;//减小降低上界，增大提高上界
//		fp32 pitch_limit_offset=DEGREE_TO_RAD*100;//减小降低下界，增大提高下界
//		if(final_relative_pitch_angle_set>pitch_limit-pitch_limit_offset)//下界
//		{
//			final_relative_pitch_angle_set=pitch_limit-pitch_limit_offset;
//			buzzer_on(60,5000);
//		}
//		else if(final_relative_pitch_angle_set<-pitch_limit)//上界
//		{
//			final_relative_pitch_angle_set=-pitch_limit;
//			buzzer_on(60,5000);
//		}
//		else
//		{
//			//buzzer_off();
//		}
		
		//角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, final_relative_pitch_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_motor_relative_angle_control_yaw(Gimbal_Motor_t *gimbal_motor)//普通模式下YAW电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }
				
		static fp32 delta_yaw;//yaw电机角度目标变量
				
		delta_yaw=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[2])*-0.000005f
						 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.x)*-0.00025f;////-0.000025f//relative_angle_set鼠标用这个系数///////-0.0025f;//relative_angle+delta_yaw鼠标用这个系数
		
		
		//校准摄像头虚拟深度时用
		tx2.horizontal_pixel=tx2.raw_horizontal_pixel;//赋值自瞄数据
		
		//更改relative_angle_set的值来达到锁定位置环
		gimbal_motor->relative_angle_set+=delta_yaw;

		//限制云台yaw电机左右幅度
		fp32 yaw_limit=DEGREE_TO_RAD*900;//增大扩展左右界，减小收缩左右界
		if(gimbal_motor->relative_angle_set>yaw_limit)//左界
		{
			gimbal_motor->relative_angle_set=yaw_limit;
			buzzer_on(60,5000);
		}
		else if(gimbal_motor->relative_angle_set<-yaw_limit)//右界
		{
			gimbal_motor->relative_angle_set=-yaw_limit;
			buzzer_on(60,5000);
		}
		else
		{
			buzzer_off();
		}
	
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    
		//读取编码器的值
		//(gimbal_motor->gimbal_motor_measure->ecd<=5097)//最右 1417（5143）；中间 1C17（7191）；最左0418（8191+1048）

		//控制值赋值
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
}

static void gimbal_motor_relative_angle_control_pitch(Gimbal_Motor_t *gimbal_motor)//普通模式下PITCH电机
{
    if (gimbal_motor == NULL)
    {
        return;
    }

		static fp32 delta_pitch;//pitch电机角度目标变量

		delta_pitch=(fp32)(gimbal_control.gimbal_rc_ctrl->rc.ch[3])*-0.000005f
							 +(fp32)(gimbal_control.gimbal_rc_ctrl->mouse.y)*0.00025f;////0.000025f//relative_angle_set时鼠标用这个系数///////-0.0025f;//relative_angle+delta_pitch鼠标用这个系数
		
		//校准摄像头虚拟深度时用
		tx2.vertical_pixel=tx2.raw_vertical_pixel;//赋值自瞄数据
		
		//更改relative_angle_set的值来达到锁定位置环
		gimbal_motor->relative_angle_set+=delta_pitch;
		
//		//限制云台pitch电机上下幅度
//		fp32 pitch_limit=DEGREE_TO_RAD*300;//减小降低上界，增大提高上界
//		fp32 pitch_limit_offset=DEGREE_TO_RAD*100;//减小降低下界，增大提高下界
//		if(gimbal_motor->relative_angle_set>pitch_limit-pitch_limit_offset)//下界
//		{
//			gimbal_motor->relative_angle_set=pitch_limit-pitch_limit_offset;
//			buzzer_on(60,5000);
//		}
//		else if(gimbal_motor->relative_angle_set<-pitch_limit)//上界
//		{
//			gimbal_motor->relative_angle_set=-pitch_limit;
//			buzzer_on(60,5000);
//		}
//		else
//		{
//			//buzzer_off();
//		}
		
		//角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
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
