#ifndef GPIO_H
#define GPIO_H

#include "main.h"
#include "stm32f4xx_gpio.h"
typedef enum
{
	I1=1,//PF1	赋值为1以免与NULL=0冲突
	I2,	//PF0
	J1,	//PE5
	J2,	//PE4
	K1,	//PE6
	K2,	//PE12
	L1,	//PC2
	L2,	//PB0
	M1,	//PC3
	M2,	//PB1
	N1,	//PC4
	N2,	//PC0
	O1,	//PC5
	O2,	//PC1
	P1,	//PA5
	P2,	//PA4
//	Q1, //PF10，被17mm微动开关占用
	Q2,//PI9
//	R1, GND
//	R2,	+5V
} GPIO_ID_E;

typedef enum
{
	A=1,//PI0 	TIM5_CH4	赋值为1以免与NULL=0冲突
	B,	//PH12	TIM5_CH3
	C,	//PH11	TIM5_CH2
	D,	//PH10	TIM5_CH1
	E,	//PD15	TIM4_CH4
	F,	//PD14	TIM4_CH3
	G,	//PD13	TIM4_CH2
	H,	//PD12	TIM4_CH1
	
	S,	//PA0		TIM2_CH1		
	T,	//PA1		TIM2_CH2
	U,	//PA2		TIM2_CH3
	V,	//PA3		TIM2_CH4
	W,	//PI5		TIM8_CH1
	X,	//PI6		TIM8_CH2
	Y,	//PI7		TIM8_CH3
	Z,	//PI2		TIM8_CH4
} PWM_ID_E;

typedef struct
{
	GPIO_ID_E GPIO_ID;//GPIO开发板字母
	GPIO_TypeDef * GPIOX;//GPIO类型
	uint16_t GPIO_Pin_x;//GPIO管脚
	uint32_t RCC_AHB1Periph_GPIOX;//GPIO端口AHB总线
} User_GPIO_X;

typedef struct
{
	PWM_ID_E PWM_ID;//PWM开发板字母
	GPIO_TypeDef * GPIOX;//GPIO类型
	TIM_TypeDef * TIMx;//计时器
	uint16_t GPIO_Pin_x;//GPIO管脚
	uint32_t RCC_AHB1Periph_GPIOX;//GPIO端口AHB总线
	uint32_t RCC_APBxPeriph_TIMx;//GPIO时钟APB总线
	uint32_t CHANNEL;//GPIO时钟通道
	uint8_t GPIO_PinSourcex;
	uint8_t GPIO_AF_TIMx;//GPIO复用计时器
} User_PWM_X;

extern void User_GPIO_Init(User_GPIO_X* gpio_x);//初始化自定义GPIO端口和管脚
extern void User_PWM_Init(User_PWM_X* pwm_x);//初始化自定义PWM端口，管脚和时钟
extern void Set_User_GPIO(User_GPIO_X* gpio_x, FunctionalState Status);//设置自定义GPIO状态
extern void Set_User_PWM(User_PWM_X* pwm_x, uint32_t pulse_width);//设置自定义PWM脉宽
extern User_GPIO_X user_gpio;
extern User_PWM_X user_pwm;
#endif
