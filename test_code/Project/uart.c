//File Created by Tony Han. The Ohio State University
//Created Date: 2/2/2019


#include "stm32f4xx.h"
#include "uart.h"
#include "stm32f4xx_usart.h"
void uart_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB1_PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	
}