#include "stm32f4xx.h"
#include "gpio.h"
#include "main.h"


void PC_Init(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//PC0 init->N2 on board
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//PC4 init->N1 on board
}

void Set_PC0_Low(void){
    GPIO_WriteBit(GPIOC,GPIO_Pin_0,0);
}

void Set_PC0_High(void){
    GPIO_WriteBit(GPIOC,GPIO_Pin_0,1);
}




