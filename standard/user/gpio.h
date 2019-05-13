#ifndef GPIO_H
#define GPIO_H
#include "main.h"
#include "gpio.h"
#include "stm32f4xx_gpio.h"

void PC_Init(void);
void Set_GPIO_PC(GPIO_TypeDef* GPIOx, uint16_t Pin, unsigned short voltage_value);

#endif
