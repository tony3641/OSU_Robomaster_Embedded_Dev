#include "Receive_Task.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"




void ReceiveTask(void *pvParameters){
	USART_TypeDef UART_TX2;
  USART_DeInit(UART_TX2);
	
	RCC_APB2PeriphResetCmd(RCC_APB1Periph_USART7);
  
  
}


