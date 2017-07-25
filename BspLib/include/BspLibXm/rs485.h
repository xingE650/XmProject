#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "main_config.h"

/**********************************************************************************************************************/
void rs485Init(USART_TypeDef* USARTx , uint32_t BaudRate , uint8_t GPIO_AF);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef DELAY_H
