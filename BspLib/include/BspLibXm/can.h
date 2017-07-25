#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx.h"

void HF_CAN_Init(CAN_TypeDef *CANx, uint8_t GPIO_AF);
void HF_CANTX_Message(CAN_TypeDef* CANx, unsigned int ID, uint8_t *TxBuf);

#ifdef __cplusplus
}
#endif 

#endif // #ifndef CAN_H


