#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f1xx_hal.h"

void RETARGET_Init(UART_HandleTypeDef *huart);

#endif //#ifndef _RETARGET_H__