#ifndef HMC_5883L_H
#define HMC_5883L_H

#define USE_MPU_6050
#define MPU6050_ADDRESS 0x68

#include "stm32f1xx_hal.h"

uint8_t HMC_Init(I2C_HandleTypeDef *hi2c);

uint8_t HMC_GetAngle(float *ax, float *ay, float *az);

#endif //HMC_5883L_H
