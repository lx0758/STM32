#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

void MPU_Init(I2C_HandleTypeDef *hi2c);

uint8_t MPU_GetGyroscope(float *gx, float *gy, float *gz);
uint8_t MPU_GetAccelerometer(float *ax, float *ay, float *az);
uint8_t MPU_GetTemperature(float *temperature);

#endif //MPU6050_H
