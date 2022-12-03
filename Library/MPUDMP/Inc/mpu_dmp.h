/**
 * 三方库来源
 * https://invensense.tdk.com/developers/software-downloads/
 *
 * 当前版本
 * eMD 6.12
 */

#ifndef STM32_DMP_H
#define STM32_DMP_H

#include "stm32f1xx_hal.h"

uint8_t MPU_DMP_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU_DMP_GetData(float *pitch, float *roll, float *yaw);

uint8_t stm32_i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t stm32_i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t stm32_delay_ms(unsigned long num_ms);
uint8_t stm32_get_ms(unsigned long *count);

#endif //STM32_DMP_H
