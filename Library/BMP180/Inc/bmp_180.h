/**
 * 使用博世官方驱动
 * https://github.com/BoschSensortec/BMP180_driver
 */

#ifndef BMP_180_H
#define BMP_180_H

#include "stm32f1xx_hal.h"

uint8_t BMP_Init(I2C_HandleTypeDef *hi2c);

/**
 * 获取温度值
 * @param temperature 温度值, 单位℃
 * @return 执行结果
 */
uint8_t BMP_GetTemperature(float *temperature);

/**
 * 获取气压
 * @param airPressure
 * @return 执行结果
 */
uint8_t BMP_GetAirPressure(float *airPressure);

/**
 * 根据气压计算海拔
 * @param temperature 温度值, 单位℃
 * @param airPressure 气压值, 单位hPa
 * @param altitude 海拔值, 单位m
 * @return 执行结果
 */
uint8_t BMP_GetAltitude(float *temperature, float *airPressure, float *altitude);

#endif //BMP_180_H
