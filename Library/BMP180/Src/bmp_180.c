#include <math.h>
#include "bmp_180.h"
#include "bmp180.h"

#define BMP_P0              (1013.25)
#define BMP_POWER           (1 / 5.257)
#define BMP_FAHRENHEIT      (273.15)
#define BMP_DENOMINATOR     (0.0065)

I2C_HandleTypeDef *gBmpHi2c;
struct bmp180_t gBmp180_t = {0};

uint8_t BMP_bus_read(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length);
uint8_t BMP_bus_write(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length);
void BMP_delay_msec(uint32_t value);

uint8_t BMP_Init(I2C_HandleTypeDef *hi2c) {
    gBmpHi2c = hi2c;
    gBmp180_t.dev_addr = BMP180_I2C_ADDR;
    gBmp180_t.bus_read = (s8 (*)(u8, u8, u8 *, u8)) BMP_bus_read;
    gBmp180_t.bus_write = (s8 (*)(u8, u8, u8 *, u8)) BMP_bus_write;
    gBmp180_t.delay_msec = (void (*)(u32)) BMP_delay_msec;
    return bmp180_init(&gBmp180_t) && bmp180_get_calib_param();
}

uint8_t BMP_GetTemperature(float *temperature) {
    *temperature = (float) bmp180_get_temperature(
            bmp180_get_uncomp_temperature()
    ) / 10.0F;
    return 0;
}

uint8_t BMP_GetAirPressure(float *airPressure) {
    *airPressure = (float) bmp180_get_pressure(
            bmp180_get_uncomp_pressure()
    ) / 100.0F;
    return 0;
}

uint8_t BMP_GetAltitude(float *temperature, float *airPressure, float *altitude) {
    *altitude = (pow(BMP_P0 / (*airPressure), BMP_POWER) - 1) * (*temperature + BMP_FAHRENHEIT) / BMP_DENOMINATOR;
    return 0;
}

uint8_t BMP_bus_read(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length) {
    return HAL_I2C_Mem_Read(
            gBmpHi2c,
            device_addr << 1,
            register_addr,
            1,
            register_data,
            write_length,
            0xFFFF
    );
}

uint8_t BMP_bus_write(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length) {
    return HAL_I2C_Mem_Write(
            gBmpHi2c,
            device_addr << 1,
            register_addr,
            1,
            register_data,
            write_length,
            0xFFFF
    );
}

void BMP_delay_msec(uint32_t value) {
    HAL_Delay(value);
}