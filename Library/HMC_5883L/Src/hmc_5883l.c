#include <math.h>
#include "hmc_5883l.h"

#define HMC_ADDRESS               0x1E

#define HMC_MEM_ADDR_CONF_A       0x00
#define HMC_MEM_ADDR_CONF_B       0x01
#define HMC_MEM_ADDR_MODE         0x02
#define HMC_MEM_ADDR_DATA_OUT     0x03

#define HMC_SAMPLING_1            0b00000000
#define HMC_SAMPLING_2            0b00100000
#define HMC_SAMPLING_4            0b01000000
#define HMC_SAMPLING_8            0b01100000

#define HMC_SPEED_0_75            0b00000000
#define HMC_SPEED_1_5             0b00000100
#define HMC_SPEED_3               0b00001000
#define HMC_SPEED_7_5             0b00001100
#define HMC_SPEED_15              0b00010000
#define HMC_SPEED_30              0b00010100
#define HMC_SPEED_75              0b00011000

#define HMC_GAIN_1370             0b00000000
#define HMC_GAIN_1090             0b00100000
#define HMC_GAIN_820              0b01000000
#define HMC_GAIN_660              0b01100000
#define HMC_GAIN_440              0b10000000
#define HMC_GAIN_390              0b10100000
#define HMC_GAIN_330              0b11000000
#define HMC_GAIN_230              0b11100000

#define HMC_MODE_COILED           0b00000000
#define HMC_MODE_SINGLE           0b00000001

#define HMC_PI                    acos(-1)

#define MPU6050_INT_PIN_CFG       0x37
#define MPU6050_USER_CTRL         0x6A
#define MPU6050_PWR_MGMT_1        0x6B

I2C_HandleTypeDef *gHmcHi2c;
float gHmcGain;

/**
 * MPU 初始化
 * @return
 */
uint8_t HMC_MpuInit();
/**
 * 配置寄存器 A (采样数量 & 输出速率)
 * @param sampleAndSpeed
 * @return
 */
uint8_t HMC_SetSampleAndSpeed(uint8_t sampleAndSpeed);
/**
 * 配置寄存器 B (增益)
 * @param gain
 * @return
 */
uint8_t HMC_SetGain(uint8_t gain);
/**
 * 模式寄存器
 * @param mode
 * @return
 */
uint8_t HMC_SetMode(uint8_t mode);
/**
 * 计算角度
 * @param gauss1
 * @param gauss2
 * @return
 */
float HMC_ComputeAngle(float gauss1, float gauss2);

uint8_t HMC_Init(I2C_HandleTypeDef *hi2c) {
    gHmcHi2c = hi2c;

    uint8_t result = 0;

    result |= HMC_MpuInit();
    if (result) return result;
    result |= HMC_SetSampleAndSpeed(HMC_SAMPLING_8 | HMC_SPEED_15);
    if (result) return result;
    result |= HMC_SetGain(HMC_GAIN_1090);
    if (result) return result;
    result |= HMC_SetMode(HMC_MODE_COILED);
    if (result) return result;

    return 0;
}

uint8_t HMC_GetAngle(float *ax, float *ay, float *az) {
  uint8_t result = 0;
  uint8_t data[6] = {0};
  result |= HAL_I2C_Mem_Read(
      gHmcHi2c,
      HMC_ADDRESS << 1,
      HMC_MEM_ADDR_DATA_OUT,
      I2C_MEMADD_SIZE_8BIT,
      (uint8_t *) &data,
      6,
      0xFFFF
  );
  // The raw data is x/z/y
  short raw_x = data[0] << 8 | data[1];
  short raw_z = data[2] << 8 | data[3];
  short raw_y = data[4] << 8 | data[5];
  float gauss_x = raw_x / gHmcGain;
  float gauss_y = raw_y / gHmcGain;
  float gauss_z = raw_z / gHmcGain;
  *ax = HMC_ComputeAngle(gauss_z, gauss_y);
  *ay = HMC_ComputeAngle(gauss_z, gauss_x);
  *az = HMC_ComputeAngle(gauss_y, gauss_x);
  return result;
}

uint8_t HMC_MpuInit() {
    uint8_t result = 0;
#ifdef USE_MPU_6050
    uint8_t data = 0b10000000;
    result |= HAL_I2C_Mem_Write(gHmcHi2c, MPU6050_ADDRESS << 1, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &data, 1, 0xFFFF);
    HAL_Delay(100);
    data = 0b00000000;
    result |= HAL_I2C_Mem_Write(gHmcHi2c, MPU6050_ADDRESS << 1, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &data, 1, 0xFFFF);
    data = 0b00000000;
    result |= HAL_I2C_Mem_Write(gHmcHi2c, MPU6050_ADDRESS << 1, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT,  (uint8_t *) &data, 1, 0xFFFF);
    data = 0b00000010;
    result |= HAL_I2C_Mem_Write(gHmcHi2c, MPU6050_ADDRESS << 1, MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT,  (uint8_t *) &data, 1, 0xFFFF);
#endif
    return result;
}

uint8_t HMC_SetSampleAndSpeed(uint8_t sampleAndSpeed) {
    return HAL_I2C_Mem_Write(
        gHmcHi2c,
        HMC_ADDRESS << 1,
        HMC_MEM_ADDR_CONF_A,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &sampleAndSpeed,
        1,
        0xFFFF
    );
}

uint8_t HMC_SetGain(uint8_t gain) {
    switch (gain) {
        case HMC_GAIN_1370:gHmcGain = 1370.0F;break;
        case HMC_GAIN_1090:gHmcGain = 1090.0F;break;
        case HMC_GAIN_820:gHmcGain = 820.0F;break;
        case HMC_GAIN_660:gHmcGain = 660.0F;break;
        case HMC_GAIN_440:gHmcGain = 440.0F;break;
        case HMC_GAIN_390:gHmcGain = 390.0F;break;
        case HMC_GAIN_330:gHmcGain = 330.0F;break;
        case HMC_GAIN_230:gHmcGain = 230.0F;break;
        default:gHmcGain = 1090.0F;break;
    }
    return HAL_I2C_Mem_Write(
        gHmcHi2c,
        HMC_ADDRESS << 1,
        HMC_MEM_ADDR_CONF_B,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &gain,
        1,
        0xFFFF
    );
}

uint8_t HMC_SetMode(uint8_t mode) {
    return HAL_I2C_Mem_Write(
        gHmcHi2c,
        HMC_ADDRESS << 1,
        HMC_MEM_ADDR_MODE,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &mode,
        1,
        0xFFFF
    );
}

float HMC_ComputeAngle(float gauss1, float gauss2) {
  float angle = (float) atan2(gauss1, gauss2) * 180 / HMC_PI;
  if (angle < 0) angle += 360;
  if (angle > 360) angle -= 360;
  return angle;
}