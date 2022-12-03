#include <math.h>
#include "mpu_dmp.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ  (100)
#define Q30             (1073741824.0f)

I2C_HandleTypeDef *gDmpHi2c;

static signed char gyro_orientation[9] = {
        -1, 0, 0,
        0,-1, 0,
        0, 0, 1
};

static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static int run_self_test(void);

uint8_t MPU_DMP_Init(I2C_HandleTypeDef *hi2c) {
    gDmpHi2c = hi2c;
    // 初始化
    uint8_t res = mpu_init(NULL);
    if (res) return 1;
    // 设置所需要的传感器
#if defined MPU9150 || defined MPU9250
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    if (res) return 2;
    // 设置FIFO
#if defined MPU9150 || defined MPU9250
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    if (res) return 3;
    // 设置采样率
    res = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if (res) return 4;
    // 加载dmp固件
    res = dmp_load_motion_driver_firmware();
    if (res) return 5;
    // 设置陀螺仪方向
    res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (res) return 6;
    // 设置dmp功能
    res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                             DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                             DMP_FEATURE_GYRO_CAL);
    if (res) return 7;
    // 设置DMP输出速率(最大不超过200Hz)
    res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if (res) return 8;
    // 自检
    res = run_self_test();
    if (res) return 9;
    // 使能DMP
    res = mpu_set_dmp_state(1);
    if (res) return 10;
    return 0;
}

uint8_t MPU_DMP_GetData(float *pitch, float *roll, float *yaw) {
    float q0, q1, q2, q3;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) return 1;

    /*
     * Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
     */
    // if (sensors & INV_XYZ_GYRO )
    // send_packet(PACKET_TYPE_GYRO, gyro);
    // if (sensors & INV_XYZ_ACCEL)
    // send_packet(PACKET_TYPE_ACCEL, accel);

    /*
     * Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
     */
    if (sensors & INV_WXYZ_QUAT) {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;    // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;    // roll
        *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;    //yaw
        return 0;
    }
    return 2;
}

uint8_t stm32_i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    return HAL_I2C_Mem_Write(
            gDmpHi2c,
            slave_addr << 1, // I2C 器件地址是 7 位的, 需要转换
            reg_addr,
            1,
            data,
            length,
            0xFFFF
    );
}

uint8_t stm32_i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    return HAL_I2C_Mem_Read(
            gDmpHi2c,
            slave_addr << 1, // I2C 器件地址是 7 位的, 需要转换
            reg_addr,
            1,
            data,
            length,
            0xFFFF
    );
}

uint8_t stm32_delay_ms(unsigned long num_ms) {
    HAL_Delay(num_ms);
    return 0;
}

uint8_t stm32_get_ms(unsigned long *count) {
    *count = HAL_GetTick();
    return 0;
}

static unsigned short inv_row_2_scale(const signed char *row) {
    unsigned short b;
    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

static int run_self_test(void) {
    int result;
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3 || result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
    // return result;
    return 0;
}