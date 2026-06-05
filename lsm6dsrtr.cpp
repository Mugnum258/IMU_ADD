/**
 * @file lsm6dsrtr.cpp
 * @brief LSM6DSRTR 6-axis IMU driver implementation (I2C interface)
 */

#include <Arduino.h>
#include "lsm6dsrtr.h"
#include <Wire.h>
#include <math.h>

/* ============================================================================
 * Private Functions - I2C Register Access
 * ============================================================================ */

/**
 * @brief Write single register to LSM6DSRTR via I2C
 * 
 * I2C write format: [device_addr] [reg_addr] [data]
 * 
 * @param reg Register address
 * @param data Data to write
 */
static void LSM_WriteReg(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(LSM6DSRTR_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

/**
 * @brief Read single register from LSM6DSRTR via I2C
 * 
 * I2C read format: [device_addr] [reg_addr] [restart] [device_addr|RD] [data]
 * 
 * @param reg Register address
 * @return Register value
 */
static uint8_t LSM_ReadReg(uint8_t reg)
{
    Wire.beginTransmission(LSM6DSRTR_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);        // Send restart condition
    Wire.requestFrom((int)LSM6DSRTR_ADDR, 1);
    return Wire.read();
}

/**
 * @brief Read multiple consecutive registers from LSM6DSRTR via I2C
 * 
 * Uses burst read for efficiency. Auto-increments register address.
 * 
 * @param reg Starting register address
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void LSM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    Wire.beginTransmission(LSM6DSRTR_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);        // Send restart condition
    Wire.requestFrom((int)LSM6DSRTR_ADDR, (int)len);
    for (uint16_t i = 0; i < len; i++) buf[i] = Wire.read();
}

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize LSM6DSRTR via I2C
 * 
 * Configuration:
 * - CTRL1_XL (0x10): Accel ±8g, 416Hz ODR
 * - CTRL2_G (0x11): Gyro ±2000dps, 416Hz ODR
 * - CTRL3_C (0x12): Block update enabled, auto-increment
 * 
 * Scale factors:
 * - Accel: 8g / 32768 LSB
 * - Gyro: 2000°/s / 32768 LSB, converted to rad/s
 */
uint8_t LSM6DSRTR_Init(LSM6DSRTR_t *imu)
{
    if (!imu) return 0;
    
    /* Configure accelerometer: ±4g, 104Hz ODR */
    // CTRL1_XL: ODR_XL[7:4]=0100 (104Hz), FS_XL[3:2]=10 (±4g), LPF_XL_SEL=0
    LSM_WriteReg(0x10, 0x48);
    
    /* Configure gyroscope: ±2000°/s, 416Hz ODR */
    // CTRL2_G: ODR_G[7:4]=0110 (416Hz), FS_G[3:2]=10 (±2000dps)
    LSM_WriteReg(0x11, 0x4C);
    
    /* Configure control register 3 */
    // CTRL3_C: BDU=1 (block update), IF_INC=1 (auto-increment)
    LSM_WriteReg(0x12, 0x44);
    
    /* Calculate scale factors */
    // Accel: ±4g range, 16-bit signed output
    // g = LSB * (4/32768)
    imu->accel_scale = 4.0f / 32768.0f;
    
    // Gyro: ±2000°/s range, 16-bit signed output
    // rad/s = LSB * (2000/32768) * (pi/180)
    imu->gyro_scale = 2000.0f / 32768.0f * (3.14159265f / 180.0f);
    
    imu->inited = 1;
    return 1;
}

/**
 * @brief Read all sensor data from LSM6DSRTR
 * 
 * Burst reads 14 bytes starting from OUT_TEMP_L (0x20):
 * - Bytes 0-1:  Temperature (little-endian, 16-bit)
 * - Bytes 2-7:  Gyroscope X/Y/Z (little-endian, 16-bit each)
 * - Bytes 8-13: Accelerometer X/Y/Z (little-endian, 16-bit each)
 * 
 * Temperature conversion:
 * T(°C) = 25 + (TEMP_OUT / 256)
 * 
 * Data is converted to physical units using scale factors.
 */
uint8_t LSM6DSRTR_ReadAll(LSM6DSRTR_t *imu)
{
    if (!imu || !imu->inited) return 0;
    
    /* Burst read 14 bytes from OUT_TEMP_L register */
    uint8_t buf[14];
    LSM_ReadRegs(0x20, buf, 14);
    
    /* Parse raw data (little-endian, 16-bit signed) */
    int16_t t = (int16_t)(buf[1]<<8 | buf[0]);
    int16_t gx = (int16_t)(buf[3]<<8 | buf[2]);
    int16_t gy = (int16_t)(buf[5]<<8 | buf[4]);
    int16_t gz = (int16_t)(buf[7]<<8 | buf[6]);
    int16_t ax = (int16_t)(buf[9]<<8 | buf[8]);
    int16_t ay = (int16_t)(buf[11]<<8 | buf[10]);
    int16_t az = (int16_t)(buf[13]<<8 | buf[12]);
    
    /* Convert to physical units */
    // Temperature: offset 25°C, sensitivity 1/256 °C/LSB
    imu->temperature = 25.0f + (float)t / 256.0f;
    
    // Gyroscope: convert to rad/s
    imu->gyro_x = (float)gx * imu->gyro_scale;
    imu->gyro_y = (float)gy * imu->gyro_scale;
    imu->gyro_z = (float)gz * imu->gyro_scale;
    
    // Accelerometer: convert to g
    imu->accel_x = (float)ax * imu->accel_scale;
    imu->accel_y = (float)ay * imu->accel_scale;
    imu->accel_z = (float)az * imu->accel_scale;
    
    return 1;
}
