/**
 * @file mpu6500.cpp
 * @brief MPU6500 6-axis IMU driver implementation (SPI interface)
 */

#include <Arduino.h>
#include "mpu6500.h"
#include <SPI.h>
#include <math.h>

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/** @brief SPI instance pointer */
static SPIClass *mpu_spi = nullptr;

/** @brief Chip select pin */
static uint8_t mpu_cs_pin = 0;

/* ============================================================================
 * Private Functions - CS Control
 * ============================================================================ */

/** @brief Pull CS low to begin transaction */
static void MPU6500_CS_Low(void)  { digitalWrite(mpu_cs_pin, LOW); }

/** @brief Pull CS high to end transaction */
static void MPU6500_CS_High(void) { digitalWrite(mpu_cs_pin, HIGH); }

/* ============================================================================
 * Private Functions - Register Access
 * ============================================================================ */

/**
 * @brief Read single register from MPU6500
 * 
 * SPI read format: [reg_addr | 0x80] [dummy] [data]
 * 
 * @param reg Register address (0x00-0x75)
 * @return Register value
 */
static uint8_t MPU6500_ReadReg(uint8_t reg)
{
    mpu_spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MPU6500_CS_Low();
    mpu_spi->transfer(reg | 0x80);      // Set R/W bit to 1 for read
    uint8_t val = mpu_spi->transfer(0x00);
    MPU6500_CS_High();
    mpu_spi->endTransaction();
    return val;
}

/**
 * @brief Write single register to MPU6500
 * 
 * SPI write format: [reg_addr & 0x7F] [data]
 * 
 * @param reg Register address (0x00-0x75)
 * @param data Data to write
 */
static void MPU6500_WriteReg(uint8_t reg, uint8_t data)
{
    mpu_spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MPU6500_CS_Low();
    mpu_spi->transfer(reg & 0x7F);      // Set R/W bit to 0 for write
    mpu_spi->transfer(data);
    MPU6500_CS_High();
    mpu_spi->endTransaction();
}

/**
 * @brief Read multiple consecutive registers from MPU6500
 * 
 * Uses burst read for efficiency. Auto-increments register address.
 * 
 * @param reg Starting register address
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void MPU6500_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    mpu_spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MPU6500_CS_Low();
    mpu_spi->transfer(reg | 0x80);      // Set R/W bit to 1 for read
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = mpu_spi->transfer(0x00);
    }
    MPU6500_CS_High();
    mpu_spi->endTransaction();
}

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize MPU6500 via SPI
 * 
 * Initialization sequence:
 * 1. Store SPI and CS pin references
 * 2. Reset device (PWR_MGMT_1 = 0x80)
 * 3. Wake device, set clock source to gyro PLL (PWR_MGMT_1 = 0x01)
 * 4. Disable I2C interface (USER_CTRL = 0x10)
 * 5. Configure gyro range: ±2000°/s
 * 6. Configure accel range: ±16g
 * 7. Set DLPF bandwidth: 41Hz (CONFIG = 0x03)
 * 8. Set sample rate divider: 0 (1kHz output)
 */
uint8_t MPU6500_Init(MPU6500_t *imu, SPIClass *spi, uint8_t cs_pin)
{
    if (!imu || !spi) return 0;
    
    /* Store SPI and CS pin */
    mpu_spi = spi;
    mpu_cs_pin = cs_pin;
    
    /* Configure CS pin */
    pinMode(mpu_cs_pin, OUTPUT);
    MPU6500_CS_High();
    
    delay(50);  // Wait for power-up
    
    /* Reset device */
    MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x80);
    delay(100);  // Wait for reset
    
    /* Wake device, use gyro PLL as clock source */
    MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x01);
    delay(50);
    
    /* Disable I2C interface (SPI only mode) */
    MPU6500_WriteReg(MPU6500_USER_CTRL, 0x10);
    MPU6500_WriteReg(MPU6500_INT_PIN_CFG, 0x00);
    
    /* Configure sensor ranges */
    MPU6500_WriteReg(MPU6500_GYRO_CONFIG, MPU6500_GYRO_2000DPS);   // ±2000°/s
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_16G);     // ±16g
    
    /* Configure digital low-pass filter */
    MPU6500_WriteReg(MPU6500_CONFIG, 0x03);         // DLPF: 41Hz
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG2, 0x03);  // Accel DLPF: 41Hz
    
    /* Set sample rate */
    MPU6500_WriteReg(MPU6500_SMPLRT_DIV, 0);        // 1kHz sample rate
    
    /* Calculate scale factors */
    // Gyro: 2000°/s range, 16-bit signed output
    // rad/s = LSB * (2000/32768) * (pi/180)
    imu->gyro_scale_factor = 2000.0f / 32768.0f * (3.14159265f / 180.0f);
    
    // Accel: 16g range, 16-bit signed output
    // g = LSB * (16/32768)
    imu->accel_scale_factor = 16.0f / 32768.0f;
    
    /* Initialize calibration offsets to zero */
    imu->gyro_offset_x = imu->gyro_offset_y = imu->gyro_offset_z = 0;
    imu->accel_offset_x = imu->accel_offset_y = imu->accel_offset_z = 0;
    imu->calibrated = 0;
    
    imu->inited = 1;
    return 1;
}

/**
 * @brief Read all sensor data from MPU6500
 * 
 * Burst reads 14 bytes starting from ACCEL_XOUT_H (0x3B):
 * - Bytes 0-1:  Accelerometer X (high byte, low byte)
 * - Bytes 2-3:  Accelerometer Y
 * - Bytes 4-5:  Accelerometer Z
 * - Bytes 6-7:  Temperature
 * - Bytes 8-9:  Gyroscope X
 * - Bytes 10-11: Gyroscope Y
 * - Bytes 12-13: Gyroscope Z
 * 
 * Data is converted to physical units and calibration offsets applied.
 */
uint8_t MPU6500_ReadAll(MPU6500_t *imu)
{
    if (!imu || !imu->inited) return 0;
    
    /* Burst read 14 bytes from ACCEL_XOUT_H */
    uint8_t buf[14];
    MPU6500_ReadRegs(MPU6500_ACCEL_XOUT_H, buf, 14);
    
    /* Parse raw data (big-endian, 16-bit signed) */
    imu->accel_raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    imu->accel_raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    imu->accel_raw_z = (int16_t)((buf[4] << 8) | buf[5]);
    imu->temp_raw    = (int16_t)((buf[6] << 8) | buf[7]);
    imu->gyro_raw_x  = (int16_t)((buf[8] << 8) | buf[9]);
    imu->gyro_raw_y  = (int16_t)((buf[10] << 8) | buf[11]);
    imu->gyro_raw_z  = (int16_t)((buf[12] << 8) | buf[13]);
    
    /* Convert to physical units with calibration */
    imu->accel_x = (float)imu->accel_raw_x * imu->accel_scale_factor - imu->accel_offset_x;
    imu->accel_y = (float)imu->accel_raw_y * imu->accel_scale_factor - imu->accel_offset_y;
    imu->accel_z = (float)imu->accel_raw_z * imu->accel_scale_factor - imu->accel_offset_z;
    
    imu->gyro_x = (float)imu->gyro_raw_x * imu->gyro_scale_factor - imu->gyro_offset_x;
    imu->gyro_y = (float)imu->gyro_raw_y * imu->gyro_scale_factor - imu->gyro_offset_y;
    imu->gyro_z = (float)imu->gyro_raw_z * imu->gyro_scale_factor - imu->gyro_offset_z;
    
    /* Temperature conversion formula from datasheet */
    // T(°C) = (TEMP_OUT / 333.87) + 21.0
    imu->temperature = ((float)imu->temp_raw / 333.87f) + 21.0f;
    
    return 1;
}

/**
 * @brief Calibrate MPU6500 zero-offsets
 * 
 * Performs static calibration by averaging sensor readings.
 * Device MUST be stationary during calibration.
 * 
 * Gyro offsets: Average of angular velocity readings
 * Accel offsets: Average of acceleration readings, Z-axis adjusted for 1g gravity
 * 
 * @param imu Pointer to MPU6500_t structure
 * @param samples Number of samples to average (recommend 100-500)
 */
void MPU6500_Calibrate(MPU6500_t *imu, uint32_t samples)
{
    if (!imu || !imu->inited) return;
    
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    
    /* Accumulate sensor readings */
    for (uint32_t i = 0; i < samples; i++) {
        MPU6500_ReadAll(imu);
        gx += imu->gyro_x; gy += imu->gyro_y; gz += imu->gyro_z;
        ax += imu->accel_x; ay += imu->accel_y; az += imu->accel_z;
        delay(5);  // 5ms interval between readings
    }
    
    /* Calculate average offsets */
    imu->gyro_offset_x = gx / samples;
    imu->gyro_offset_y = gy / samples;
    imu->gyro_offset_z = gz / samples;
    
    imu->accel_offset_x = ax / samples;
    imu->accel_offset_y = ay / samples;
    // Z-axis: subtract 1g to account for gravity (assuming device is level)
    imu->accel_offset_z = (az / samples) - 1.0f;
    
    imu->calibrated = 1;
}
