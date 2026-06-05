/**
 * @file mpu6500.h
 * @brief MPU6500 6-axis IMU driver (SPI interface)
 * @author FPV Flight Control System
 * @date 2026
 * 
 * @description
 * MPU6500 is a 6-axis inertial measurement unit combining a 3-axis gyroscope
 * and a 3-axis accelerometer. This driver provides SPI communication interface
 * for ESP32-S3, supporting configurable measurement ranges and calibration.
 * 
 * @features
 * - SPI communication (HSPI)
 * - Configurable gyro range: ±250/500/1000/2000 °/s
 * - Configurable accel range: ±2/4/8/16 g
 * - Gyro and accel zero-offset calibration
 * - Temperature measurement
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>

#ifdef __cplusplus
#include <SPI.h>
extern "C" {
#endif

/* ============================================================================
 * Register Addresses
 * ============================================================================ */

/** @brief WHO_AM_I register, contains device ID */
#define MPU6500_WHO_AM_I            0x75

/** @brief Expected WHO_AM_I value for MPU6500 */
#define MPU6500_WHO_AM_I_VALUE      0x70

/** @brief Power management 1 register */
#define MPU6500_PWR_MGMT_1          0x6B

/** @brief Power management 2 register */
#define MPU6500_PWR_MGMT_2          0x6C

/** @brief Sample rate divider register */
#define MPU6500_SMPLRT_DIV          0x19

/** @brief Configuration register (DLPF) */
#define MPU6500_CONFIG              0x1A

/** @brief Gyroscope configuration register */
#define MPU6500_GYRO_CONFIG         0x1B

/** @brief Accelerometer configuration register */
#define MPU6500_ACCEL_CONFIG        0x1C

/** @brief Accelerometer configuration 2 register */
#define MPU6500_ACCEL_CONFIG2       0x1D

/** @brief User control register */
#define MPU6500_USER_CTRL           0x6A

/** @brief Interrupt pin configuration register */
#define MPU6500_INT_PIN_CFG         0x37

/** @brief Accelerometer X-axis output high byte */
#define MPU6500_ACCEL_XOUT_H        0x3B

/* ============================================================================
 * Enumerations
 * ============================================================================ */

/**
 * @brief Gyroscope full-scale range selection
 * 
 * Selectable ranges:
 * - 250DPS:  High precision, low dynamic range
 * - 500DPS:  Balanced precision and range
 * - 1000DPS: Medium precision, higher dynamic range
 * - 2000DPS: Low precision, maximum dynamic range (default for FPV)
 */
typedef enum {
    MPU6500_GYRO_250DPS  = 0x00,   /**< ±250 °/s */
    MPU6500_GYRO_500DPS  = 0x08,   /**< ±500 °/s */
    MPU6500_GYRO_1000DPS = 0x10,   /**< ±1000 °/s */
    MPU6500_GYRO_2000DPS = 0x18    /**< ±2000 °/s */
} MPU6500_GyroScale_t;

/**
 * @brief Accelerometer full-scale range selection
 * 
 * Selectable ranges:
 * - 2G:  High precision, suitable for slow motion
 * - 4G:  Balanced precision and range
 * - 8G:  Medium precision, suitable for moderate dynamics
 * - 16G: Low precision, maximum dynamic range (default for FPV)
 */
typedef enum {
    MPU6500_ACCEL_2G  = 0x00,      /**< ±2 g */
    MPU6500_ACCEL_4G  = 0x08,      /**< ±4 g */
    MPU6500_ACCEL_8G  = 0x10,      /**< ±8 g */
    MPU6500_ACCEL_16G = 0x18       /**< ±16 g */
} MPU6500_AccelScale_t;

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief MPU6500 device structure
 * 
 * Contains all sensor data, calibration parameters, and device state.
 * Users should allocate this structure and pass pointer to API functions.
 */
typedef struct {
    /* Processed sensor data */
    float accel_x, accel_y, accel_z;   /**< Acceleration in g */
    float gyro_x, gyro_y, gyro_z;       /**< Angular velocity in rad/s */
    float temperature;                   /**< Temperature in °C */
    
    /* Raw sensor data (16-bit signed integers) */
    int16_t accel_raw_x, accel_raw_y, accel_raw_z;  /**< Raw acceleration */
    int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;     /**< Raw angular velocity */
    int16_t temp_raw;                               /**< Raw temperature */
    
    /* Scale factors for unit conversion */
    float gyro_scale_factor;    /**< Gyro LSB to rad/s conversion factor */
    float accel_scale_factor;   /**< Accel LSB to g conversion factor */
    
    /* Calibration offsets */
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;   /**< Gyro zero-offset */
    float accel_offset_x, accel_offset_y, accel_offset_z; /**< Accel zero-offset */
    uint8_t calibrated;         /**< Calibration status flag (1=calibrated) */
    
    /* Device state */
    uint8_t inited;             /**< Initialization status flag (1=initialized) */
} MPU6500_t;

/* ============================================================================
 * API Functions
 * ============================================================================ */

/**
 * @brief Read all sensor data from MPU6500
 * 
 * Reads 14 bytes from ACCEL_XOUT_H register:
 * - Accelerometer X/Y/Z (6 bytes)
 * - Temperature (2 bytes)
 * - Gyroscope X/Y/Z (6 bytes)
 * 
 * Converts raw values to physical units using scale factors
 * and applies calibration offsets if calibrated.
 * 
 * @param imu Pointer to MPU6500_t structure
 * @return 1 on success, 0 on failure
 */
uint8_t MPU6500_ReadAll(MPU6500_t *imu);

/**
 * @brief Calibrate MPU6500 zero-offsets
 * 
 * Performs static calibration by averaging sensor readings
 * over specified number of samples. Device must be stationary.
 * 
 * Gyro offsets: Average angular velocity (should be near zero)
 * Accel offsets: Average acceleration (Z-axis offset adjusted for 1g)
 * 
 * @param imu Pointer to MPU6500_t structure
 * @param samples Number of samples to average (recommend 100-500)
 */
void MPU6500_Calibrate(MPU6500_t *imu, uint32_t samples);

#ifdef __cplusplus
}
#endif

/* C++ only functions (use SPIClass) */
#ifdef __cplusplus
/**
 * @brief Initialize MPU6500 via SPI
 * 
 * Performs the following:
 * - Resets device
 * - Wakes device from sleep mode
 * - Configures gyro range (±2000°/s)
 * - Configures accel range (±16g)
 * - Sets digital low-pass filter
 * 
 * @param imu Pointer to MPU6500_t structure
 * @param spi Pointer to SPIClass instance (must be initialized by caller)
 * @param cs_pin Chip select GPIO pin number
 * @return 1 on success, 0 on failure
 */
uint8_t MPU6500_Init(MPU6500_t *imu, SPIClass *spi, uint8_t cs_pin);
#endif

#endif /* MPU6500_H */
