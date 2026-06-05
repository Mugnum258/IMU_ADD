/**
 * @file lsm6dsrtr.h
 * @brief LSM6DSRTR 6-axis IMU driver (I2C interface)
 * @author FPV Flight Control System
 * @date 2026
 * 
 * @description
 * LSM6DSRTR is a high-performance 6-axis inertial measurement unit from STMicroelectronics.
 * It features low noise density (3.6 mdps/rtHz for gyro) and high ODR (up to 6.66 kHz),
 * making it suitable as a backup IMU for FPV flight control.
 * 
 * @features
 * - I2C communication (address: 0x6A when SA0=GND)
 * - Gyro range: ±2000 °/s
 * - Accel range: ±8 g
 * - Temperature measurement
 * - High output data rate (up to 6.66 kHz)
 * 
 * @note
 * CS pin must be pulled HIGH (or connected to VDD) for I2C mode.
 * If CS is floating or LOW, the device enters SPI mode.
 */

#ifndef LSM6DSRTR_H
#define LSM6DSRTR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Device Constants
 * ============================================================================ */

/** @brief I2C address when SA0 pin is connected to GND */
#define LSM6DSRTR_ADDR          0x6A

/** @brief WHO_AM_I register address */
#define LSM6DSRTR_WHO_AM_I      0x0F

/** @brief Expected WHO_AM_I value for LSM6DSRTR */
#define LSM6DSRTR_WHO_AM_I_VAL  0x6B

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief LSM6DSRTR device structure
 * 
 * Contains all sensor data, scale factors, and device state.
 * Users should allocate this structure and pass pointer to API functions.
 */
typedef struct {
    /* Processed sensor data */
    float accel_x, accel_y, accel_z;   /**< Acceleration in g */
    float gyro_x, gyro_y, gyro_z;       /**< Angular velocity in rad/s */
    float temperature;                   /**< Temperature in °C */
    
    /* Scale factors for unit conversion */
    float accel_scale;    /**< Accel LSB to g conversion factor */
    float gyro_scale;     /**< Gyro LSB to rad/s conversion factor */
    
    /* Device state */
    uint8_t inited;       /**< Initialization status flag (1=initialized) */
} LSM6DSRTR_t;

/* ============================================================================
 * API Functions
 * ============================================================================ */

/**
 * @brief Initialize LSM6DSRTR via I2C
 * 
 * Performs the following:
 * - Configures accelerometer: ±8g, 416Hz ODR
 * - Configures gyroscope: ±2000°/s, 416Hz ODR
 * 
 * @param imu Pointer to LSM6DSRTR_t structure
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DSRTR_Init(LSM6DSRTR_t *imu);

/**
 * @brief Read all sensor data from LSM6DSRTR
 * 
 * Reads 14 bytes from OUT_TEMP_L register (0x20):
 * - Temperature (2 bytes)
 * - Gyroscope X/Y/Z (6 bytes)
 * - Accelerometer X/Y/Z (6 bytes)
 * 
 * Converts raw values to physical units using scale factors.
 * 
 * @param imu Pointer to LSM6DSRTR_t structure
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DSRTR_ReadAll(LSM6DSRTR_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSRTR_H */
