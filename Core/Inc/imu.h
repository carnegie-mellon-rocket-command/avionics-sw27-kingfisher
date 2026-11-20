/* imu.h -- LSM6DSOX on I2C1 (PB8 SCL / PB9 SDA), using ST's C driver (lsm6dsox_reg.c/.h).
 *
 * Setup: copy lsm6dsox_reg.c -> Core/Src and lsm6dsox_reg.h -> Core/Inc
 * (from github.com/STMicroelectronics/lsm6dsox-pid).
 *
 * Units match last year's code: acceleration in ft/s^2, gyro in rad/s, temperature in deg C.
 */
#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdbool.h>

#define GRAVITY 32.174f   /* ft/s^2 */

typedef struct {
    float accel_x, accel_y, accel_z;   /* ft/s^2 (includes gravity, like last year) */
    float gyro_x, gyro_y, gyro_z;      /* rad/s */
    float temp_c;                      /* deg C, from the IMU */
    float accel_mag;                   /* sqrt(x^2+y^2+z^2), ft/s^2 -- what the Kalman filter used last year */
} imu_data_t;

/* Finds the device at 0x6A/0x6B, enables its internal SA0 pull-up so it stays
 * at 0x6B, then resets, configures and reads back every required setting. */
bool imu_init(I2C_HandleTypeDef *hi2c);

/* Reads the latest sample. Returns false on an I2C error. */
bool imu_read(imu_data_t *out);

#endif
