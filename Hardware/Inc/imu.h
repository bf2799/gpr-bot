/*
 * imu.h
 * Product: BNO055
 * Interface: I2C
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "bno055.h"
#include "stm32f7xx_hal.h"

typedef struct imu_data_t {
	struct bno055_euler_t euler;
	struct bno055_quaternion_t q;
	struct bno055_linear_accel_t linear_accel;
} imu_data_t;

typedef struct imu_t {
	struct bno055_t bno_dev;
	I2C_HandleTypeDef* hi2c;
	imu_data_t imu_data;
} imu_t;

/**
 * @brief Initialize IMU
 * @param[out] dev: Initialized IMU device
 * @param[in] hi2c: I2C handle for IMU device
 */
void imu_init(imu_t* dev, I2C_HandleTypeDef* hi2c);

/**
 * @brief Read relevant IMU data
 * @param[in] dev: IMU device to read from
 * @return NULL if read operation failed, acquired IMU data otherwise
 */
imu_data_t* imu_get_data(imu_t* dev);

#endif /* INC_IMU_H_ */
