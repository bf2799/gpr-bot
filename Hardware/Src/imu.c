/*
 * imu.c
 */

#include "imu.h"
#include "string.h"

#define IMU_I2C_TIMEOUT_MS 100

// For callback read/write functions. Since a write/read will be called before the callbacks,
// this can always be set to the right value first.
static I2C_HandleTypeDef* hi2c_cur = NULL;

/**
 * @brief Callback for writing to IMU serially via I2C
 * @param[in] dev_addr: I2C device address
 * @param[in] reg_addr: IMU target register address
 * @param[in] reg_data: Data to write to address
 * @param[in] wr_len: Length of register to write (length of reg_data)
 */
static int8_t imu_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t wr_len) {
	// Set up transmit data
	uint8_t tx_data[wr_len + 1];
	tx_data[0] = reg_addr;
	memcpy(&tx_data[1], reg_data, wr_len);

	// Transmit tx data
	if (HAL_I2C_Master_Transmit(hi2c_cur, (dev_addr << 1), tx_data, wr_len + 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		return BNO055_ERROR;
	}
	return BNO055_SUCCESS;
}

/**
 * @brief Callback for reading from IMU serially via I2C
 * @param[in] dev_addr: I2C device address
 * @param[in] reg_addr: IMU target register address
 * @param[in] reg_data: Data to write to address
 * @param[in] r_len: Length of register to read (length of reg_data)
 */
static int8_t imu_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t r_len) {
	// Transmit register to read from
	if (HAL_I2C_Master_Transmit(hi2c_cur, (dev_addr << 1), &reg_addr, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		return BNO055_ERROR;
	}
	// Read register into reg_data
	if (HAL_I2C_Master_Receive(hi2c_cur, (dev_addr << 1) | 0x01, reg_data, r_len, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		return BNO055_ERROR;
	}
	return BNO055_SUCCESS;
}

static void imu_delay(u32 msec) {
	HAL_Delay(msec);
}

void imu_init(imu_t* dev, I2C_HandleTypeDef* hi2c) {
	// Initialize IMU parameters
	dev->hi2c = hi2c;

	// Initialize BNO device
	dev->bno_dev.dev_addr = BNO055_I2C_ADDR1;
	dev->bno_dev.bus_read = imu_i2c_read;
	dev->bno_dev.bus_write = imu_i2c_write;
	dev->bno_dev.delay_msec = imu_delay;
	hi2c_cur = dev->hi2c;
	bno055_init(&dev->bno_dev);

	// Set power mode, sensor reading mode, and correct units
	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
	bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
	bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
}

imu_data_t* imu_get_data(imu_t* dev) {
	// Set current I2C handle for callbacks
	hi2c_cur = dev->hi2c;

	// Get heading as quaternion
	struct bno055_euler_double_t temp_heading;
	if (bno055_convert_double_euler_hpr_rad(&temp_heading) != BNO055_SUCCESS) {
		return NULL;
	}
	dev->imu_data.heading = temp_heading;

	// Get linear acceleration
	struct bno055_linear_accel_double_t temp_linear_accel;
	if (bno055_convert_double_linear_accel_xyz_msq(&temp_linear_accel) != BNO055_SUCCESS) {
		return NULL;
	}
	dev->imu_data.linear_accel = temp_linear_accel;

	return &(dev->imu_data);
}


