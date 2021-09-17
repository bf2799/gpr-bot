/*
 * pin_assignment.h
 */

#ifndef INC_PERIPHERAL_ASSIGNER_H_
#define INC_PERIPHERAL_ASSIGNER_H_

#define ENCODER_LEFT_TIMER 		&htim3
#define ENCODER_RIGHT_TIMER 	&htim4
#define GPS_UART 				&huart2
#define IMU_I2C					&hi2c2

#include "i2c.h"
#include "tim.h"
#include "usart.h"

#endif /* INC_PERIPHERAL_ASSIGNER_H_ */
