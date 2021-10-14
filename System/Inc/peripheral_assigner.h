/*
 * pin_assignment.h
 */

#ifndef INC_PERIPHERAL_ASSIGNER_H_
#define INC_PERIPHERAL_ASSIGNER_H_

#define ENCODER_LEFT_TIMER 					&htim3
#define ENCODER_RIGHT_TIMER 				&htim4
#define GPS_UART 							&huart2
#define IMU_I2C								&hi2c2
#define SIGNAL_GENERATOR_SPI				&hspi3
#define SIGNAL_GENERATOR_TIMER				&htim10
#define SIGNAL_GENERATOR_TIMER_CHANNEL		TIM_CHANNEL_1
#define SIGNAL_RECEIVER_ADC					&hadc1
#define SIGNAL_RECEIVER_REF_SPI				&hspi2
#define SIGNAL_RECEIVER_REF_TIMER			&htim11
#define SIGNAL_RECEIVER_REF_TIMER_CHANNEL	TIM_CHANNEL_1
#define GPR_MANAGER_TIMER					&htim7
#define RADIO_UART							&huart4

#include "adc.h"
#include "i2c.h"
#include "main.h" // For GPIO
#include "spi.h"
#include "tim.h"
#include "usart.h"

#endif /* INC_PERIPHERAL_ASSIGNER_H_ */
