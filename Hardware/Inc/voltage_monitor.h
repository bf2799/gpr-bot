/*
 * voltage_monitor.h
 *
 * Driver for monitoring battery voltage via ADC
 */

#ifndef INC_VOLTAGE_MONITOR_H_
#define INC_VOLTAGE_MONITOR_H_

#include <stdbool.h>

#include "stm32f7xx_hal.h"

typedef struct voltage_monitor_t {
	ADC_HandleTypeDef* hadc;
	double volts_per_adc_inc;
	uint32_t adc_val;
} voltage_monitor_t;

/**
 * @brief Initialize voltage monitor
 * @param[in, out] dev: Voltage monitor device
 * @param[in] hadc: ADC handle to map to voltage monitor
 */
void voltage_monitor_init(voltage_monitor_t* dev, ADC_HandleTypeDef* hadc);

/**
 * @brief Start read from voltage monitor
 * @param[in] dev: Voltage monitor device
 *
 * This conversion takes time, so separating from getting the value increases flexibility
 */
void voltage_monitor_start_read(voltage_monitor_t* dev);

/**
 * @brief Gets voltage from the voltage monitor
 * @param[in] dev: Voltage monitor device
 * @param[out] voltage: Voltage that was received from the monitor
 */
bool voltage_monitor_get_voltage(voltage_monitor_t* dev, double* voltage);

#endif /* INC_VOLTAGE_MONITOR_H_ */
