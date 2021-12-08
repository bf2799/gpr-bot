/*
 * voltage_monitor.c
 */

#include "voltage_monitor.h"

#include "stm32f7xx_hal.h"

#define MAX_MEASURABLE_VOLTAGE 14.0
#define R1 3260
#define R2 10960

static void voltage_monitor_conv_complete(ADC_HandleTypeDef* hadc) {
	hadc->ConvCpltCallback = NULL;
}

void voltage_monitor_init(voltage_monitor_t* dev, ADC_HandleTypeDef* hadc) {
	// Check user inputts
	if (!dev || !hadc) {
		return;
	}

	dev->hadc = hadc;

	// Figure out max raw value of ADC
	int max_raw_val = 0;
	switch(hadc->Init.Resolution) {
		case ADC_RESOLUTION_12B:
			max_raw_val = 4096; // 2 << 12
			break;
		case ADC_RESOLUTION_10B:
			max_raw_val = 1024; // 2 << 10
			break;
		case ADC_RESOLUTION_8B:
			max_raw_val = 256; // 2 << 8
			break;
		default:
			break;
	}


	dev->volts_per_adc_inc = MAX_MEASURABLE_VOLTAGE / max_raw_val;
}

void voltage_monitor_start_read(voltage_monitor_t* dev) {
	// Check user inputs
	if (!dev) {
		return;
	}

	// Check conversion isn't currently in progress
	if (dev->hadc->ConvCpltCallback != NULL) {
		return;
	}

	HAL_ADC_RegisterCallback(dev->hadc, HAL_ADC_CONVERSION_COMPLETE_CB_ID, voltage_monitor_conv_complete);
	HAL_ADC_Start_DMA(dev->hadc, &dev->adc_val, 1);
}

bool voltage_monitor_get_voltage(voltage_monitor_t* dev, double* voltage) {
	// Check user inputs
	if (!dev) {
		return false;
	}
	// If callback exists, it hasn't been called yet
	if (dev->hadc->ConvCpltCallback) {
		return false;
	}

	*voltage = dev->adc_val * dev->volts_per_adc_inc * (R1 + R2) / R1;
	return true;
}
