/*
 * signal_generator.c
 */

#include "signal_generator.h"

#include <math.h>

/**
 * @brief Writes the given register based on the device struct values
 * @param[in] dev: Signal generator device
 * @param[in] register_num: Number of register to write (0-5)
 */
static void signal_generator_write_register(const signal_generator_t* dev, uint8_t register_num) {

	// Check user inputs
	if (!dev || register_num > 5) {
		return;
	}

	// Produce register value (convert from little endian to big endian for SPI send)
	uint8_t reg_vals[4] = {0};
	for (int i = 0; i < 4; i++) {
		reg_vals[i] = dev->regs[register_num].reg_vals[3 - i];
	}

	// SPI write to device
	HAL_GPIO_WritePin(dev->le_port, dev->le_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, reg_vals, 4, 100);
	HAL_GPIO_WritePin(dev->le_port, dev->le_pin, GPIO_PIN_SET);
}

void signal_generator_init(signal_generator_t* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* le_port, uint16_t le_pin, TIM_HandleTypeDef* htim, uint32_t tim_channel, double ref_clk_freq_mhz) {

	// Check user inputs
	if (!dev || !hspi || !le_port || !htim) {
		return;
	}

	// Set dev properties
	dev->hspi = hspi;
	dev->le_port = le_port;
	dev->le_pin = le_pin;
	dev->htim = htim;
	dev->tim_channel = tim_channel;
	dev->ref_clk_freq_mhz = ref_clk_freq_mhz;

	// Set register 0
	R0* reg0 = &dev->regs[0].reg0;
	reg0->RESERVED = 0;
	reg0->INT = 0;
	reg0->FRAC = 0;
	reg0->CTRL = 0;

	// Set register 1
	R1* reg1 = &dev->regs[1].reg1;
	reg1->RESERVED = 0;
	reg1->PRESCALAR = 0;
	reg1->PHASE = 1; // Recommended in datasheet
	reg1->MOD = 4095; // Max resolution
	reg1->CTRL = 1;

	// Set register 2
	R2* reg2 = &dev->regs[2].reg2;
	reg2->LNLS = 0b11; // Low-spur mode
	reg2->MUXOUT = 0b100; // N-divider output for debug
	reg2->RDOUBLER = dev->ref_clk_freq_mhz < 16 ? 1 : 0;
	reg2->RDIV2 = 0;
	reg2->RCOUNT = (uint16_t) ceil(dev->ref_clk_freq_mhz * (1 + reg2->RDOUBLER) / 32.0);
	reg2->DOUBLEBUFF = 0; // Disable, we have good enough control
	reg2->ICP = 0b1000; // Average current (2.81mA)
	reg2->LDF = 0; // Fractional lock detect
	reg2->LDP = 0; // 40 10-ns cycles for lock detect
	reg2->PDPOLARITY = 0; // Depends on loop filter
	reg2->POWERDOWN = 0; // Keep on
	reg2->CP3STATE = 0; // Normal operation
	reg2->COUNTRESET = 0; // Normal operation
	reg2->CTRL = 2;

	// Set register 3
	R3* reg3 = &dev->regs[3].reg3;
	reg3->RESERVED = 0;
	reg3->CSR = 0; // Disable cycle slip reduction
	reg3->RESERVED2 = 0;
	reg3->CLKDIVMODE = 0; // Phase clock off
	reg3->CLKDIVVAL = 1; // No division
	reg3->CTRL = 3;

	// Set register 4
	R4* reg4 = &dev->regs[4].reg4;
	reg4->RESERVED = 0;
	reg4->FBSELECT = 0; // Enable output dividers
	reg4->DIVSELECT = 0;
	reg4->BANDSELCLKDIVVAL = 1;
	reg4->VCOPOWERDOWN = 0; // Keep VCO powered
	reg4->MTLD = 0; // Disable mute till lock detect
	reg4->AUXOUTSEL = 0; // Auxilary output from dividers
	reg4->AUXOUTEN = 0; // Disable auxilary output
	reg4->AUXOUTPOW = 0b11; // If aux enabled, output power max
	reg4->RFOUT = 1; // Enable RF output
	reg4->POUT = 0b11; // RF output power +5dBm
	reg4->CTRL = 4;

	// Set register 5
	R5* reg5 = &dev->regs[5].reg5;
	reg5->RESERVED = 0;
	reg5->LDPIN = 0b01; // Digital lock detect as output
	reg5->RESERVED2 = 0;
	reg5->CTRL = 5;

	// Calculate pfd frequency
	// fpfd = REF_in * [(1 + D)/(R * (T + 1))]
	// D = doubler bit (0 - 1)
	// T = divide-by-2 bit (0 - 1)
	// R = R counter divisor (1 - 1023)
	dev->freq_pfd_mhz = ref_clk_freq_mhz * ((1 + reg2->RDOUBLER) / reg2->RCOUNT);

	// Write registers
	for (int i = 5; i >= 0; i--) {
		signal_generator_write_register(dev, (uint8_t) i);
	}
}

void signal_generator_set_output_freq(signal_generator_t* dev, double freq_mhz) {

	// Check user inputs
	if (!dev || freq_mhz < 137.5 || freq_mhz > 4400) {
		return;
	}

	// Choose DIVSELECT (final divider either 1, 2, 4, 8, 16). VCO output is only 2.2GHz - 4.4GHz
	// DIVSELECT = floor(log(4400/freq_mhz)/log(2))
	dev->regs[4].reg4.DIVSELECT = (uint8_t) floor(log(4400.0 / freq_mhz) / log(2.0));
	signal_generator_write_register(dev, 4);

	// Set PLL values to produce desired output frequency
	// Rf_out = fpfd * (INT + FRAC / MOD) / RF_div
	// INT = (23 - 65535)
	// MOD = (2 - 4095)
	// FRAC = (0 - [MOD - 1])
	// RF_div = DIVSELECT (1, 2, 4, 8, 16)
	double remainder = freq_mhz * (double)(1u << dev->regs[4].reg4.DIVSELECT) / dev->freq_pfd_mhz;
	double INT = floor(remainder);
	double FRAC = (remainder - INT) * (double) dev->regs[0].reg1.MOD;
	dev->regs[0].reg0.INT = (uint16_t) INT;
	dev->regs[0].reg0.FRAC = (uint16_t) FRAC;
	signal_generator_write_register(dev, 0);
}

void signal_generator_start(const signal_generator_t* dev) {
	if (!dev) {
		return;
	}

	// Turn on reference clock
	HAL_TIM_OC_Start(dev->htim, dev->tim_channel);
}

void signal_generator_stop(const signal_generator_t* dev) {
	if (!dev) {
		return;
	}

	// Turn off reference clock
	HAL_TIM_OC_Stop(dev->htim, dev->tim_channel);
}

