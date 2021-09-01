/*
 * signal_generator.h
 *
 * Driver for the ground-penetrating radar's signal generator (VCO & PLL)
 * Part: ADF4350
 * Interface: 1-way SPI (max 33MHz), Load Enable
 */

#ifndef INC_SIGNAL_GENERATOR_H_
#define INC_SIGNAL_GENERATOR_H_

#include <stdbool.h>
#include "stm32f7xx_hal.h"

typedef struct __attribute__((__packed__)) R0 {
	uint8_t RESERVED : 1;
	uint16_t INT : 16;
	uint16_t FRAC : 12;
	uint8_t CTRL : 3;
} R0;

typedef struct __attribute__((__packed__)) R1 {
	uint8_t RESERVED : 4;
	uint8_t PRESCALAR : 1;
	uint16_t PHASE : 12;
	uint16_t MOD : 12;
	uint8_t CTRL : 3;
} R1;

typedef struct __attribute__((__packed__)) R2 {
	uint8_t RESERVED : 1;
	uint8_t LNLS : 2;  // Low-noise and low-spur modes
	uint8_t MUXOUT : 3;
	uint8_t RDOUBLER : 1; // Reference doubler
	uint8_t RDIV2 : 1; // Reference divide-by-2
	uint16_t RCOUNT : 10; // R counter
	uint8_t DOUBLEBUFF : 1; // Double buffer
	uint8_t ICP : 4; // Charge pump current setting
	uint8_t LDF : 1;
	uint8_t LDP : 1;
	uint8_t PDPOLARITY : 1;
	uint8_t POWERDOWN : 1;
	uint8_t CP3STATE : 1;
	uint8_t COUNTRESET : 1;
	uint8_t CTRL : 3;
} R2;

typedef struct __attribute__((__packed__)) R3 {
	uint16_t RESERVED : 13;
	uint8_t CSR : 1; // Cycle slip reduction
	uint8_t RESERVED2 : 1;
	uint16_t CLKDIVMODE : 2; // Clock divider mode
	uint16_t CLKDIVVAL : 12; // Clock divider value
	uint8_t CTRL : 3;
} R3;

typedef struct __attribute__((__packed__)) R4 {
	uint8_t RESERVED : 8;
	uint8_t FBSELECT : 1; // Feedback select
	uint8_t DIVSELECT : 3; // Divider select
	uint16_t BANDSELCLKDIVVAL : 8; // Band select clock divider value
	uint8_t VCOPOWERDOWN : 1;
	uint8_t MTLD : 1; // Mute till lock detect
	uint8_t AUXOUTSEL : 1; // Aux output select
	uint8_t AUXOUTEN : 1; // Aux output enable
	uint8_t AUXOUTPOW : 2; // Aux output power
	uint8_t RFOUT : 1; // RF output enable
	uint8_t POUT: 2; // Output power
	uint8_t CTRL : 3;
} R4;

typedef struct __attribute__((__packed__)) R5 {
	uint8_t RESERVED : 8;
	uint8_t LDPIN : 2; // Lock detect pin operation
	uint32_t RESERVED2 : 19;
	uint8_t CTRL : 3;
} R5;

typedef union register_t {
	R0 reg0;
	R1 reg1;
	R2 reg2;
	R3 reg3;
	R4 reg4;
	R5 reg5;
	uint32_t reg_vals[4];
} register_t;

typedef struct signal_generator_t {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* le_port;
	uint16_t le_pin;
	TIM_HandleTypeDef* htim;
	uint32_t tim_channel;
	GPIO_TypeDef* on_port;
	uint16_t on_pin;
	double ref_clk_freq_mhz;
	bool is_init;
	register_t regs[6];
	double freq_pfd_mhz;
} signal_generator_t;

/**
 * @brief Initialize signal generator. Must be called before any other functions.
 * @param[out] dev: Signal generator device to hold properties
 * @param[in] hspi: SPI handle for data
 * @param[in] le_port: Latch enable GPIO port
 * @param[in] le_pin: Latch enable GPIO pin
 * @param[in] htim: Timer handle for reference clock
 * @param[in] tim_channel: Timer channel for reference clock
 * @param[in] on_port: Transmitter switch GPIO port
 * @param[in] on_pin: Transmitter switch GPIO pin
 * @param[in] ref_clk_freq_mhz: Frequency of the reference clock in MHz
 */
void signal_generator_init(signal_generator_t* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* le_port, uint16_t le_pin, TIM_HandleTypeDef* htim, uint32_t tim_channel, GPIO_TypeDef* on_port, uint16_t on_pin, double ref_clk_freq_mhz);

/**
 * @brief Set the output frequency of the signal generator (137.5MHz - 4.4GHz)
 * @param[in, out] dev: Signal generator device
 * @param[in] freq_mhz: Frequency of output signal, at max resolution
 */
void signal_generator_set_output_freq(signal_generator_t* dev, double freq_mhz);

/**
 * @brief Start signal generation with current settings
 * @param[in] dev: Signal generator device
 */
void signal_generator_start(const signal_generator_t* dev);

/**
 * @brief Stops signal generation
 * @param[in] dev: Signal generator device
 */
void signal_generator_stop(const signal_generator_t* dev);

#endif /* INC_SIGNAL_GENERATOR_H_ */
