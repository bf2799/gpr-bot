/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USR_BUTTON_Pin GPIO_PIN_13
#define USR_BUTTON_GPIO_Port GPIOC
#define IMU_SDA_Pin GPIO_PIN_0
#define IMU_SDA_GPIO_Port GPIOF
#define IMU_SCL_Pin GPIO_PIN_1
#define IMU_SCL_GPIO_Port GPIOF
#define SIG_GEN_REF_CLK_Pin GPIO_PIN_6
#define SIG_GEN_REF_CLK_GPIO_Port GPIOF
#define SIG_REC_REF_CLK_Pin GPIO_PIN_7
#define SIG_REC_REF_CLK_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SIG_REC_REF_DATA_Pin GPIO_PIN_3
#define SIG_REC_REF_DATA_GPIO_Port GPIOC
#define SIG_REC_ADC_Pin GPIO_PIN_0
#define SIG_REC_ADC_GPIO_Port GPIOA
#define VOLTAGE_MONITOR_IN_Pin GPIO_PIN_3
#define VOLTAGE_MONITOR_IN_GPIO_Port GPIOA
#define ENC_L_A_Pin GPIO_PIN_6
#define ENC_L_A_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define MOTOR_L_EN_Pin GPIO_PIN_7
#define MOTOR_L_EN_GPIO_Port GPIOE
#define MOTOR_L_ENB_Pin GPIO_PIN_8
#define MOTOR_L_ENB_GPIO_Port GPIOE
#define MOTOR_L_PWM1_Pin GPIO_PIN_9
#define MOTOR_L_PWM1_GPIO_Port GPIOE
#define MOTOR_R_EN_Pin GPIO_PIN_10
#define MOTOR_R_EN_GPIO_Port GPIOE
#define MOTOR_L_PWM2_Pin GPIO_PIN_11
#define MOTOR_L_PWM2_GPIO_Port GPIOE
#define MOTOR_R_ENB_Pin GPIO_PIN_12
#define MOTOR_R_ENB_GPIO_Port GPIOE
#define MOTOR_R_PWM1_Pin GPIO_PIN_13
#define MOTOR_R_PWM1_GPIO_Port GPIOE
#define MOTOR_R_PWM2_Pin GPIO_PIN_14
#define MOTOR_R_PWM2_GPIO_Port GPIOE
#define SIG_REC_REF_CLKB10_Pin GPIO_PIN_10
#define SIG_REC_REF_CLKB10_GPIO_Port GPIOB
#define SIG_REC_REF_LE_Pin GPIO_PIN_11
#define SIG_REC_REF_LE_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENC_R_A_Pin GPIO_PIN_12
#define ENC_R_A_GPIO_Port GPIOD
#define ENC_R_B_Pin GPIO_PIN_13
#define ENC_R_B_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define ENC_L_B_Pin GPIO_PIN_7
#define ENC_L_B_GPIO_Port GPIOC
#define SIG_GEN_REF_CLKC9_Pin GPIO_PIN_9
#define SIG_GEN_REF_CLKC9_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SIG_GEN_CLK_Pin GPIO_PIN_10
#define SIG_GEN_CLK_GPIO_Port GPIOC
#define SIG_GEN_DATA_Pin GPIO_PIN_12
#define SIG_GEN_DATA_GPIO_Port GPIOC
#define RADIO_RXI_Pin GPIO_PIN_0
#define RADIO_RXI_GPIO_Port GPIOD
#define RADIO_TXO_Pin GPIO_PIN_1
#define RADIO_TXO_GPIO_Port GPIOD
#define SIG_GEN_LE_Pin GPIO_PIN_2
#define SIG_GEN_LE_GPIO_Port GPIOD
#define GPS_TXO_Pin GPIO_PIN_5
#define GPS_TXO_GPIO_Port GPIOD
#define GPS_RXI_Pin GPIO_PIN_6
#define GPS_RXI_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
