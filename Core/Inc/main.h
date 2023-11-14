/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define	FREQMODEL	1
#define SINGLEWIRE	0

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define JHANT	10
#define ADC_MAX_CHANNEL	2
#define ADC_DMA_LENGTH	50			//number of samples to read from ADC before interrupt
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
#define CAN_MINUS_Pin GPIO_PIN_13
#define CAN_MINUS_GPIO_Port GPIOC
#define CAN_PLUS_Pin GPIO_PIN_14
#define CAN_PLUS_GPIO_Port GPIOC
#define BATVOLT_Pin GPIO_PIN_0
#define BATVOLT_GPIO_Port GPIOA
#define BRK_UC_Pin GPIO_PIN_1
#define BRK_UC_GPIO_Port GPIOA
#define SPDANALOG_Pin GPIO_PIN_2
#define SPDANALOG_GPIO_Port GPIOA
#define SPDFREQ_Pin GPIO_PIN_3
#define SPDFREQ_GPIO_Port GPIOA
#define REV_Pin GPIO_PIN_4
#define REV_GPIO_Port GPIOA
#define G1_Pin GPIO_PIN_6
#define G1_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_7
#define G2_GPIO_Port GPIOA
#define KEY_ODO_Pin GPIO_PIN_0
#define KEY_ODO_GPIO_Port GPIOB
#define KEY_ODO_EXTI_IRQn EXTI0_1_IRQn
#define G3_Pin GPIO_PIN_1
#define G3_GPIO_Port GPIOB
#define BKLT_Pin GPIO_PIN_15
#define BKLT_GPIO_Port GPIOA
#define LED8_Pin GPIO_PIN_0
#define LED8_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_1
#define LED7_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOD
#define BUZZ_Pin GPIO_PIN_3
#define BUZZ_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_3
#define LED6_GPIO_Port GPIOB
#define SEG_DATA_Pin GPIO_PIN_4
#define SEG_DATA_GPIO_Port GPIOB
#define SEG_WR_Pin GPIO_PIN_5
#define SEG_WR_GPIO_Port GPIOB
#define SEG_RD_Pin GPIO_PIN_6
#define SEG_RD_GPIO_Port GPIOB
#define SEG_CS_Pin GPIO_PIN_7
#define SEG_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define  CMD_SYS_DIS  0x00  // SYS DIS    (0000-0000-X) Turn off system oscillator, LCD bias gen [Default]
#define  CMD_SYS_EN   0x01  // SYS EN     (0000-0001-X) Turn on  system oscillator
#define  CMD_LCD_OFF  0x02  // LCD OFF    (0000-0010-X) Turn off LCD display [Default]
#define  CMD_LCD_ON   0x03  // LCD ON     (0000-0011-X) Turn on  LCD display
#define  CMD_RC_INT   0x18  // RC INT     (0001-10XX-X) System clock source, on-chip RC oscillator
#define  CMD_BIAS_COM 0xA6  // BIAS & COM (0010-10X1-X) 1/3 bias, 4 commons // HT1621 only
#define  CMD_WDT_DIS	0x05
#define	 CMD_WDT_CLR	0x0F
#define	 CMD_TEST			0xE0

#define VERS	12

#define LOGO	1
#define GEAR	2
//#define CYCLE	3
#define ODO 	4
#define TRIP	5
#define BATERY	6
#define PERCENTAGE	7
#define POWER	8
#define VOLT	9
#define MM  	10
#define AMP 	11
#define BAT_FAULT	12
#define BAT_DEAD	13
#define ODO_DECIMAL	14
#define ONE	15
#define MILES	16
#define KM	17
#define MPH	18
#define KMPH	19
#define VDOT	20
#define	THROTEL	21
#define ECU		22
#define CONTROLLER	23
#define REVERSE	45
#define NUTRAL	46
#define ECHO	47
#define FAST	48
#define	CRUISE	49
#define	RNG	50
#define WARN	23
#define THROTLE	21
#define	BOOST	51
#define	bOOST	52
#define	HILL	53
#define MOTOR	54

//#define BASE_ADDR	0				//0x4700 for APROM
//#define TRIP_OFFSET	1000

//#define ADDR_TRIP_OFF	120
#define SHORT 1
#define LONG	2

#define ADDR_TOG	260				//241
#define ADDR_PTR	261				//242
#define ADDR_TRIP_OFF	128		//120
#define ADDR_LVAL	262
#define ADDR_VS	264
#define ADDR_G1	265
#define ADDR_G2	266
#define ADDR_G3	267
#define ADDR_G4	268
#define ADDR_GA1	269
#define ADDR_GA2	271
#define ADDR_GA3	272
#define ADDR_GA4	273
#define ADDR_DIV	274				//243
#define ADDR_RMUL	275
#define ADDR_RMUL1	276
#define ADDR_RMUL2	277
#define ADDR_RMUL3	278
#define ADDR_DIVDIS	280
#define ADDR_VERS	281
#define ADDR_FREQ	282
#define ADDR_DivFrq	283
#define	ADDR_BAUD	300
#define ADDR_SPDLH	307
#define ADDR_EMP1	308
#define ADDR_EMP2	309


#define ADRINX_TOG 	0
#define ADRINX_PTR	1
#define ADRINX_LVAL	2
#define ADRINX_VS	4
#define ADRINX_G1	5
#define ADRINX_G2	6
#define ADRINX_G3	7
#define ADRINX_G4	8
#define ADRINX_GA1	9
#define ADRINX_GA2	11
#define ADRINX_GA3	12
#define ADRINX_GA4	13
#define ADRINX_DIV	14

// define event flag bit variable
#define SPEED_BIT   (1UL << 0UL) 	// zero shift for bit0
#define ODO_BIT   	(1UL << 1UL) 	// 1 shift for flag bit 1
#define TRIP_BIT   	(1UL << 2UL) 	// 2 shift for flag bit 2
#define RANGE_BIT   (1UL << 3UL) 	// 3 shift for flag bit 3
#define GEAR_BIT   	(1UL << 4UL) 	// 4 shift for flag bit 4
#define VOLT_BIT   	(1UL << 5UL) 	// 5 shift for flag bit 5
#define PARK_BIT	(1UL << 6UL)	// 6 shift for flag bit 6
#define REV_BIT		(1UL << 7UL)	// 7 shift for flag bit 6
#define ECHO_BIT	(1UL << 8UL)	// 8
#define CURISE_BIT	(1UL << 9UL)	// 9
#define WARN_BIT	(1UL << 10UL)	// 10
#define THROTLE_BIT	(1UL << 11UL)	// 11
#define MOTOR_BIT	(1UL << 12UL)	// 12
#define SIDE_BIT	(1UL << 13UL)
#define BAT_DEAD_BIT	(1UL << 14UL)
#define ECU_BIT		(1UL << 15UL)
#define SOC_DISP_BIT1		(1UL << 16UL)
#define SOC_DISP_BIT2		(1UL << 17UL)
#define CAH_BIT		(1UL << 18UL)
#define AMP_BIT1	(1UL << 19UL)
#define VOLT_BIT1	(1UL << 20UL)
#define SOC_DISP_BIT3		(1UL << 21UL)
#define AMP_BIT2	(1UL << 23UL)
/*#define AMP_BIT3	(1UL << 23UL)
#define AMP_BIT4	(1UL << 24UL)
#define AMP_BIT5	(1UL << 25UL)
#define AMP_BIT6	(1UL << 26UL)
#define AMP_BIT7	(1UL << 27UL)
#define AMP_BIT8	(1UL << 28UL)
#define AMP_BIT9	(1UL << 29UL)*/

#define DBAUD	3

#define SOC1	1
#define SOC2	2
#define SOC3	6
#define SOC4	7
#define SOC5	8
#define VOLT1	4
#define VOLT2	9
#define VOLT3	10
#define AMP1	5
#define AMP2	11
#define AMP3	12
#define AMP4	13
#define AMP5	14
#define AMP6	15
#define AMP7	16
#define AMP8	17
#define AMP9	18
#define AMP10	19

void eepromWrite(uint16_t addr, uint8_t *dat, uint16_t siz);
void eepromRead(uint16_t addr, uint8_t *dat, uint16_t siz);
//void MX_FDCAN1_Init(void);
void FDCAN1_Config(void);
void FDCAN_SetBaud(unsigned char baud);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
