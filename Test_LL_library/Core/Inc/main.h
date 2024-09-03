/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f1xx_it.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define ROWS 8
#define COLS 8

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern unsigned char dotMatrixBoard_yellow[ROWS][COLS];
extern unsigned char dotMatrixBoard_red[ROWS][COLS];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void led_blink_example_code(void);
void UART2_TX_string(char* str);
static void inline delay_us(unsigned int delay);
void write_serial_in_parallel_out_register_plus(unsigned char data);
void write_serial_in_parallel_out_register_minus_yellow(unsigned char data);
void write_serial_in_parallel_out_register_minus_red(unsigned char data);
unsigned char rowToBinary(unsigned char (*ptr)[8], int row);
void findCommonOnes(unsigned char(*board1)[COLS], unsigned char(*board2)[COLS], int rows, int cols);
void led_serial_in_parallel_out_example_1(void);
void heart_example_1(void);
void heart_example_2(void);
void heart_example_3(void);
void heart_example_4(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MR_3_Pin LL_GPIO_PIN_7
#define MR_3_GPIO_Port GPIOA
#define tact_switch_Pin LL_GPIO_PIN_4
#define tact_switch_GPIO_Port GPIOC
#define tact_switch_EXTI_IRQn EXTI4_IRQn
#define CP_2_Pin LL_GPIO_PIN_10
#define CP_2_GPIO_Port GPIOB
#define DSA_2_Pin LL_GPIO_PIN_14
#define DSA_2_GPIO_Port GPIOB
#define CP_3_Pin LL_GPIO_PIN_7
#define CP_3_GPIO_Port GPIOC
#define MR_2_Pin LL_GPIO_PIN_8
#define MR_2_GPIO_Port GPIOA
#define DSA_3_Pin LL_GPIO_PIN_9
#define DSA_3_GPIO_Port GPIOA
#define DSA_1_Pin LL_GPIO_PIN_10
#define DSA_1_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define CP_1_Pin LL_GPIO_PIN_4
#define CP_1_GPIO_Port GPIOB
#define MR_1_Pin LL_GPIO_PIN_6
#define MR_1_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
