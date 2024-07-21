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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 *	@desc Calculates the steps required given an x mm distance. 40 steps required to travel 1 mm
 *	@desc Does 400 iterations, of alternating high and low pulse i.e. 200 steps for one revolution
 */
int distCal(int distance);//in mm
/**
 *	@desc blocking microsecond delay
 */
void microDelay (uint16_t delay);
/**
 *	@desc Does the printing routine. GO FRONT, then GO RETRACT/GO BACK.
 */
void iniDelay(void);
/**
 *	@desc Checks for start response byte to commence checksum routine
 */
uint8_t* tpixCheckStartCmd(uint8_t* spiRxBuffer);
/**
 *	@desc Retrieves the details of blocks
 */
void retrieveGetBlocks(uint8_t* spiRxBuffer);
/**
 *	@desc Retrieves the coordinates of blocks detected
 */
void retrieveGetBlocks(uint8_t* spiRxBuffer);

/**
 *	@desc Will only be executed if conveyer stepper stops and upon detection of pixy colour blocks
 */
void printHeadMovementRoutine_Manual();

void printHeadMovementRoutine_AUTO();

int createRows();

void xAxisMovement(int z);



void swap(uint16_t *xp, uint16_t *yp);
// Function to perform Bubble Sort
void bubbleSort(uint16_t arr[], int n);

void conveyerMovement(int z);

// Limit switch
void backToHomePosition(void);

void pressEveBot(void);

void testCaseHomeMovement(int z);

void testCasePressEveBot();

void testCaseConveyerMovement();

void normalOperationTestCase();

void conveyerMovePastry();

char Keypad_Scan(void);

void keyPadScanRoutine();

void testCaseZAxis(int z);

void moveZAxisUp();

void moveZAxisDown();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LimitStart_Pin GPIO_PIN_0
#define LimitStart_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Orange_Keypad_Pin GPIO_PIN_1
#define Orange_Keypad_GPIO_Port GPIOB
#define Yellow_Keypad_Pin GPIO_PIN_2
#define Yellow_Keypad_GPIO_Port GPIOB
#define Black_Keypad_Pin GPIO_PIN_11
#define Black_Keypad_GPIO_Port GPIOB
#define STEP_Z_AXIS_Pin GPIO_PIN_8
#define STEP_Z_AXIS_GPIO_Port GPIOC
#define DIR_Z_AXIS_Pin GPIO_PIN_9
#define DIR_Z_AXIS_GPIO_Port GPIOC
#define EvebotButton_Pin GPIO_PIN_9
#define EvebotButton_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Green_Keypad_Pin GPIO_PIN_4
#define Green_Keypad_GPIO_Port GPIOB
#define Blue_Keypad_Pin GPIO_PIN_5
#define Blue_Keypad_GPIO_Port GPIOB
#define Purple_Keypad_Pin GPIO_PIN_7
#define Purple_Keypad_GPIO_Port GPIOB
#define Grey_Keypad_Pin GPIO_PIN_8
#define Grey_Keypad_GPIO_Port GPIOB
#define White_Keypad_Pin GPIO_PIN_9
#define White_Keypad_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
