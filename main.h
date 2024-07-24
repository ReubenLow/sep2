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
/*
 * @desc
 * Converts the distance in mm to steps required for stepper to take
 * The "steps" are looping values to be iterated UNTIL.
 * @param distance in mm
 */
int distCal(int distance);//in mm
/*
 * @desc
 * Generates microsecond delay that otherwise not possible with HAL_Delay()
 * @param delay
 *
 */
void microDelay (uint16_t delay);
/*
 * @desc
 * Function is called after evebot button is pressed
 * The function is called to cover the distance gap that is marked on the evebot ruler before
 * the start of distance measurement at START on the ruler.
 *
 * The Printhead moves by 5mm forward +ve X axis, before retracting 5mm backwards -ve X axis.
 * Once back to the beginning of the printing position, movement() is called, with the PRINTIZE as the argument.
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
/*
 *	@desc
 *	Manual Mode
 *	ROUTINE:
 *	1. (conveyerMovePastry) The conveyer moves the centre point of the lowest object (used as a row reference) to the PRINTLINE
 *
 *	2.  First datum point ??
 *		For each object within the row along x axis
 *		Travel to the (centrepoint - PRINTSIZE/2), i.e. printing point
 *		Press the evebot button (a human does this)
 *		move 5mm x2 back and forth to cover the gap where printing does not happen
 *		move according to PRINTSIZE to print
 *		Press the evebot button (a human does this)
 *		Set the new datum point to be after the (centrepoint+printlength/2) of the previous object
 *		Repeat
 *
 */
void printHeadMovementRoutine_Manual();
/*
 *	@desc
 *	Auto Mode
 *	ROUTINE:
 *	1. (conveyerMovePastry) The conveyer moves the centre point of the lowest object (used as a row reference) to the PRINTLINE
 *
 *	2.  First datum point ??
 *		For each object within the row along x axis
 *		Travel to the (centrepoint - PRINTSIZE/2), i.e. printing point
 *		Press the evebot button (Servo does this)
 *		move 5mm x2 back and forth to cover the gap where printing does not happen
 *		move according to PRINTSIZE to print
 *		Press the evebot button (Servo does this)
 *		Set the new datum point to be after the (centrepoint+printlength/2) of the previous object
 *		Repeat
 *
 */
void printHeadMovementRoutine_AUTO();
/*
 * @desc
 *	1. Find a reference point for a row object i.e. lowest centre point or lowest object
 *  2. Add all the pastries that falls within the buffer range of the reference point. i.e. y axis range from centrepoint
 *
 *  The new row will contain all the x_coords of pastries to be printed.
 */
int createRows();
/*
 * @desc
 *	Moves the printhead according to the int z (distance in mm specified)
 * @params
 * int z is the distance in mm
 */
void xAxisMovement(int z);
/*
 * @desc
 *	Helper Swap function for bubblesort
 */
void swap(uint16_t *xp, uint16_t *yp);
/*
 * @desc
 *	Sorts the objects in the PastryData[] array according to distance closest from
 *	the left (datum) to the rightmost along the x axis
 */
void bubbleSort(uint16_t arr[], int n);
/*
 * @desc
 * Sends pulses every few microsends to the conveyer stepper motor to turn
 */
void conveyerMovement(int z);
/*
 * @desc
 *	Moves the printhead back towards the leftmost/HOME position
 *	until the printhead hits the limit switch
 *
 */
void backToHomePosition(void);
/*
 * @desc
 	 Uses TIM3 to generate PWM and turn the servo for EveBot button pressing
 *
 */
void pressEveBot(void);
/*
 * @desc
 *	Test case for x axis home movement
 * @params
 * distance (in mm) to test. Max is 250mm
 */
void testCaseHomeMovement(int z);
/*
 * @desc
 *	Test case for pressing of evebot button (Whether successful button presses is achieved)
 */
void testCasePressEveBot();
/*
 * @desc
 *	Test case for conveyer movement to move 14 cm forward (towards the printhead)
 */
void testCaseConveyerMovement();
/*
 *
 * @desc
 * The Function "snapsshots" - essentially it is retrieving the positions of the
 * pastries on the conveyer belt before the movement and printing routine starts.
 */
void normalOperationTestCase();
/*
 * @desc
 *	Moves the lowest centrepoint object among all objects along the row towards the printline
 *
 */
void conveyerMovePastry();
/*
 * @desc
 * @return
 * Returns the key pressed among the keys found on the keypad
 *
 */
char Keypad_Scan(void);
/*
 * @desc
 * Main control program - Dictates whether the program is
 * 1. adjusting the print size,
 * 2. adjusting the height of current print head i.e. z axis
 * 3. main program execution
 *
 */
void keyPadScanRoutine();
/*
 * @desc
 *	Test case z axis based on mm height that programmer puts
 * @param
 * int z in mm height
 */
void testCaseZAxis(int z);
/*
 * @desc
 *	Function will always be called in the main loop whenever the programState is set to
 *	1. adjustHeightUpwards
 *
 *	That assumes that the user presses once, this function runs until the user presses the button once again
 *
 */
void moveZAxisUp();
/*
 * @desc
 *	Function will always be called in the main loop whenever the programState is set to
 *	1. adjustHeightDownwards
 *
 *
 *That assumes that the user presses once, this function runs until the user presses the button once again
 */
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
