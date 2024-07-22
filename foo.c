/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Keypad
#define BLINK_DELAY 500 // Delay in milliseconds
#define DEBOUNCE_DELAY 200 // Debounce delay in milliseconds

//printhead
#define PRINTHEAD_DIR_PIN GPIO_PIN_1
#define PRINTHEAD_DIR_PORT GPIOA
#define PRINTHEAD_STEP_PIN GPIO_PIN_0
#define PRINTHEAD_STEP_PORT GPIOA

// Pins for conveyor belt stepper motor, Direction pin PB10. Step pin PA8
#define STEP_PIN_CONVEYOR GPIO_PIN_8
#define STEP_PIN_Z_AXIS GPIO_PIN_2
#define DIR_PIN_Z_AXIS GPIO_PIN_12
#define DIR_PIN_CONVEYOR GPIO_PIN_10
#define STEP_PORT_CONVEYOR GPIOA
#define DIR_PORT_CONVEYOR GPIOB



#define INIDIST 10

/**
 * Lines on pixy camera
 */
#define PRINT_LINE 200 // previous value 135
#define EVENT_HORIZON PRINT_LINE - 10 // 133

#define PIXY_RATIO 1.5
#define DATUM 20
int PRINTSIZE = 43;

#define RETRACT_PEN_UP 200
#define EXTEND_PEN_DOWN 336

//#define ROW_LINE 80 | NOT USED
//#define EXCESS_LINE 30 | NOT USED
//#define CONVEYER_MOVEMENT_MULTIPLER 0.74 | NOT USED

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum __pixyChecksumstate{
	PIXY_RESULT_OK = 1,
	PIXY_RESULT_ERROR = 0
}PixyChecksumState;



typedef struct __pastryData{
	//centrepoint of pastry coordinates
	uint16_t coordX; //8-9  response bits
	uint16_t coordY; //10-11 response bits
	//dimensions
	uint16_t height;
	uint16_t width;
	//bottom line
	uint16_t bottomY;
	//top line
	uint16_t topY;
} PastryData;


typedef struct __row{
	uint16_t coordRow_Y;
	uint16_t numberOfRowObjects;
	uint16_t coordXArray[10];
}Row;


typedef enum{
  TOGGLED_OFF = 0,
  TOGGLED_ON = 1,
  UNKNOWN = 2
}ComponentState;

typedef struct{
  ComponentState state;
}LimitSwitch;

typedef enum __programState{
	programON = 0,
	programOFF = 1,
	adjustHeightUpwards = 2,
	adjustHeightDownwards = 3
}ProgramState;

typedef enum __heightAdjustmentStatus{
	upwards_set = 0,
	no_set = 1,
	downwards_set = 2
}HeightAdjustmentState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_SIZE 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
uint8_t* responsePacketStart;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t spiRxBuffer[SPI_BUFFER_SIZE];
uint8_t spiTxBuffer[SPI_BUFFER_SIZE];

uint8_t getVersion[] = {
  0xae,
  0xc1,
  0x0e,
  0x00,
};

uint8_t getBlocks[] = {
  0xae, //175
  0xc1, //193
  0x20, //type of packet - getBlocks Request 32 dec 0x20 Hex
  0x02, // payload (below length) 0x02
  0xFF, // get all signatures recognised
  0x06 // number of blocks x4
};

uint8_t setLamp[] = {
  0xae, //175
  0xc1, //193
  0x16,
  0x02,
  0x01
};

char inputBuffer[10];
int bufferIndex = 0;
volatile bool listeningForBreadWidth = false;
volatile bool listeningForAxisHeight = false;
volatile int breadWidth = 0;
volatile int axisHeight = 0;
volatile char lastKeyPressed = 'L'; // Track the last key pressed for debouncing
int stepDelay = 700; // SPEED, 1000us more delay means less speed


/**
 *	@desc For debugging: Tracks the checksum of the incoming response packet.
 */
PixyChecksumState checkSumStatus;

/**
 *	@desc Variables related to key pad and key press routine
 */

volatile int rowKeypadMatrix;
volatile int columnKeypadMatrix;
volatile char theKey;
uint32_t previousTick = 0;
uint32_t lastDebounceTime = 0; // Last debounce time
volatile char global;
HeightAdjustmentState adjustmentState = no_set;
ProgramState programState = programOFF;

/**
 *	@desc Non blocking numpad
 */
typedef enum {
    SCAN_STATE_IDLE,
    SCAN_STATE_SCAN_COL,
    SCAN_STATE_WAIT_RELEASE
} KeypadScanState;

volatile KeypadScanState scanState = SCAN_STATE_IDLE;
volatile int currentCol = 0;
volatile char detectedKey = 'L';

/**
 *	@desc Number of blocks/pastry detected by PixyCamera
 */
uint8_t numberOfBlocks;
uint16_t rowval = 0;



LimitSwitch startSwitch = {
  .state = TOGGLED_OFF
};
Row row; // number of rows

PastryData pastry[6]; //max no of objects

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/**
 *	@desc Determine if the data inside the uint8_t* array is reliable to use
 *	@params uint8_t array, which is the SPI buffer used during SPI transmission protocol
 *	@returns PIXY_RESULT_OK, indicating array is RELIABLE, PIXY_RESULT_ERROR, array CANNOT BE USED.
 */

PixyChecksumState checkSumCalculator(uint8_t* array);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(DIR_PORT_CONVEYOR, DIR_PIN_CONVEYOR, GPIO_PIN_SET); //set conveyer in the right direction
	HAL_GPIO_WritePin(PRINTHEAD_DIR_PORT, PRINTHEAD_DIR_PIN, GPIO_PIN_SET); //set printhead to move left -> right
	HAL_GPIO_WritePin(GPIOC, DIR_Z_AXIS_Pin, GPIO_PIN_SET); //set z axis to move upwards
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//Time channel 3 for PWM, specifically for servo motor
	HAL_TIM_Base_Start(&htim1);// Time channel 1 to generate microsecond delays
	HAL_Delay(10);


//	HAL_SPI_Transmit(&hspi2, setLamp, 5, 100); // to switch on the pixy lamp

	// keypad gpio writes
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/**
	 *Operation starts here
	 */
	  keyPadScanRoutine();
	  //user has to hold '#' when there are no objects to stop the entire program
	  if(programState == programON){// user presses "*" key to start the program
		  normalOperationTestCase();
	  }else{
		  // user presses "C" key and let go to move printhead upwards
		  // user presses "D" key and let go to move printhead upwards
		  if(programState == adjustHeightUpwards || programState == adjustHeightDownwards){
			  moveZAxis();
		  }
		  //user has to press C or D key (whichever selected) to stop the movement
	  }
//	  normalOperationTestCase();

	/**
	 * Operation END
	 * */
	  /*
	   * Test cases to be operated to check system functionality is OK
	   */
//	  	  testCaseHomeMovement(100);
//	  	  xAxisMovement(40);
//	  	  testCasePressEveBot();
//	  	  testCaseZAxis(40);
//	  	  testCaseConveyerMovement();
//	  	  keyPadScanRoutine();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 144-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_7
                          |GPIO_PIN_8|EvebotButton_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|Black_Keypad_Pin|GPIO_PIN_6|Purple_Keypad_Pin
                          |Grey_Keypad_Pin|White_Keypad_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP_Z_AXIS_Pin|DIR_Z_AXIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitStart_Pin */
  GPIO_InitStruct.Pin = LimitStart_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LimitStart_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin PA7
                           PA8 EvebotButton_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin|GPIO_PIN_7
                          |GPIO_PIN_8|EvebotButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Orange_Keypad_Pin Yellow_Keypad_Pin Green_Keypad_Pin Blue_Keypad_Pin */
  GPIO_InitStruct.Pin = Orange_Keypad_Pin|Yellow_Keypad_Pin|Green_Keypad_Pin|Blue_Keypad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 Black_Keypad_Pin PB6 Purple_Keypad_Pin
                           Grey_Keypad_Pin White_Keypad_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|Black_Keypad_Pin|GPIO_PIN_6|Purple_Keypad_Pin
                          |Grey_Keypad_Pin|White_Keypad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Z_AXIS_Pin DIR_Z_AXIS_Pin */
  GPIO_InitStruct.Pin = STEP_Z_AXIS_Pin|DIR_Z_AXIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTick = HAL_GetTick();

	if ((currentTick - lastDebounceTime) > DEBOUNCE_DELAY)
	{
		lastDebounceTime = currentTick;

		uint16_t row_pins[4] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5};
		uint16_t col_pins[4] = {GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_11};
		const char keys[4][4] = {
			{'1', '2', '3', 'A'},
			{'4', '5', '6', 'B'},
			{'7', '8', '9', 'C'},
			{'*', '0', '#', 'D'}
		};

		// Set all row pins to HIGH initially
		for (int i = 0; i < 4; i++) {
			HAL_GPIO_WritePin(GPIOB, row_pins[i], GPIO_PIN_SET);
		}

		// Determine which column triggered the interrupt
		int col = -1;
		for (int i = 0; i < 4; i++)
		{
			if (col_pins[i] == GPIO_Pin)
			{
				col = i;
				break;
			}
		}

		// If a valid column is detected, proceed to check rows
		if (col != -1)
		{
			for (int row = 0; row < 4; row++)
			{
				HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_RESET);
				if (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET)
				{
					global = keys[row][col];
					// Wait for key release
					while (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET);
					break;
				}
				HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_SET);
			}
		}
	}
}

/*
 * @desc
 * Generates microsecond delay that otherwise not possible with HAL_Delay()
 * @param delay
 *
 */
void microDelay (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
/*
 * @desc
 * Converts the distance in mm to steps required for stepper to take
 * The "steps" are looping values to be iterated UNTIL.
 * @param distance in mm
 */
int distCal(int distance)//in mm
{
	//1 revolution - 200 steps of ON AND OFF pulses
	//In our case, 400 loops of ON AND OFF pulses separately/alternating, to mimic "200 steps of ON AND OFF pulses"
	int value = distance * 400/40;
	return value;
}
/*
 * @desc
 * Function is called after evebot button is pressed
 * The function is called to cover the distance gap that is marked on the evebot ruler before
 * the start of distance measurement at START on the ruler.
 *
 * The Printhead moves by 5mm forward +ve X axis, before retracting 5mm backwards -ve X axis.
 * Once back to the beginning of the printing position, movement() is called, with the PRINTIZE as the argument.
 */
void iniDelay(void)
{
	int x;
	int dist = distCal(INIDIST/2);
	int delay = 1000;
	for(x=0; x<dist; ++x)
	{
		HAL_GPIO_TogglePin(PRINTHEAD_STEP_PORT, PRINTHEAD_STEP_PIN);
		microDelay(delay);
	}
  	HAL_GPIO_TogglePin(PRINTHEAD_DIR_PORT, PRINTHEAD_DIR_PIN);
	for(x=0; x<dist; ++x)
	{
		HAL_GPIO_TogglePin(PRINTHEAD_STEP_PORT, PRINTHEAD_STEP_PIN);
		microDelay(delay);
	}
  	HAL_GPIO_TogglePin(PRINTHEAD_DIR_PORT, PRINTHEAD_DIR_PIN);
}
/*
 * @desc
 *	Checksum calculator to verify that data is reliable to use
 * @returns
 * 	PIXY_RESULT_OK - if data is reliable
 * 	PIXY_RESULT_ERROR - Do not use this data in the SPI Buffer
 */
PixyChecksumState checkSumCalculator(uint8_t* array)//calculate checksum of pixycam
{
	uint16_t size = array[3];
	size = size + 6;//including header bytes
	uint16_t checksum = array[5] << 8 | array[4];
	uint16_t cal_checksum = 0;
	for(int i = 6; i<size;i++)
	{
		cal_checksum+=array[i];
	}
	return checksum==cal_checksum?PIXY_RESULT_OK:PIXY_RESULT_ERROR;
}

/*
 * @desc
 *	SPI has both rubbish values and actual data
 *	Tries to find the starting position of the actual data in the SPI buffer
 * @returns
 * 	NULL - Actual data is not there
 * 	uint8_t* Points to the start of actual data
 */
uint8_t* tpixCheckStartCmd(uint8_t* spiRxBuffer) {
    for(int index = 0; index < SPI_BUFFER_SIZE; ++index){
        if (spiRxBuffer[index] == 0xAF && spiRxBuffer[index + 1] == 0xC1) {
        	responsePacketStart = &(spiRxBuffer[index]);
            return responsePacketStart;
        }
    }
//    printf("AF header byte not found in rx SPI buffer\r\n");
    return NULL;
}

/*
 * @desc
 *
 * 	1. Counts the number of pastries there are caught on camera
 *
 *	2. Deciphers the SPI Rx Buffer to get information of pastries that are in "pixy block signatures"
 *
 * @data
 *	Each Pastry object or "pixy block signatures has the following information"
 * 	//centrepoint of pastry coordinates
	1. coordX;
	2. coordY
	//dimensions
	1. height;
	2. width;
	//bottom line y axis location
	1. bottomY;
	//top line y axis location
	2. topY;
 *
 */
void retrieveGetBlocks(uint8_t* spiRxBuffer){
	uint8_t* responsePacketPtr = spiRxBuffer;
	if(responsePacketPtr != NULL){
		//retrieve
		numberOfBlocks = responsePacketPtr[3] / 14;
		int pasPosX = 8;
		int pasPosY = 10;

		for(int i = 0; i<numberOfBlocks; ++i, pasPosX+=14, pasPosY+=14)//loop for the no objects
		{
			pastry[i].coordX = responsePacketPtr[pasPosX+1] << 8 | responsePacketPtr[pasPosX];
			pastry[i].coordY = responsePacketPtr[pasPosY+1] << 8 | responsePacketPtr[pasPosY];
			pastry[i].width = responsePacketPtr[pasPosY+5] << 8 | responsePacketPtr[pasPosY+4];
			pastry[i].height = responsePacketPtr[pasPosX+7] << 8 | responsePacketPtr[pasPosX+6];
			pastry[i].bottomY = pastry[i].coordY + (pastry[i].height/2);
			pastry[i].topY = pastry[i].coordY - (pastry[i].height/2);
		}
	}else{
		//Do not use buffer if is literally empty
	}
}
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
void printHeadMovementRoutine_Manual(){
	conveyerMovePastry();
    int offset = PRINTSIZE/2;
    int datum = DATUM+offset;

    for(int i = 0; i< row.numberOfRowObjects; i++)
    {
    	int dist = distCal((row.coordXArray[i]- datum) * PIXY_RATIO);
    	//travels to print location
    	xAxisMovement(dist);

        HAL_Delay(3000); // person presses the evebot button
        iniDelay(); // covers the print gap (distance gap where printing is not happening)
        xAxisMovement(PRINTSIZE); //prints along the PRINTSIZE width
        HAL_Delay(3000); // person presses the evebot button
        datum = row.coordXArray[i] + (PRINTSIZE-offset); // new datum point is set at the end of the bread width
        //print head now travels from the current datum point (at the end of the print length) to the next centre point of the object
    }
}
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
void printHeadMovementRoutine_AUTO(){
	conveyerMovePastry();
    int offset = PRINTSIZE/2;
    int datum = DATUM+offset;

    for(int i = 0; i< row.numberOfRowObjects; i++)
    {
    	int dist = distCal((row.coordXArray[i]- datum) * PIXY_RATIO);
    	//travels to print location
    	xAxisMovement(dist);
        pressEveBot();
        HAL_Delay(100);
        iniDelay();
        HAL_Delay(100);
        xAxisMovement(PRINTSIZE);
        pressEveBot();
        HAL_Delay(100);
        datum = row.coordXArray[i] + (PRINTSIZE-offset);
    }
}
/*
 * @desc
 *	Moves the printhead according to the int z (distance in mm specified)
 * @params
 * int z is the distance in mm
 */
void xAxisMovement(int distance){
    int dist = distCal(distance);
    for(int x =0; x<dist;++x)
    {
		HAL_GPIO_TogglePin(PRINTHEAD_STEP_PORT, PRINTHEAD_STEP_PIN);
		microDelay(stepDelay);
    }
}
/*
 * @desc
 *	1. Find a reference point for a row object i.e. lowest centre point or lowest object
 *  2. Add all the pastries that falls within the buffer range of the reference point. i.e. y axis range from centrepoint
 *
 *  The new row will contain all the x_coords of pastries to be printed.
 */
int createRows()
{
	//find ROW
	rowval = 0;
//	uint16_t rowval = 0;
	for(int j = 0; j<numberOfBlocks; ++j)
	{
		//find the lowest centre point of an object within a "row" AND ONLY IF
		//The bottom width of the object is before the EVENT HORIZON i.e. PRINTLINE - 10
		//the lowest centre point of the object will be the main reference for a "row"
		if(pastry[j].coordY > rowval && pastry[j].bottomY<= EVENT_HORIZON)
		{
			rowval = pastry[j].coordY;
		}

	}
	//stores the distance from the lowest centre point of a row to the print line.
	row.coordRow_Y = PRINT_LINE - rowval;
	//Reset OBJECTS before creating a new row
	row.numberOfRowObjects = 0;
	for (int i = 0; i< 10; i++)
	{
		row.coordXArray[i] = 400;
	}
	//Calibrate Objects
	for(int i = 0; i< numberOfBlocks; ++i)//loop for the no objects
	{
		//since the lowest centrepoint is the reference for the row,
		//we account for objects within the gap
		//before the lowest centrepoint
		if(pastry[i].coordY >= (rowval - 10) && pastry[i].coordY <= rowval){
			row.coordXArray[i] = pastry[i].coordX;
			row.numberOfRowObjects++;
		}
	}
	//sorts the row objects, from the smallest (i.e. closest to the left to the rightmost) along the xAxis
	bubbleSort(row.coordXArray, 10);
	return row.numberOfRowObjects;
}
/*
 * @desc
 *	Helper Swap function for bubblesort
 */
void swap(uint16_t *xp, uint16_t *yp) {
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}
/*
 * @desc
 *	Sorts the objects in the PastryData[] array according to distance closest from
 *	the left (datum) to the rightmost along the x axis
 */
void bubbleSort(uint16_t arr[], int n) {
    int i, j;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                swap(&arr[j], &arr[j + 1]);
            }
        }
    }
}
/*
 * @desc
 *	Sorts the objects in the PastryData[] array according to distance closest from
 *	the left (datum) to the rightmost along the x axis
 */
void conveyerMovement(int z){
    int dist = distCal(z);
    int conveyerStepperDelay = 1900;
    //moves to set distance
    for(int x =0; x<dist;++x)
    {
		HAL_GPIO_TogglePin(STEP_PORT_CONVEYOR, STEP_PIN_CONVEYOR);
		microDelay(conveyerStepperDelay);
    }
}
/*
 * @desc
 *	Test case for x axis home movement
 * @params
 * distance (in mm) to test. Max is 250mm
 */
void testCaseHomeMovement(int z){
	if(z > 250){
		z = 250;
	}
	if(z < 0){
		z = 0;
	}
	xAxisMovement(z);
	backToHomePosition();
	while(1){

	}
}
/*
 * @desc
 *	Test case for pressing of evebot button (Whether successful button presses is achieved)
 */
void testCasePressEveBot(){
	  pressEveBot();
	  while(1){

	  }
}
/*
 * @desc
 *	Test case for conveyer movement to move 14 cm forward (towards the printhead)
 */
void testCaseConveyerMovement(){
	HAL_Delay(1000);
	conveyerMovement(140);
	while(1){

	}
}
/*
 * @desc
 *	Test case z axis based on mm height that programmer puts
 * @param
 * int z in mm height
 */
void testCaseZAxis(int z){
    int dist = distCal(z);
    int conveyerStepperDelay = 2000;
    HAL_GPIO_TogglePin(GPIOC, DIR_Z_AXIS_Pin);
    //moves to set distance
    for(int x =0; x<dist;++x)
    {
		HAL_GPIO_TogglePin(GPIOC, STEP_Z_AXIS_Pin);
		microDelay(conveyerStepperDelay);
    }
    while(1){

    }
}
/*
 * @desc
 *	Function will always be called in the main loop whenever the programState is set to
 *	1. adjustHeightUpwards
 *	2. adjustHeightDownwards
 *
 */
void moveZAxis(){
	HAL_GPIO_TogglePin(GPIOC, STEP_Z_AXIS_Pin);
	microDelay(2000);
}

/*
 * @desc
 *	Moves the printhead back towards the leftmost/HOME position
 *	until the printhead hits the limit switch
 *
 */
void backToHomePosition() {
  HAL_GPIO_WritePin(PRINTHEAD_DIR_PORT, PRINTHEAD_DIR_PIN, GPIO_PIN_RESET);
  while(1){
	  GPIO_PinState limitState = HAL_GPIO_ReadPin(LimitStart_GPIO_Port, LimitStart_Pin);
	  if(limitState == GPIO_PIN_RESET){
		  break;
	  }else{
		  HAL_GPIO_TogglePin(PRINTHEAD_STEP_PORT, PRINTHEAD_STEP_PIN);
		  microDelay(stepDelay);
	  }
  }
  HAL_GPIO_WritePin(PRINTHEAD_DIR_PORT, PRINTHEAD_DIR_PIN, GPIO_PIN_SET);
}
/*
 * @desc
 *	Moves the printhead back towards the leftmost/HOME position
 *	until the printhead hits the limit switch
 *
 */
void pressEveBot(void){
	//higher the value - anticlockwise - i.e. goes down further
	//lower the value - clockwise (capped 200) - i.e. goes up further
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, RETRACT_PEN_UP);
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, EXTEND_PEN_DOWN);
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, RETRACT_PEN_UP);
  HAL_Delay(500);

}

/*
 * Will be iterated infinitely in the while loop
 * Conveyer runs upon detection of pastries
 * The conveyer runs until the lowest object
 */
void normalOperationTestCase(){
	    HAL_SPI_Transmit(&hspi2, getBlocks, 6, 100);
	    HAL_Delay(100);
		HAL_SPI_Receive(&hspi2, spiRxBuffer, SPI_BUFFER_SIZE, 100);
		uint8_t* startingResponsePtr = tpixCheckStartCmd(spiRxBuffer);
		if(startingResponsePtr == NULL){

		}else{
			checkSumStatus = checkSumCalculator(startingResponsePtr);
			if(checkSumStatus == PIXY_RESULT_OK){
				retrieveGetBlocks(startingResponsePtr);
				int pastryCount = createRows();
				if(pastryCount > 0){
//					printHeadMovementRoutine_Manual();
					printHeadMovementRoutine_AUTO();
					backToHomePosition();
					//let color signature set in
					HAL_Delay(300);
				}
			}else if(checkSumStatus == PIXY_RESULT_ERROR){
				//do nothing
			}
		}


}
/*
 * @desc
 *	Moves the lowest centrepoint object among all objects along the row towards the printline
 *
 */
void conveyerMovePastry(){
	int distanceToTravel = row.coordRow_Y * PIXY_RATIO;
	conveyerMovement(distanceToTravel);

}

/*
 * @desc
 * @return
 * Returns the key pressed among the keys found on the keypad
 *
 */
//char Keypad_Scan(void)
//{
//    uint16_t row_pins[4] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5};
//    uint16_t col_pins[4] = {GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_11};
//    const char keys[4][4] = {
//        {'1', '2', '3', 'A'},
//        {'4', '5', '6', 'B'},
//        {'7', '8', '9', 'C'},
//        {'*', '0', '#', 'D'}
//    };

//    // Set all row pins to HIGH initially
//    for (int i = 0; i < 4; i++) {
//        HAL_GPIO_WritePin(GPIOB, row_pins[i], GPIO_PIN_SET);
//    }
//
//    for (int col = 0; col < 4; col++) {
//        // Set the current column pin to LOW
//        HAL_GPIO_WritePin(GPIOB, col_pins[col], GPIO_PIN_RESET);
//
//        for (int row = 0; row < 4; row++) {
//            // Check if the row pin is LOW (key pressed)
//            if (HAL_GPIO_ReadPin(GPIOB, row_pins[row]) == GPIO_PIN_RESET) {
//                // Wait for key release
//                while (HAL_GPIO_ReadPin(GPIOB, row_pins[row]) == GPIO_PIN_RESET);
//
//                // Set the current column pin back to HIGH
//                HAL_GPIO_WritePin(GPIOB, col_pins[col], GPIO_PIN_SET);
//
//                rowKeypadMatrix = row;
//                columnKeypadMatrix = col;
//                theKey = keys[row][col];
//                return keys[row][col];
//            }
//        }
//
//        // Set the current column pin back to HIGH
//        HAL_GPIO_WritePin(GPIOB, col_pins[col], GPIO_PIN_SET);
//    }
//
//    return 'L'; // No key pressed

//    if (scanState == SCAN_STATE_IDLE) {
//    	return 'L'; // No key pressed
//	} else {
//		return detectedKey;
//	}
//}
/*
 * @desc
 * Main control program - Dictates whether the program is
 * 1. adjusting the print size,
 * 2. adjusting the height of current print head i.e. z axis
 * 3. main program execution
 *
 */
void keyPadScanRoutine(){
	uint32_t currentTick = HAL_GetTick();

	if (global != 'L' && (currentTick - lastDebounceTime) > DEBOUNCE_DELAY)
	{
		char key = global;
		global = 'L'; // Reset global key after processing

		lastDebounceTime = currentTick; // Update debounce time

		if (key == 'A' && programState != programON)
		{
			listeningForBreadWidth = true;
			bufferIndex = 0;
			memset(inputBuffer, 0, sizeof(inputBuffer));
		}
		else if (key == 'B' && listeningForBreadWidth)
		{
			breadWidth = atoi(inputBuffer);
			PRINTSIZE = breadWidth;
			listeningForBreadWidth = false;
		}
		else if (key == 'C')
		{
			if(adjustmentState == upwards_set)
			{
				programState = programOFF;
				adjustmentState = no_set;
			}
			else if(adjustmentState == no_set || programState == adjustHeightDownwards)
			{
				adjustmentState = upwards_set;
				programState = adjustHeightUpwards;
				HAL_GPIO_WritePin(GPIOC, DIR_Z_AXIS_Pin, GPIO_PIN_SET);
				HAL_Delay(1);
			}
		}
		else if (key == 'D')
		{
			if(adjustmentState == downwards_set)
			{
				programState = programOFF;
				adjustmentState = no_set;
			}
			else if(adjustmentState == no_set || programState == adjustHeightUpwards)
			{
				adjustmentState = downwards_set;
				programState = adjustHeightDownwards;
				HAL_GPIO_WritePin(GPIOC, DIR_Z_AXIS_Pin, GPIO_PIN_RESET);
				HAL_Delay(1);
			}
		}
		else if (key == '*' && !listeningForBreadWidth && adjustmentState == no_set)
		{
			programState = programON;
		}
		else if (key == '#' && !listeningForBreadWidth)
		{
			programState = programOFF;
		}
		else if (listeningForBreadWidth && key >= '0' && key <= '9')
		{
			if (bufferIndex < sizeof(inputBuffer) - 1)
			{
				inputBuffer[bufferIndex++] = key;
			}
		}
	}

	// Non-blocking delay for any other necessary operations
	if (HAL_GetTick() - previousTick >= BLINK_DELAY)
	{
		previousTick = HAL_GetTick();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
