/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "HAL_MPU9250.h"
#include "sbzrgnCM.h"
#include "attitude_estimator.hpp"
#include "eeprom.h"

//#define TEENSIZE
#define KIDSIZE


#define M_PI                3.1415926535897932384626433832795
#define QUEUE_SIZE					30
#define ZERO_MAX            20
#define ZERO_MIN           -20
#define BATT_TRESHOLD       11
#define BATT_SCALE          0.054

using namespace stateestimation;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
MPU9250TypeDef MPU9250;

USBLineTypeDef MasterLine1 ; // USART1 (MainBoard)

UARTLineTypeDef SlaveLine1 ; // USART3

UARTLineTypeDef SlaveLine2 ; // USART5

UARTLineTypeDef SlaveLine3 ; // USART6

AttitudeEstimator Est(true);
///////////////////////////////////////////////////////////////////////////////////////////////////
double SB_Timer_Value = 0;
double SB_Timer_Scale = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t Switch1_State = 0;
uint8_t Switch2_State = 0;
uint8_t Switch3_State = 0;
uint8_t Switch4_State = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
double BatteryVoltage = 0;
uint8_t Shifted_BatteryVoltage = 0;
uint16_t SB_LowBatteryCounter = 0;
uint16_t SB_BuzzerState = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IMU_Buffer[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t CHECK = 0;

double Roll = 0, Pitch = 0, Yaw = 0;
uint16_t Shifted_Roll = 0, Shifted_Pitch = 0, Shifted_Yaw = 0;

int16_t Raw_Accel[3] = {0,0,0};
uint16_t Shifted_Accel_X = 0, Shifted_Accel_Y = 0, Shifted_Accel_Z = 0;
int16_t Last_Raw_Accel_X = 0, Last_Raw_Accel_Y = 0, Last_Raw_Accel_Z = 0;
double AX = 0, AY = 0, AZ = 0;

int16_t Raw_Gyro[3] = {0,0,0};
uint16_t Shifted_Gyro_X = 0, Shifted_Gyro_Y = 0, Shifted_Gyro_Z = 0;
int16_t Last_Raw_Gyro_X = 0, Last_Raw_Gyro_Y = 0, Last_Raw_Gyro_Z = 0;
double GX = 0, GY = 0, GZ = 0;
uint8_t CalibCounter = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t Loop_Counter = 0;
uint8_t nReadyPackets = 0;
int16_t lastSentIndex = -1;
uint16_t Queue[QUEUE_SIZE];
uint8_t MainBoard_ResponsePacket[100];
uint8_t LookupTable[200];

int16_t GX_Off = 0, GY_Off = 0, GZ_Off = 0, AX_Off = 0, AY_Off = 0, AZ_Off = 0;
int16_t EERead_GX_OFF, EERead_GY_OFF, EERead_GZ_OFF, EERead_AX_OFF, EERead_AY_OFF, EERead_AZ_OFF;
uint8_t Calibrated;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t Master_CheckPacket(UART_HandleTypeDef *UARTx, USBLineTypeDef* master);
SB_StatusTypeDef Slave_CheckPacket(UART_HandleTypeDef *UARTx, UARTLineTypeDef* slave);
void MainBoard_ReadRegister(uint8_t *packet);
void MainBoard_WriteRegister(uint8_t *packet);
void Queue_Data_Send(void);
uint8_t DXL_CheckSumCalc(uint8_t *packet);
void IMU_PWR_Reset(void);
void IMU_Init(void);
void IMU_Filter_Init(void);
void IMU_Update(void);
void IMU_Filter_Update(void);
void Button_Update(void);
double ReadVoltage(void);
void Battery_Update(void);
void LookUpTable_Update(void);
void Struct_Initializer(USBLineTypeDef* stct);
void Debugger(void);
void Calibration_Init(void);
void Calibration(void);
void set_offset(void);
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
	HAL_FLASH_Unlock();
	EE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//Buffer Configration
	Struct_Initializer(&MasterLine1);
	
	SlaveLine1.BufferCurrentIndex = 0;
	SlaveLine1.CurrentState = 0;
	SlaveLine1.Length = 0;
	
	SlaveLine2.BufferCurrentIndex = 0;
	SlaveLine2.CurrentState = 0;
	SlaveLine2.Length = 0;
	
	SlaveLine3.BufferCurrentIndex = 0;
	SlaveLine3.CurrentState = 0;
	SlaveLine3.Length = 0;
	
	MainBoard_ResponsePacket[0] = 255;
	MainBoard_ResponsePacket[1] = 255;
	MainBoard_ResponsePacket[2] = MY_ID;
	
	IMU_Init();
	
	IMU_Filter_Init();
	
	//Calibration();

	HAL_UART_Receive_IT(&huart3, &SlaveLine1.InputData, 1);
	HAL_UART_Receive_IT(&huart5, &SlaveLine2.InputData, 1);
	HAL_UART_Receive_IT(&huart6, &SlaveLine3.InputData, 1);
	HAL_UART_Receive_IT(&huart1, &MasterLine1.InputData, 1);
	
	HAL_GPIO_WritePin(FT_EN_GPIO_Port,FT_EN_Pin,GPIO_PIN_SET);

	EE_ReadVariable(0, &EERead_GX_OFF);
	EE_ReadVariable(1, &EERead_GY_OFF);
	EE_ReadVariable(2, &EERead_GZ_OFF);
	EE_ReadVariable(3, &EERead_AX_OFF);
	EE_ReadVariable(4, &EERead_AY_OFF);
	EE_ReadVariable(5, &EERead_AZ_OFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Button_Update();
		
		IMU_Update();
		
		IMU_Filter_Update();
		
		Battery_Update();
		
		if(Loop_Counter == 2000)
		{
			//TeeeeEST
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			//TeeeeEST
			Loop_Counter = 0;
		}
		Loop_Counter++;

		LookUpTable_Update();

		Debugger();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 40000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 1000000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 1000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|IMU_PWR_DIS_Pin|DXL_3_TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin|DXL_PWR_EN_Pin|DXL_5_TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DXL_6_TX_EN_Pin|FT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW4_Pin SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW4_Pin|SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA9 PA10
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_PWR_DIS_Pin */
  GPIO_InitStruct.Pin = IMU_PWR_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_PWR_DIS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DXL_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DXL_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DXL_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB10 PB11
                           PB12 PB13 PB14 PB3
                           PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DXL_6_TX_EN_Pin */
  GPIO_InitStruct.Pin = DXL_6_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DXL_6_TX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DXL_5_TX_EN_Pin */
  GPIO_InitStruct.Pin = DXL_5_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DXL_5_TX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DXL_3_TX_EN_Pin */
  GPIO_InitStruct.Pin = DXL_3_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DXL_3_TX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FT_EN_Pin */
  GPIO_InitStruct.Pin = FT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FT_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UART receive callbback
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//---------------------------------------------------
	if(huart ->Instance == USART1) // USART1 >>>>> MasterLine1
	{
		//uint8_t *bufferEL=&buffer[bufferlastindex]; // mahze ettlea baraye ayande
		uint8_t packetLength = Master_CheckPacket(huart, &MasterLine1);
		if(packetLength > 0)
		{
			if(MasterLine1.Buffer[MasterLine1.BufferLastIndex+2] == MY_ID)
			{
				if(DXL_CheckSumCalc(&MasterLine1.Buffer[MasterLine1.BufferLastIndex]) == MasterLine1.Buffer[MasterLine1.BufferLastIndex+packetLength-1]) 
				{
					switch(MasterLine1.Buffer[MasterLine1.BufferLastIndex+4])
					{
						case DXL_READ_DATA:
							MainBoard_ReadRegister(&MasterLine1.Buffer[MasterLine1.BufferLastIndex]);
						break;
						case DXL_WRITE_DATA:
							if(MasterLine1.Buffer[MasterLine1.BufferLastIndex+5] == 0x18 && MasterLine1.Buffer[MasterLine1.BufferLastIndex+6] == 0x01)
							{								
								HAL_GPIO_WritePin(DXL_PWR_EN_GPIO_Port, DXL_PWR_EN_Pin, GPIO_PIN_SET);
								Est.reset(1,1);
							}
							else if (MasterLine1.Buffer[MasterLine1.BufferLastIndex+5] == 0x18 && MasterLine1.Buffer[MasterLine1.BufferLastIndex+6] == 0x00)
							{
								HAL_GPIO_WritePin(DXL_PWR_EN_GPIO_Port, DXL_PWR_EN_Pin, GPIO_PIN_RESET);
							}
							else if(MasterLine1.Buffer[MasterLine1.BufferLastIndex+5] == 0x4A && MasterLine1.Buffer[MasterLine1.BufferLastIndex+6] == 0x01)
							{
								CalibCounter = 0;
								Calibration();
							}
							else
							{
								MainBoard_WriteRegister(&MasterLine1.Buffer[MasterLine1.BufferLastIndex]);
							}
						break;
					}
				}
			}
			else
			{
				Queue[nReadyPackets]= MasterLine1.BufferCurrentIndex - packetLength;
				nReadyPackets++;
				if(nReadyPackets >= QUEUE_SIZE)
				{
					nReadyPackets = QUEUE_SIZE;
				}
			}
			if((BUFFER_SIZE - (int16_t)MasterLine1.BufferCurrentIndex) < MAX_PACKET_SIZE)
			{
				MasterLine1.BufferCurrentIndex = 0;
				MasterLine1.BufferLastIndex = 0;
			}
			MasterLine1.BufferLastIndex = MasterLine1.BufferCurrentIndex;
			Queue_Data_Send();
		}
		
	}
//----------------------------------------------------------------
	else if(huart ->Instance == USART3)
	{
		if(Slave_CheckPacket(&huart3, &SlaveLine1) == SB_OK)
		{
			HAL_UART_Transmit_IT(&huart1, SlaveLine1.Buffer, SlaveLine1.Buffer[3]+4);
		}
	}
	else if (huart->Instance == UART5)
	{
		if(Slave_CheckPacket(&huart5, &SlaveLine2) == SB_OK)
		{
			HAL_UART_Transmit_IT(&huart1, SlaveLine2.Buffer, SlaveLine2.Buffer[3]+4);
		}
	}
	else if (huart->Instance == USART6)
	{
		if(Slave_CheckPacket(&huart6, &SlaveLine3) == SB_OK)
		{
			HAL_UART_Transmit_IT(&huart1, SlaveLine3.Buffer, SlaveLine3.Buffer[3]+4);
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UART transmit callback
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_GPIO_WritePin(DXL_3_TX_EN_GPIO_Port, DXL_3_TX_EN_Pin , GPIO_PIN_RESET);//Receive Enable
	}
	else if(huart->Instance == UART5)
	{
		HAL_GPIO_WritePin(DXL_5_TX_EN_GPIO_Port, DXL_5_TX_EN_Pin , GPIO_PIN_RESET);//Receive Enable
	}
	else if(huart->Instance == USART6)
	{
		HAL_GPIO_WritePin(DXL_6_TX_EN_GPIO_Port, DXL_6_TX_EN_Pin , GPIO_PIN_RESET);//Receive Enable
	}
	Queue_Data_Send();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//this function is used to extract dynamixel packet send by main board
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t Master_CheckPacket(UART_HandleTypeDef *UARTx, USBLineTypeDef* master)
{
	if(HAL_UART_Receive_IT(UARTx, &master->InputData, 1) == HAL_OK)
	{
		master->Buffer[master->BufferCurrentIndex]=master->InputData;
		master->BufferCurrentIndex++;
		switch(master->CurrentState)
		{
			case 0:
				if(master->Buffer[master->BufferLastIndex] == 255) {master->CurrentState = 1;}
				else {master->CurrentState = 0;	master->BufferCurrentIndex = master->BufferLastIndex;}
			break;
			
			case 1:
				if(MasterLine1.Buffer[master->BufferLastIndex+1] == 255) {master->CurrentState = 2;}
  			else {master->CurrentState = 0; master->BufferCurrentIndex = master->BufferLastIndex;}
			break;
			
			case 2:
				if(MasterLine1.Buffer[master->BufferLastIndex+2] != 0xFF) {master->CurrentState=3;}
  			else {master->CurrentState = 0; master->BufferCurrentIndex = master->BufferLastIndex;}
			break;
			
			case 3:
				if(MasterLine1.Buffer[master->BufferLastIndex+3] > master->Length) {master->Length++;}
  			else
				{
					master->Length = 0;
					master->CurrentState = 0;
					uint8_t packetLength = MasterLine1.Buffer[master->BufferLastIndex+3]+4;
					return packetLength;
				}
			break;
		}
	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//this function is used to extract dynamixel packet send by DXL chain line
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SB_StatusTypeDef Slave_CheckPacket(UART_HandleTypeDef *UARTx, UARTLineTypeDef* slave)
{
	if(HAL_UART_Receive_IT(UARTx, &slave->InputData , 1) == HAL_OK)
	{
		slave->Buffer[slave->BufferCurrentIndex] = slave->InputData;
		slave->BufferCurrentIndex ++;	
		switch(slave->CurrentState)
		{
			case 0:
				if(slave->Buffer[0] == 0xFF) {slave->CurrentState = 1;}
				else {slave->CurrentState = 0;	slave->BufferCurrentIndex = 0;}
			break;
			
			case 1:
				if(slave->Buffer[1] == 0xFF) {slave->CurrentState = 2;}
  			else {slave->CurrentState = 0; slave->BufferCurrentIndex = 0;}
			break;
			
			case 2:
				if(slave->Buffer[2] != 0xFF) {slave->CurrentState = 3;}
  			else {slave->CurrentState = 0; slave->BufferCurrentIndex = 0;}
			break;
			
			case 3:
				if(slave->Buffer[3] > slave->Length) {slave->Length++;}
  			else
				{
					slave->Length = 0;
					slave->CurrentState = 0;
					slave->BufferCurrentIndex = 0;
					return SB_OK;
				}
			break;
		}
	}
	return SB_ERROR;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ft send packet function
//packet[5] >> address of the registers to be read             packet[6] >> number of data to be read
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainBoard_ReadRegister(uint8_t *packet)
{
	MainBoard_ResponsePacket[3] = packet[6] + 2;
	MainBoard_ResponsePacket[4] = DXL_Error;
	for(int8_t i = 0 ; i < packet[6] ; i++)
	{
		MainBoard_ResponsePacket[i + 5] = LookupTable[packet[5] + i];
	}
	MainBoard_ResponsePacket[packet[6] + 5] = DXL_CheckSumCalc(MainBoard_ResponsePacket);
	HAL_UART_Transmit_IT(&huart1,MainBoard_ResponsePacket, packet[6] + 6);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//writing data to the specified register
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainBoard_WriteRegister(uint8_t *packet)
{
	uint8_t i;
	for(i = 0; i < packet[3] - 3; i++)
	{
		LookupTable[packet[5]+i] = packet[i+6];
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function is called whenever any ready packet whould be available
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Queue_Data_Send(void)
{
	if(nReadyPackets > 0 && huart3.gState == HAL_UART_STATE_READY && huart5.gState == HAL_UART_STATE_READY && huart6.gState == HAL_UART_STATE_READY)
	{
		uint8_t *data = &MasterLine1.Buffer[Queue[lastSentIndex+1]];
		
		HAL_GPIO_WritePin(DXL_3_TX_EN_GPIO_Port, DXL_3_TX_EN_Pin , GPIO_PIN_SET);//Transmit Enable
		HAL_GPIO_WritePin(DXL_5_TX_EN_GPIO_Port, DXL_5_TX_EN_Pin , GPIO_PIN_SET);//Transmit Enable
		HAL_GPIO_WritePin(DXL_6_TX_EN_GPIO_Port, DXL_6_TX_EN_Pin , GPIO_PIN_SET);//Transmit Enable
		
		HAL_UART_Transmit_IT(&huart3 , data , data[3]+4);//data transmision via interrupt
		HAL_UART_Transmit_IT(&huart5 , data , data[3]+4);//data transmision via interrupt
		HAL_UART_Transmit_IT(&huart6 , data , data[3]+4);//data transmision via interrupt
		
		nReadyPackets--;
		lastSentIndex++;
		if(nReadyPackets < 1 || lastSentIndex == QUEUE_SIZE)
		{
			lastSentIndex = -1;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//checksum calculation for input packet
//Note : in dynamixels frame the fourth byte (packet[3]) is always indicates the length of the packet
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t DXL_CheckSumCalc(uint8_t *packet)
{
	uint8_t i;
	uint8_t checksum =0;
	for(i = 2; i < packet[3] + 3; i++)
	{
		checksum += packet[i];
	}
	checksum =~ checksum;
	return(checksum);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU PWR Restart
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_PWR_Reset(void)
{
	HAL_GPIO_WritePin(IMU_PWR_DIS_GPIO_Port,IMU_PWR_DIS_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
	HAL_GPIO_WritePin(IMU_PWR_DIS_GPIO_Port,IMU_PWR_DIS_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU Init
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Init(void)
{
	MPU9250.Gyro_DLPF   = BW250_D970;
	MPU9250.Accel_DLPF  = BW218_D1880;
	MPU9250.Gyro_Range  = MPU9250_Gyroscope_2000;
	MPU9250.Accel_Range = MPU9250_Accelerometer_16;
	
	IMU_PWR_Reset();

	if(MPU9250_Init(&hspi1, &MPU9250) != MPU9250_RESULT_OK)
	{
		// GOTO IMU_ERROR HANDLER
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU_Filter Init
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Filter_Init(void)
{
	//Configure Filter's Initial State
	Est.reset(1,1);
	Est.setPIGains(2.2, 2.65, 10.0, 1.25);
  Est.setQLTime(3.0);
  Est.setGyroBias(0.0, 0.0, 0.0);
  Est.setAccMethod(Est.ME_FUSED_YAW);
	//this timre is used to calculate time passed between every time "IMU_Filter_Update" called
	SB_Timer_Scale = (double) 1 / ((double) (HAL_RCC_GetPCLK1Freq() * 2) / (htim2.Init.Prescaler + 1));
	
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_TIM_SetCounter(&htim2,0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update IMU Data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Update(void)
{
	//Reading IMU Data in 8Bit
	MPU9250_ReadData(&hspi1, IMU_Buffer, MPU9250_ACCEL_XOUT_H, 6);
	//merging Data to get real IMU data
	Raw_Accel[0] = (int16_t)(IMU_Buffer[0] << 8 | IMU_Buffer[1]);
	Raw_Accel[1] = (int16_t)(IMU_Buffer[2] << 8 | IMU_Buffer[3]);
	Raw_Accel[2] = (int16_t)(IMU_Buffer[4] << 8 | IMU_Buffer[5]);
	MPU9250_ReadData(&hspi1, IMU_Buffer, MPU9250_GYRO_XOUT_H, 6);
	Raw_Gyro[0] = (int16_t)(IMU_Buffer[0] << 8 | IMU_Buffer[1]);
	Raw_Gyro[1] = (int16_t)(IMU_Buffer[2] << 8 | IMU_Buffer[3]);
	Raw_Gyro[2] = (int16_t)(IMU_Buffer[4] << 8 | IMU_Buffer[5]);

	//Removing Offset (extraxted using calibration function in OffLine mode)
	Raw_Gyro[0] += EERead_GX_OFF;
	Raw_Gyro[1] += EERead_GY_OFF;
	Raw_Gyro[2] += EERead_GZ_OFF;
	Raw_Accel[0] += EERead_AX_OFF;
	Raw_Accel[1] += EERead_AY_OFF;
	Raw_Accel[2] += EERead_AZ_OFF;
	//Removing Gyro's Noise around 0 state
	Raw_Gyro[0] = (Raw_Gyro[0] <= ZERO_MAX && Raw_Gyro[0] >= ZERO_MIN)? 0: Raw_Gyro[0];
	Raw_Gyro[1] = (Raw_Gyro[1] <= ZERO_MAX && Raw_Gyro[1] >= ZERO_MIN)? 0: Raw_Gyro[1];
	Raw_Gyro[2] = (Raw_Gyro[2] <= ZERO_MAX && Raw_Gyro[2] >= ZERO_MIN)? 0: Raw_Gyro[2];
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update IMU Filter Data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Filter_Update(void)
{
	if( Last_Raw_Gyro_X != Raw_Gyro[0] || Last_Raw_Gyro_Y != Raw_Gyro[1] || Last_Raw_Gyro_Z != Raw_Gyro[2] ||
					Last_Raw_Accel_X != Raw_Accel[0] || Last_Raw_Accel_Y != Raw_Accel[1] || Last_Raw_Accel_Z != Raw_Accel[2] )
	{
		Last_Raw_Gyro_X = Raw_Gyro[0];
		Last_Raw_Gyro_Y = Raw_Gyro[1];
		Last_Raw_Gyro_Z = Raw_Gyro[2];
		Last_Raw_Accel_X = Raw_Accel[0];
		Last_Raw_Accel_Y = Raw_Accel[1];
		Last_Raw_Accel_Z = Raw_Accel[2];
		//Preparing Filter's Data
		#ifdef TEENSIZE
		GX = ((-1*Raw_Gyro[1]) / 16.4);
		GY = (Raw_Gyro[0] / 16.4);
		GZ = (Raw_Gyro[2] / 16.4);
		AX = ((-1*Raw_Accel[1]) / 2048.0);
		AY = (Raw_Accel[0] / 2048.0);
		AZ = (Raw_Accel[2] / 2048.0);
		#endif
		
		#ifdef KIDSIZE
		GX = (Raw_Gyro[1] / 16.4);
		GY = ((-1*Raw_Gyro[0]) / 16.4);
		GZ = (Raw_Gyro[2] / 16.4);
		AX = (Raw_Accel[1] / 2048.0);
		AY = ((-1*Raw_Accel[0]) / 2048.0);
		AZ = (Raw_Accel[2] / 2048.0);
		#endif
		//Degree To Radian
		GX = GX / (180.0 / M_PI);
		GY = GY / (180.0 / M_PI);
		GZ = GZ / (180.0 / M_PI);
		//Smaple Time Update
		SB_Timer_Value = __HAL_TIM_GetCounter(&htim2) * SB_Timer_Scale;
		__HAL_TIM_SetCounter(&htim2,0);
		//Filter Update Function
		Est.update(SB_Timer_Value,GX,GY,GZ,AX,AY,AZ,0,0,0);
		Roll = Est.eulerRoll();
		Pitch = Est.eulerPitch();
		Yaw = Est.eulerYaw();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update Push-Button(s) data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Button_Update(void)
{
	Switch4_State = (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) ^ 0x01) << 0;
	Switch3_State = (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) ^ 0x01) << 1;
	Switch2_State = (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) ^ 0x01) << 2;
	Switch1_State = (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) ^ 0x01) << 3;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read Battery Voltage
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ReadVoltage(void)
{	
	uint32_t raw_adc = 0;
  double volt = 0;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	raw_adc = HAL_ADC_GetValue(&hadc1);
	volt = raw_adc * BATT_SCALE;
	HAL_ADC_Stop(&hadc1);
	
	return volt;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Battery Update
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Battery_Update(void)
{
	double volt = 0;
	
	volt = ReadVoltage();
	
	if(volt <= BATT_TRESHOLD)
	{
		SB_LowBatteryCounter++;
	}
	else
	{
		SB_LowBatteryCounter -= SB_LowBatteryCounter >> 1;
	}
	if(SB_LowBatteryCounter >= 200)
	{
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
		TIM5->CCR1 = 50;
		SB_BuzzerState = 1;
	}
	else if(SB_BuzzerState == 1 && SB_LowBatteryCounter < 10)
	{
		HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
		SB_BuzzerState = 0;
	}
	BatteryVoltage = volt;	
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update LookUp Table
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LookUpTable_Update(void)
{
	//Preparing IMU's Data for 8Bit UART Buffer
	#ifdef TEENSIZE
	Shifted_Gyro_X  = (-1*Raw_Gyro[1]) + 32768;	  //Converting the Gyro_X to 16-bit Variable
	Shifted_Gyro_Y  = (Raw_Gyro[0]) + 32768;	  //Converting the Gyro_Y to 16-bit Variable
	Shifted_Gyro_Z  = (Raw_Gyro[2]) + 32768; 	//Converting the Gyro_Z to 16-bit Variable

	Shifted_Accel_X = (-1*Raw_Accel[1]) + 32768;	//Converting the Accel_X to 16-bit Variable
	Shifted_Accel_Y = (Raw_Accel[0]) + 32768;	//Converting the Accel_Y to 16-bit Variable
	Shifted_Accel_Z = (Raw_Accel[2]) + 32768;	//Converting the Accel_Z to 16-bit Variable
	#endif
	
	#ifdef KIDSIZE
	Shifted_Gyro_X  = (Raw_Gyro[1]) + 32768;	  //Converting the Gyro_X to 16-bit Variable
	Shifted_Gyro_Y  = (-1*Raw_Gyro[0]) + 32768;	  //Converting the Gyro_Y to 16-bit Variable
	Shifted_Gyro_Z  = (Raw_Gyro[2]) + 32768; 	//Converting the Gyro_Z to 16-bit Variable

	Shifted_Accel_X = (Raw_Accel[1]) + 32768;	//Converting the Accel_X to 16-bit Variable
	Shifted_Accel_Y = (-1*Raw_Accel[0]) + 32768;	//Converting the Accel_Y to 16-bit Variable
	Shifted_Accel_Z = (Raw_Accel[2]) + 32768;	//Converting the Accel_Z to 16-bit Variable
	#endif
	//Preparing Filter's Data for 8Bit UART Buffer
	Shifted_Roll = (Roll + M_PI) * 10000;        	//[ -PI   , +PI   ]
	Shifted_Pitch = (Pitch + M_PI) * 10000;	      //[ -PI/2 , +PI/2 ]
	Shifted_Yaw = (Yaw + M_PI) * 10000;	          //[ -PI   , +PI   ]
	//Preparing Battery's Data for 8Bit UART Buffer
	Shifted_BatteryVoltage = (uint8_t) (BatteryVoltage * 10);
	
	LookupTable[0]  = Shifted_Gyro_X;             //Save 8-bits of the First Part of Gyro_X
	LookupTable[1]  = Shifted_Gyro_X >> 8;	      //Saving 8 bits of the last part of Gyro_X
	LookupTable[2]  = Shifted_Gyro_Y;	            //Save 8-bits of the First Part of Gyro_Y
	LookupTable[3]  = Shifted_Gyro_Y >> 8;	      //Saving 8 bits of the last part of Gyro_Y
	LookupTable[4]  = Shifted_Gyro_Z;	            //Save 8-bits of the First Part of Gyro_Z
	LookupTable[5]  = Shifted_Gyro_Z >> 8;	      //Saving 8 bits of the last part of Gyro_Z
	
	LookupTable[6]  = Shifted_Accel_X;	          //Save 8-bits of the First Part of Accel_X
	LookupTable[7]  = Shifted_Accel_X >> 8;	      //Saving 8 bits of the last part of Accel_X
	LookupTable[8]  = Shifted_Accel_Y;	          //Save 8-bits of the First Part of Accel_Y
	LookupTable[9]  = Shifted_Accel_Y >> 8;	      //Saving 8 bits of the last part of Accel_Y
	LookupTable[10] = Shifted_Accel_Z;	          //Save 8-bits of the First Part of Accel_Z
	LookupTable[11] = Shifted_Accel_Z >> 8;	      //Saving 8 bits of the last part of Accel_Z
	
	LookupTable[12] = Shifted_Roll;	              //Save 8-bits of the First Part of Roll
	LookupTable[13] = Shifted_Roll >> 8;	        //Saving 8 bits of the last part of Roll
	LookupTable[14] = Shifted_Pitch;	            //Save 8-bits of the First Part of Pitch
	LookupTable[15] = Shifted_Pitch >> 8;	        //Saving 8 bits of the last part of Pitch
	LookupTable[16] = Shifted_Yaw;	              //Save 8-bits of the First Part of Yaw
	LookupTable[17] = Shifted_Yaw >> 8;	          //Saving 8 bits of the last part of Yaw
	
	LookupTable[30] = Switch1_State + Switch2_State + Switch3_State + Switch4_State;
	
	LookupTable[50] = Shifted_BatteryVoltage;
	
	LookupTable[60] = Calibrated;
	LookupTable[61] = EERead_GX_OFF;
	LookupTable[62] = EERead_GY_OFF;
	LookupTable[63] = EERead_GZ_OFF;
	LookupTable[64] = EERead_AX_OFF;
	LookupTable[65] = EERead_AY_OFF;
	LookupTable[66] = EERead_AZ_OFF;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Struct Initializer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Struct_Initializer(USBLineTypeDef* stct)
{
	stct->InputData = 0;
	stct->Length = 0;
	stct->CurrentState = 0;
	stct->BufferCurrentIndex = 0;
	stct->BufferLastIndex = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debug function
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Debugger(void)
{
	if((huart1.gState == HAL_UART_STATE_BUSY_TX) && ((huart1.Instance->CR1 & USART_CR1_TXEIE) == RESET) && (huart1.TxXferCount == huart1.TxXferSize))
	{
		SET_BIT(huart1.Instance->CR1, USART_CR1_TXEIE);
	}
	if((huart3.gState == HAL_UART_STATE_BUSY_TX) && ((huart3.Instance->CR1 & USART_CR1_TXEIE) == RESET) && (huart3.TxXferCount == huart3.TxXferSize))
	{
		SET_BIT(huart3.Instance->CR1, USART_CR1_TXEIE);
	}
	if((huart5.gState == HAL_UART_STATE_BUSY_TX) && ((huart5.Instance->CR1 & USART_CR1_TXEIE) == RESET) && (huart5.TxXferCount == huart5.TxXferSize))
	{
		SET_BIT(huart5.Instance->CR1, USART_CR1_TXEIE);
	}
	if((huart6.gState == HAL_UART_STATE_BUSY_TX) && ((huart6.Instance->CR1 & USART_CR1_TXEIE) == RESET) && (huart6.TxXferCount == huart6.TxXferSize))
	{
		SET_BIT(huart6.Instance->CR1, USART_CR1_TXEIE);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU OffLine Calibration
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Calibration(void)
{
	int16_t count = 0;
	while(EE_Format() != HAL_OK);
	int16_t temp[6];
	int32_t sum[6] = {0, 0, 0, 0, 0, 0};
	
	while(count < 4000)
	{
		MPU9250_ReadData(&hspi1, IMU_Buffer, MPU9250_ACCEL_XOUT_H, 14);	//Read Gyro and Accel data
		temp[0] = IMU_Buffer[0] << 8;
		temp[0] |= IMU_Buffer[1];
		temp[1] = IMU_Buffer[2] << 8;
		temp[1] |= IMU_Buffer[3];
		temp[2] = IMU_Buffer[4] << 8;
		temp[2] |= IMU_Buffer[5];
		temp[3] = IMU_Buffer[8] << 8;
		temp[3] |= IMU_Buffer[9];
		temp[4] = IMU_Buffer[10] << 8;
		temp[4] |= IMU_Buffer[11];
		temp[5] = IMU_Buffer[12] << 8;
		temp[5] |= IMU_Buffer[13];
		sum[0] += temp[0];
		sum[1] += temp[1];
		sum[2] += temp[2];
		sum[3] += temp[3];
		sum[4] += temp[4];
		sum[5] += temp[5];
		CHECK++;
		count++;
		HAL_Delay(2);
	}
	AX_Off = (sum[0] / count);
	AY_Off = (sum[1] / count);
	AZ_Off = (sum[2] / count);
	GX_Off = (sum[3] / count);
	GY_Off = (sum[4] / count);
	GZ_Off = (sum[5] / count);
	set_offset();
}
void set_offset(void)
{
//--------------------------------------------	
	EERead_GX_OFF = -1 * GX_Off;
	EERead_GY_OFF = -1 * GY_Off;
	EERead_GZ_OFF = -1 * GZ_Off;
	
	EERead_AX_OFF = -1 * AX_Off;
	EERead_AY_OFF = -1 * AY_Off;
	EERead_AZ_OFF = 2048 - AZ_Off;

//___________________Write To EEPROM___________________
	EE_WriteVariable(0, EERead_GX_OFF);
	EE_WriteVariable(1, EERead_GY_OFF);
	EE_WriteVariable(2, EERead_GZ_OFF);	
	EE_WriteVariable(3, EERead_AX_OFF);
	EE_WriteVariable(4, EERead_AY_OFF);
	EE_WriteVariable(5, EERead_AZ_OFF);

	Calibrated = 10;	//10 means Caliration is Finished.	
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
