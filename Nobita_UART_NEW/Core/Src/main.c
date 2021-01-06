/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/*
 * MOTOR_PWM = PA10 = D2
 * MOTOR_PWM2 = PA9 = D8
 * MOTOR_DIRECTION1 = PB4 = D5
 * MOTOR_DIRECTION2 = PB5 = D4
 * MOTOR2_DIRECTION1 = PB8 = D15
 * MOTOR2_DIRECTION2 = PB9 = D14
 * TIM4_CH1 = PB6 = D12
 * TIM4_CH2 = PB7 = D11
 * TIM2_CH1 = PA0 =
 * TIM2_CH2 = PA1 =
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t direction_l = 1;
uint8_t direction_r = 0;
float cycleTime = 0.1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t myTxData[4] = "OK\r\n";
uint8_t myRxData[16];
uint8_t testRxData[27]; // 21+\r+\n
//int RxSize = sizeof(testRxData) / sizeof(testRxData[0]);
int i = 0;

char buffer[50];
float wl,wr,dl,dr;
float _wl,_wr,_dl,_dr;
long encoder_value_l = 0;
long encoder_value_r = 0;
uint32_t en_l = 0;
float _mps_l = 0.0;
float _mps_r = 0.0;
float rpm_value_l = 0.00;
float rpm_value_r = 0.00;
uint8_t MSG1[50];
uint8_t MSG1_length;
uint8_t MSG2[50];
uint8_t MSG2_length;
uint8_t MSG3[50];
uint8_t MSG3_length;
//uint8_t MSGTEST[50];
//uint8_t MSGTEST_length;
uint8_t MSGOUTPUT[50];
uint8_t MSGOUTPUT_length;
uint8_t RX2_Char[2];
uint8_t Char_Buffer[20];
uint8_t Char_Buffer_length = 0;
uint8_t Char_Buffer_isRecieving = 0;
float cumError_l = 0.0;
float cumError_r = 0.0;
float error_l = 0.0;
float error_r = 0.0;
float error_p_l = 0.0;
float error_p_r = 0.0;
float kd_l=0;
float kd_r=0;
float ki_l=0.000001;
float ki_r=0.000001;

float kp_l=100;
float kp_r=100;

float ErrorI_l = 0.0;
float ErrorI_r = 0.0;

int pwm_l = 0;
int pwm_r = 0;

float output_l = 0.0;
float output_r = 0.0;
float rateError_l = 0.0;
float rateError_r = 0.0;
float testval = 599;
float mps_l = 0.0;
float mps_r = 0.0;
float wheel_diameter = 0.108;
//for test begin
long test_encoder=0;
//for test end
void setSpeed(){
	//HAL_GPIO_TogglePin(GPIO)
	HAL_GPIO_WritePin(MOTOR_DIRECTION1_GPIO_Port,MOTOR_DIRECTION1_Pin,_dl);
	HAL_GPIO_WritePin(MOTOR_DIRECTION2_GPIO_Port,MOTOR_DIRECTION2_Pin,!_dl);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_l);
	//HAL_GPIO_WritePin(MOTOR_PWM_GPIO_Port,MOTOR2_PWM_Pin,direction_r);
	HAL_GPIO_WritePin(MOTOR2_DIRECTION1_GPIO_Port,MOTOR2_DIRECTION1_Pin,_dr);
	HAL_GPIO_WritePin(MOTOR2_DIRECTION2_GPIO_Port,MOTOR2_DIRECTION2_Pin,!_dr);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

//	htim4.Instance->CCR1=wr*1000;
	htim4.Instance->CCR1=pwm_r;
	htim4.Instance->CCR2=pwm_l;

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	//HAL_Delay(1);
//	__HAL_TIM_SET_AUTORELOAD(&htim4, wl*1000.0);

}
double convert2mps(int en_val){
	double output;
	output = en_val * 0.000015773 / 0.1;
	return output;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//int a = log(EN_L);
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, testRxData, 27);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);
  //necessary for encoder init begin
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  //necessary for encoder init end
  /* USER CODE END 2 */
  pwm_l = 0;
  pwm_r = 0;
  cumError_l=0;
  cumError_r=0;
  ErrorI_l=0;
  ErrorI_r=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|MOTOR2_PWM_Pin|MOTOR_PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_DIRECTION1_Pin|MOTOR_DIRECTION2_Pin|MOTOR2_DIRECTION1_Pin|MOTOR2_DIRECTION2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 MOTOR2_PWM_Pin MOTOR_PWM_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|MOTOR2_PWM_Pin|MOTOR_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIRECTION1_Pin MOTOR_DIRECTION2_Pin MOTOR2_DIRECTION1_Pin MOTOR2_DIRECTION2_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIRECTION1_Pin|MOTOR_DIRECTION2_Pin|MOTOR2_DIRECTION1_Pin|MOTOR2_DIRECTION2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int buffer_length;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */

  //set variables
  sscanf(testRxData,"%f,%f,%f,%f",&wl,&wr,&dl,&dr);
  buffer_length = sprintf(buffer, "%f,%f,%.1f,%.1f\r\n",wl,wr,dl,dr);
  _wl = wl;
  _wr = wr;
  _dl = dl;
  _dr = dr;
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  /*
  encoder_value_l = htim3.Instance->CNT - 30000;
  encoder_value_r = htim5.Instance->CNT - 30000;
  htim3.Instance->CNT = 30000;
  htim5.Instance->CNT = 30000;

  //RPM VALUE SHOULD BE ((EN_VAL*60)/PPR)/GEAR_RATIO
  rpm_value_l = (encoder_value_l*60.0)/46000.0;
  rpm_value_r = (encoder_value_r*60.0)/46000.0;
  ms_l = (rpm_value_l/60)*wheel_diameter;
  MSG1_length = sprintf(MSG1,"EncoderL = %d RPM = %.2f\r\nOUTPUT_L=%f\r\n", encoder_value_l, rpm_value_l,output_l);
  MSG2_length = sprintf(MSG2,"EncoderR = %d RPM = %.2f\r\nOUTPUT_R=%f\r\n", encoder_value_r, rpm_value_r,output_r);
  */
  HAL_UART_Transmit(&huart2, buffer, buffer_length, 100);}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if(htim->Instance == TIM2){
	  //HAL_UART_Transmit(&huart2, "hi\r\n", 4, 100);

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  encoder_value_l = htim3.Instance->CNT - 30000;
	  encoder_value_r = htim5.Instance->CNT - 30000;
	  /*
	  if(test_encoder<0){
		  test_encoder*=-1;
	  }
	  */
	  test_encoder += encoder_value_l;
	  htim3.Instance->CNT = 30000;
	  htim5.Instance->CNT = 30000;

	  //RPM VALUE SHOULD BE ((EN_VAL*60)/PPR)/GEAR_RATIO
	  /*
	  rpm_value_l = (encoder_value_l*60.0)/46000.0;
	  rpm_value_r = (encoder_value_r*60.0)/46000.0;
	  ms_l = (rpm_value_l/60)*wheel_diameter;
	  */
	  mps_l = convert2mps(encoder_value_l);
	  mps_r = convert2mps(encoder_value_r);
	  MSG1_length = sprintf(MSG1,"EncoderL = %d MPS_L = %.2f m/s\r\n", encoder_value_l, mps_l);
	  MSG2_length = sprintf(MSG2,"EncoderR = %d MPS_R = %.2f m/s\r\n", encoder_value_r, mps_r);
	  /*
	  MSGTEST_length = sprintf(MSGTEST,"\nTEST = %d\r\n",test_encoder);
	  */
	  //MSGOUTPUT AFTER PID
	  if(mps_l<0){
		  _mps_l=mps_l*-1;
	  }
	  else{
		  _mps_l=mps_l;
	  }
	  if(mps_r<0){
		  _mps_r=mps_r;
	  }
	  else{
		  _mps_r=mps_r;
	  }

	  error_l = _wl - _mps_l;
	  error_r = _wr - _mps_r;

	  cumError_l += error_l;// * cycleTime;
	  //rateError_l += (error_l - error_p_l)/cycleTime;
	  //output_l = kp_l*error_l + ki_l*cumError_l + kd_l*rateError_l;
	  //error_p_l = error_l;

	  cumError_r += error_r;// * cycleTime;
	  //rateError_r += (error_r - error_p_r)/cycleTime;
	  //output_r = kp_r*error_r + ki_r*cumError_r + kd_r*rateError_r;
	  //error_p_r = error_r;

	  //pwm_l += (kp_l*error_l);
	  //pwm_r += (kp_r*error_r);
	  ErrorI_l = ki_l*cumError_l;
	  ErrorI_r = ki_r*cumError_r;
	  pwm_l += kp_l*error_l+ErrorI_l;
	  pwm_r += kp_r*error_r+ErrorI_r;

	  if(pwm_l < 0) pwm_l = 0;
	  if(pwm_l < 50 && error_l < 0.0001) pwm_l = 0;
	  if(pwm_l > 400) pwm_l = 400;
	  if(_wl < 0.0001) pwm_l = 0;

	  if(pwm_r < 0) pwm_r = 0;
	  if(pwm_r < 50 && error_r < 0.0001) pwm_r = 0;
	  if(pwm_r > 400) pwm_r = 400;
	  if(_wr < 0.0001) pwm_r = 0;

	  //USE OUTPUT_L


	  //USE OUTPUT_R
	  //MSGOUTPUT_length = sprintf(MSGOUTPUT, "Output_l = %f\tOutput_r = %f\r\npwm_l = %d\r\n",output_l,output_r,pwm_l);
	  MSG1_length = sprintf(MSG1, "Target_wl=%.2f m/s\tEnc_l=%.2f m/s\tpwm_r=%d\r\n",_wl,mps_l,pwm_l);
	  MSG2_length = sprintf(MSG2, "Target_wr=%.2f m/s\tEnc_r=%.2f m/s\tpwm_r=%d\r\n",_wr,mps_r,pwm_r);
	  MSG3_length = sprintf(MSG3, "ErrorI_l = %.2f\tErrorI_r = %.2f\r\n",ErrorI_l,ErrorI_r);
	  //USE OUTPUT_R
	  //previousTime = currentTime;
	  HAL_UART_Transmit(&huart2, MSG1, MSG1_length, 50);
	  HAL_UART_Transmit(&huart2, MSG2, MSG2_length, 50);
	  HAL_UART_Transmit(&huart2, MSG3, MSG3_length, 50);
//	  HAL_UART_Transmit(&huart2, MSGOUTPUT, MSGOUTPUT_length, 100);

	  setSpeed();

  }

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}
/*
float computePID(struct PIDValues wheel){  //float _input, float _setPoint
  //currentTime = millis();
  //elapsedTime = (float)(currentTime-elapsedTime);
  error_l = _wl - encoder_value_l;
  cumError_l += _wl * cycleTime;
  rateError_l += (error_l - error_p_l)/cycleTime;
  output_l = kp*wheel.error + ki*cumError_l + kd*rateError_l;
  error_p_l = error_l;
  //previousTime = currentTime;
  return output_l;
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
