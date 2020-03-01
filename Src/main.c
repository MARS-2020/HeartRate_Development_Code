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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "HR.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//austins variables



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//make func defs
uint8_t HR_INIT(void); //Start Algorithm
uint8_t HR_READ(uint8_t * received_data); //

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

//Set 32664 to open in application mode
//set RSTN high
  /*
  HAL_GPIO_WritePin(GPIOB, HR_MFIO_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, HR_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(3);
  HAL_GPIO_WritePin(GPIOB, HR_MFIO_Pin, GPIO_PIN_SET); //should MFIO be already low? cant find in datasheet
  HAL_Delay(7);
  HAL_GPIO_WritePin(GPIOB, HR_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);	//will have to replace with timer later
*/
  //HOST CONFIGURES MFIO AS INPUT INTERRUPT PIN ??????

//Call HR initialization code after putting 32664 into application mode
 /*if(HR_INIT() == 0) //equals 0 means initialization failed - do something? make while loop that runs until it isnt 0?
 {
	 asm("NOP");
 }
*/

//uint8_t received_data[24];
//uint8_t err_flag_read;
	uint8_t arr_1_2[3] = {0x10, 0x00, 0x03};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*
	  err_flag_read = HR_READ(received_data);
	  if(err_flag_read == 1)
	  {
		  asm("NOP");
	  }

	  uint16_t heartrate  = {received_data[17], received_data[18]};
	  uint8_t  HR_conf    =  received_data[19];
	  uint16_t spo2 	    = {received_data[20], received_data[21]};
	  uint8_t  alg_state  =  received_data[22];
	  uint8_t  alg_status =  received_data[23];

	 // if((alg_state == 0x03) && (alg_status == 0x00))
	  //{
	  //	nop;
	  //}
*/

		HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_2, sizeof(arr_1_2), 1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 2097-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HR_MFIO_Pin|HR_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : HR_MFIO_Pin HR_RESET_Pin */
  GPIO_InitStruct.Pin = HR_MFIO_Pin|HR_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


uint8_t HR_INIT()
{
	//HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	//HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)

	//1.2 - set output mode to sensor
	uint8_t arr_1_2[3] = {0x10, 0x00, 0x03};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_2, sizeof(arr_1_2), 1000);
	uint8_t receive_buff;
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.3 - Set sensor hub interrupt threshold
	uint8_t arr_1_3[3] = {0x10, 0x01, 0x0F};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_3, sizeof(arr_1_3), 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.4 - Enable AGC
	uint8_t arr_1_4[3] = {0x52, 0x00, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_4, sizeof(arr_1_4), 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.6 - Enable AFE
	uint8_t arr_1_6[3] = {0x44, 0x03, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_6, sizeof(arr_1_6), 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.7 - Enable HR/SpO2 Algorithm
	uint8_t arr_1_7[3] = {0x52, 0x02, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_7, sizeof(arr_1_7), 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	if(receive_buff != 0x00)
	{
		return 1;
	}

	return 0;
}


uint8_t HR_READ(uint8_t * receive_data)
{
	//2.1 - Data finished when bit3 of AA0000 is full (DATARDYINT)
	uint8_t arr_2_1[2] = {0x00, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_2_1, sizeof(arr_2_1), 1000);
	uint8_t receive_hub[2];
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, receive_hub, sizeof(receive_hub), 1000);
	if(receive_hub[0] != 0x00)   //failed read
	{
		return 1;
	}

	while(receive_hub[1] != 0x08)
	{
		HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_2_1, sizeof(arr_2_1), 1000);
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, receive_hub, sizeof(receive_hub), 1000);
	}

	//2.2 - get number of samples in FIFO
	uint8_t arr_2_2[2] = {0x12, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_2_2, sizeof(arr_2_2), 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, receive_hub, sizeof(receive_hub), 1000);
	if(receive_hub[0] != 0x00)	//failed read
	{
		return 1;
	}

	//2.3 - read all samples from FIFO
	uint8_t arr_2_3[2] = {0x12, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_2_3, sizeof(arr_2_3), 1000);
	//uint8_t receive_data[24];
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, receive_data, sizeof(receive_data), 1000);

	return 0;
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
