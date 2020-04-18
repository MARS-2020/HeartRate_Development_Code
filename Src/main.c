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
#include "stm32l0xx.h"
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

/* USER CODE BEGIN PV */
//uint8_t received_data[22] = {0};

uint16_t heartrate  = 0;
uint8_t  HR_conf    = 0;
uint16_t spo2  		= 0;
uint8_t  alg_state  = 0;
uint8_t  alg_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void HR_APP_MODE(void); //Put component in application mode
void HR_MFIO_SET(void); //set up MFIO as an interrupt
uint8_t HR_INIT(void); //Start Algorithm
uint8_t HR_READ(uint8_t * receive_data);
uint8_t HR_SHUTDOWN(void);
void EXTI2_TSC_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


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
  /* USER CODE BEGIN 2 */

  	//if(HR_SHUTDOWN() == 1)
  	//{
  	//	asm("NOP");
  	//}

  	//HAL_Delay(100);

	HR_APP_MODE(); //call function to put module in application mode
	HR_MFIO_SET();

	if(HR_INIT() == 1) //equals 1 means initialization failed - do something? make while loop that runs until it isnt 0?
	 {
 		asm("NOP");
	 }



  	//if(HR_SHUTDOWN() == 1)
  	//{
  	//	asm("NOP");
  	//}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //uint8_t arr_1_2[3] = {0x10, 0x00, 0x03};
//	  uint8_t arr = 0xA0;
	 // uint16_t writeAddr = 0xAA;
	 // HAL_I2C_Master_Transmit(&hi2c1, writeAddr, &arr, 1, 10);
	 // HAL_Delay(5);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HR_MFIO_GPIO_Port, HR_MFIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HR_RESET_GPIO_Port, HR_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : HR_MFIO_Pin */
  GPIO_InitStruct.Pin = HR_MFIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HR_MFIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HR_RESET_Pin */
  GPIO_InitStruct.Pin = HR_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HR_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HR_APP_MODE()
{
	  //HAL_Delay(10);
	  HAL_GPIO_WritePin(HR_MFIO_GPIO_Port, HR_MFIO_Pin, GPIO_PIN_RESET); //set MFIO LOW
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(HR_RESET_GPIO_Port, HR_RESET_Pin, GPIO_PIN_RESET); //set RSTN low for 10ms
	  HAL_Delay(3);
	  HAL_GPIO_WritePin(HR_MFIO_GPIO_Port, HR_MFIO_Pin, GPIO_PIN_SET); //set MFIO high while RSTN is low
	  //GPIOB->BSRR = HR_MFIO_Pin;
	  HAL_Delay(5);
	  HAL_GPIO_WritePin(GPIOB, HR_RESET_Pin, GPIO_PIN_SET); //return RSTN to its high state
	  HAL_Delay(1000);	//wait 1 second (in app mode after 50ms) - will have to replace with timer later

	  //GPIOB->MODER &= ~(GPIO_MODER_MODE4); //Set MFIO to be input so it can act as an interrupt
}



void HR_MFIO_SET()
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = HR_MFIO_Pin;
	  //GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(HR_MFIO_GPIO_Port, &GPIO_InitStruct);

	  //enable the interrupt
	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

void EXTI4_15_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(HR_MFIO_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	if(GPIO_Pin == HR_MFIO_Pin)
	{
		//do stuff now that the interrupt was set
		uint8_t received_data[22];
		uint8_t err_flag_read;
		err_flag_read = HR_READ(received_data);

		if(err_flag_read == 1)
		  {
			  asm("NOP");
		  }


		//uint16_t heartrate  = ((((uint16_t) received_data[18]) << 8) | (received_data[19])) / 10;
		//uint8_t  HR_conf    =  received_data[20];
		//uint16_t spo2  = ((((uint16_t) received_data[21]) << 8) | (received_data[22])) / 10;
		//uint8_t  alg_state  =  received_data[23];
		//uint8_t  alg_status =  received_data[24];
	}

	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

uint8_t HR_INIT()
{
	//1.1 - configure default spo2 alg values
	//uint8_t arr_1_1[15] = {0x50, 0x02, 0x0B, 0x00, 0x02, 0x6F, 0x60, 0xFF, 0xCB, 0x1D, 0x12, 0x00, 0xAB, 0xF3, 0x7B};
	uint16_t writeAddr = 0xAA;
	uint16_t readAddr = 0xAB;
	//HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_1_1, sizeof(arr_1_1), 1000);
	uint8_t receive_buff = -1;
	//HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_buff, sizeof(receive_buff), 1000);
	//if(receive_buff != 0x00)
	//{
	//	return 1;
	//}


	//1.2 - set output mode to sensor
	uint8_t arr_1_2[3] = {0x10, 0x00, 0x03};
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_1_2, sizeof(arr_1_2), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	receive_buff = -3;
	//HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_buff, sizeof(receive_buff), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_buff == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.3 - Set sensor hub interrupt threshold
	uint8_t arr_1_3[3] = {0x10, 0x01, 0x0F};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_3, sizeof(arr_1_3), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	receive_buff = -1;
	//HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_buff == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.4 - Enable AGC
	uint8_t arr_1_4[3] = {0x52, 0x00, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_4, sizeof(arr_1_4), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	receive_buff = -1;
	//HAL_Delay(20);

	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_buff == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}

	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.6 - Enable AFE
	uint8_t arr_1_6[3] = {0x44, 0x03, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_6, sizeof(arr_1_6), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	receive_buff = -1;
	//HAL_Delay(100);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_buff == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
	if(receive_buff != 0x00)
	{
		return 1;
	}

	//1.7 - Enable HR/SpO2 Algorithm
	uint8_t arr_1_7[3] = {0x52, 0x02, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, 0xAA, arr_1_7, sizeof(arr_1_7), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	receive_buff = -1;
	HAL_Delay(50);
	HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_buff == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_buff, sizeof(receive_buff), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
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
	uint16_t writeAddr = 0xAA;
	uint16_t readAddr = 0xAB;
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_2_1, sizeof(arr_2_1), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	uint8_t receive_hub[2];
	HAL_I2C_Master_Receive(&hi2c1, readAddr, receive_hub, sizeof(receive_hub), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	if(receive_hub[0] != 0x00)   //failed read
	{
		return 1;
	}

	/*
	while(receive_hub[1] != 0x08)
	{
		HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_2_1, sizeof(arr_2_1), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
		HAL_I2C_Master_Receive(&hi2c1, readAddr, receive_hub, sizeof(receive_hub), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
*/

	if(receive_hub[1] == 0x08)
	{
		//2.2 - get number of samples in FIFO
		uint8_t arr_2_2[2] = {0x12, 0x00};
		HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_2_2, sizeof(arr_2_2), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
		HAL_I2C_Master_Receive(&hi2c1, readAddr, receive_hub, sizeof(receive_hub), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
		if(receive_hub[0] != 0x00)	//failed read
		{
			return 1;
		}

		//2.3 - read all samples from FIFO
		uint8_t arr_2_3[2] = {0x12, 0x01};
		HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_2_3, sizeof(arr_2_3), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
		HAL_I2C_Master_Receive(&hi2c1, readAddr, receive_data, 22, 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);

		/*
		if(receive_data[18] == 0x03)
		{
			uint16_t heartrate  = ((((uint16_t) receive_data[13]) << 8) | (receive_data[14])) / 10;
			uint8_t  HR_conf    =  receive_data[15];
			uint16_t spo2  = ((((uint16_t) receive_data[16]) << 8) | (receive_data[17])) / 10;
			uint8_t  alg_state  =  receive_data[18];
			uint8_t  alg_status =  receive_data[19];

			return 0;
		}
*/


		heartrate  = ((((uint16_t) receive_data[13]) << 8) | (receive_data[14])) / 10;
		HR_conf    =  receive_data[15];
		spo2  = ((((uint16_t) receive_data[16]) << 8) | (receive_data[17])) / 10;
		alg_state  =  receive_data[18];
		alg_status =  receive_data[19];

		return 1;
	}

	return 1;
}


uint8_t HR_SHUTDOWN()
{
	/*
	//3.1 - Disable the AFE
	uint8_t arr_3_1[3] = {0x44, 0x03, 0x00};
	uint16_t writeAddr = 0xAA;
	uint16_t readAddr = 0xAB;
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_3_1, sizeof(arr_3_1), 1000);
	uint8_t receive_off = -1;
	HAL_Delay(100);
	HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_off, sizeof(receive_off), 1000);
	while(receive_off == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_off, sizeof(receive_off), 1000);
	}
	if(receive_off != 0x00)   //failed read
	{
		return 1;
	}

	//3.3 - Disable the Algorithm
	uint8_t arr_3_3[3] = {0x52, 0x02, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_3_3, sizeof(arr_3_3), 1000);
	HAL_Delay(50);
	HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_off, sizeof(receive_off), 1000);
	while(receive_off == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_off, sizeof(receive_off), 1000);
	}
	if(receive_off != 0x00)   //failed read
	{
		return 1;
	}
*/

	//SOFT RESET SENSOR

	uint16_t writeAddr = 0xAA;
	uint16_t readAddr = 0xAB;
	uint8_t receive_off = -1;
	uint8_t arr_x_x[4] = {0x40, 0x03, 0x09, 0x40};
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_x_x, sizeof(arr_x_x), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	//HAL_Delay(50);
	HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_off, sizeof(receive_off), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_off == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_off, sizeof(receive_off), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
	if(receive_off != 0x00)   //failed read
	{
		return 1;
	}

	//soft reset agc

	uint8_t arr_x_y[3] = {0x01, 0x00, 0x02};
	HAL_I2C_Master_Transmit(&hi2c1, writeAddr, arr_x_y, sizeof(arr_x_y), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	//HAL_Delay(50);
	HAL_I2C_Master_Receive(&hi2c1, readAddr, &receive_off, sizeof(receive_off), 1000);
	while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	while(receive_off == 0xFE)
	{
		HAL_I2C_Master_Receive(&hi2c1, 0xAB, &receive_off, sizeof(receive_off), 1000);
		while(((&hi2c1) -> State) != HAL_I2C_STATE_READY);
	}
	if(receive_off != 0x00)   //failed read
	{
		return 1;
	}



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
