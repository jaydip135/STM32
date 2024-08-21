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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void read_keypad (void)
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 1\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 2\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 3\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: A\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 4\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 5\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 6\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: B\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}


	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 7\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 8\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 9\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: C\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}


	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: *\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: 0\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: #\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		uint8_t *data = (uint8_t *)"Key: D\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
	}

}

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  read_keypad ();
	  HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  PeriphClkInit.PLLI2S.PLLI2SMUL = RCC_PLLI2S_MUL10;
  PeriphClkInit.PLLI2S.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLL3CLK, RCC_MCODIV_1);

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_HSE_PREDIV2_CONFIG(RCC_HSE_PREDIV2_DIV5);

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_CONFIG(RCC_PLLI2S_MUL10);

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, R3_Pin|R2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C4_Pin C3_Pin C2_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R4_Pin */
  GPIO_InitStruct.Pin = R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R2_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
