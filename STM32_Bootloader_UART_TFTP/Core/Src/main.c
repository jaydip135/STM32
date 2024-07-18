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
#include "lwip.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tftpserver.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BLOCK_SIZE          ( 1024 )                  //1KB
#define ETX_APP_START_ADDRESS   0x08020000
#define MAJOR 1
#define MINOR 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t BL_Version[2] = { MAJOR, MINOR };
uint16_t application_size = 0;
uint16_t application_write_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void tftp_update();
static void Firmware_Update(void);
static void goto_application(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//led counter
uint16_t led_blink_cnt = 0;

//variables for jump
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

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
  MX_USART1_UART_Init();
  MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(4000);

  uint8_t *data = (uint8_t *)"Bootloader v1:0 Started!!!\n";
  CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
  HAL_Delay(1000);
  uint8_t *data1 = (uint8_t *)"checking user button...\r\n";
  CDC_Transmit_FS((uint8_t *)data1, strlen((char *)data1));
  HAL_Delay(1000);
  if (HAL_GPIO_ReadPin(DP_0_GPIO_Port, DP_0_Pin) == 0x00)
  {
	  uint8_t *data3 = (uint8_t *)"Update From TFTP Server\r\n";
	  CDC_Transmit_FS((uint8_t *)data3, strlen((char *)data3));
	  HAL_Delay(1000);
	  tftp_update();
	  HAL_Delay(1000);
	  uint8_t *data4 = (uint8_t *)"start tftp server...\r\n";
	  CDC_Transmit_FS((uint8_t *)data4, strlen((char *)data4));
	  IAP_tftpd_init();
	  while(1)
	  {
		  MX_LWIP_Process();
	  }
  }
  else if (HAL_GPIO_ReadPin(DP_1_GPIO_Port, DP_1_Pin) == 0x00)
		{
	  	  uint8_t *data2 = (uint8_t *)"Update From UART\r\n";
	  	  CDC_Transmit_FS((uint8_t *)data2, strlen((char *)data2));
	  	  Firmware_Update();

	  	  goto_application();
		}
  else{
	  	  goto_application();
  	  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1)
  		{

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DP_0_Pin */
  GPIO_InitStruct.Pin = DP_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DP_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DP_1_Pin DP_2_Pin */
  GPIO_InitStruct.Pin = DP_1_Pin|DP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void tftp_update()
{
	if (HAL_GPIO_ReadPin(DP_2_GPIO_Port, DP_2_Pin) == 0x01)
	{
		/* Check if valid stack address (RAM address) then jump to user application */
		if (((*(__IO uint32_t*)USER_FLASH_FIRST_PAGE_ADDRESS) & 0x2FFF0000 ) == 0x20010000)
		{
			uint8_t *data = (uint8_t *)"valid stack address! jump to application...\r\n";
			CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));

			/* Jump to user application */
			JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + 4);
			Jump_To_Application = (pFunction) JumpAddress;
			/* Initialize user application's Stack Pointer */
			HAL_RCC_DeInit();
			HAL_DeInit();
			__set_MSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
			SysTick->CTRL = 0;
			SysTick->LOAD = 0;
			SysTick->VAL = 0;
			Jump_To_Application();
			/* do nothing */
			while(1);
		}
		else
		{
			uint8_t *data1 = (uint8_t *)"Invalid stack address! Start TFTP Server\r\n";
			CDC_Transmit_FS((uint8_t *)data1, strlen((char *)data1));
		}
	}
}


static int UART_Write_Loop( void )
{
  char tx = 'g';
  char rx = '0';
  HAL_StatusTypeDef ex;
  int ret = 0;
  int count = 0;
  while(1)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t *)&tx, 1, HAL_MAX_DELAY);
	  ex = HAL_UART_Receive(&huart1, (uint8_t *)&rx, 1, 10);
	  if( ( ex == HAL_OK ) && ( rx == 'r' ) )
	  {
		  uint8_t *data = (uint8_t *)"Firmware Update Started\r\n";
		  CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
		  ret = 1;
		  break;
	  }
	  if( count == 100 )
	  {
		  uint8_t *data = (uint8_t *)"No Data Received for Firmware Update\r\n";
		  CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
		  break;
	  }
	  count++;
	  HAL_Delay(20);              //20ms delay
  }
  return ret;
}

/**
  * @brief Write data to the Application's actual flash location.
  * @param data data to be written
  * @param data_len data length
  * @is_first_block true - if this is first block, false - not first block
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data,
                                        uint16_t data_len, bool is_first_block )
{
  HAL_StatusTypeDef ret;
  do
  {
    HAL_FLASH_Unlock();
    if( is_first_block )
    {
      uint8_t *data = (uint8_t *)"Erasing the Flash memory...\r\n";
      CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError;
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = ETX_APP_START_ADDRESS;
      EraseInitStruct.NbPages       = 64;
      HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
      application_write_idx = 0;
    }

    for(int i = 0; i < data_len/2; i++)
    {
      uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD,
                               (ETX_APP_START_ADDRESS + application_write_idx ),
                               halfword_data
                             );
      if( ret == HAL_OK )
      {
        application_write_idx += 2;
      }
    }
    if( ret != HAL_OK )
    {
      break;
    }
    HAL_FLASH_Lock();
  }while( false );
  return ret;
}

/**
  * @brief Check for Firmware Update and update the Firmware
  * @retval None
  */
static void Firmware_Update(void)
{
  uint8_t xx,yy;
  uint8_t x = 'x';
  uint8_t y = 'y';
  HAL_StatusTypeDef ex = HAL_OK;
  uint16_t current_app_size = 0;
  uint16_t i = 0;
  uint8_t block[MAX_BLOCK_SIZE] = { 0 };

  do
  {
    if( UART_Write_Loop() != 0 )
    {
      HAL_UART_Transmit(&huart1, &y, 1, HAL_MAX_DELAY);
      ex = HAL_UART_Receive(&huart1, &yy, 1, 5000);

      HAL_UART_Transmit(&huart1, &x, 1, HAL_MAX_DELAY);
      ex = HAL_UART_Receive(&huart1, &xx, 1, 5000);

      application_size = yy | (xx << 8);
      uint8_t buffer[50]; // Buffer to hold the formatted string
      sprintf((char *)buffer, "Application Size = %d bytes\r\n", application_size);
      CDC_Transmit_FS(buffer, strlen((char *)buffer));
      while(1)
      {
        if( ( i == MAX_BLOCK_SIZE ) || ( current_app_size >= application_size) )
        {
          uint8_t buffer[50]; // Buffer to hold the formatted string
          sprintf((char *)buffer, "Received Block[%d]\r\n", current_app_size / MAX_BLOCK_SIZE);
          CDC_Transmit_FS(buffer, strlen((char *)buffer));

          ex = write_data_to_flash_app(block, MAX_BLOCK_SIZE, (current_app_size <= MAX_BLOCK_SIZE) );

          if( ex != HAL_OK )
          {
            break;
          }

          memset(block, 0,MAX_BLOCK_SIZE);
          i = 0;
        }

        if( current_app_size >= application_size)
        {
          ex = HAL_OK;
          break;
        }

        HAL_UART_Transmit(&huart1, &y, 1, HAL_MAX_DELAY);
        ex = HAL_UART_Receive(&huart1, &yy, 1, 5000);

        HAL_UART_Transmit(&huart1, &x, 1, HAL_MAX_DELAY);
        ex = HAL_UART_Receive(&huart1, &xx, 1, 5000);

        block[i++] = yy;
        block[i++] = xx;
        current_app_size += 2;
      }
    }
  }
  while( false );

  if( ex != HAL_OK )
  {
    while(1);
  }
}

static void goto_application(void)
{

	if (((*(__IO uint32_t*)USER_FLASH_FIRST_PAGE_ADDRESS) & 0x2FFF0000 ) == 0x20010000)
	{
		uint8_t *data = (uint8_t *)"Gonna Jump to Application...\n";
		CDC_Transmit_FS((uint8_t *)data, strlen((char *)data));

		void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08020000 + 4U)));
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET );    //Green LED OFF

		/* Reset the Clock */
		HAL_RCC_DeInit();
		HAL_DeInit();
		__set_MSP(*(volatile uint32_t*) 0x08020000);
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;

		/* Jump to application */
		app_reset_handler();    //call the app reset handler
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
