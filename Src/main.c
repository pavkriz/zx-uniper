/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include <zx_rd_wr_interrupt.h>
#include "op_sniff_loop.h"
#include "mxconstants.h"
#include "stm32f7xx_hal_conf.h"
#include "pintest.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd1;
HAL_SD_CardInfoTypedef SDCardInfo1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void __attribute__((section(".fast_code"))) EXTI4_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
}

void __attribute__((section(".fast_code"))) EXTI9_5_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
}

#define UART_LEN 1
uint8_t UART_Rx_data[UART_LEN];

//
////Interrupt callback routine
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART2) {
//		ide_bytes_received_total += UART_LEN;
//
//		// copy byte to ide buffer
//		for (int i = 0; i < UART_LEN; i++) {
//			if (ide_bytes_received+i < 512) {
//				ide_drive_buffer[ide_bytes_received+i] = UART_Rx_data[i];
//			}
//		}
//		ide_bytes_received += UART_LEN;
//		if (ide_bytes_received >= 512) {
//			divide_command_status = DIVIDE_COMMAND_DATA_READY;
//			ide_drive_buffer_pointer = 0;
//		}
//		HAL_UART_Receive_IT(&huart2, UART_Rx_data, UART_LEN);	//activate UART receive interrupt every time
//	}
//
//}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USB_DEVICE_Init();
  //MX_UART4_Init();
  //MX_SDMMC1_SD_Init();
  MX_USART2_UART_Init();

  //HAL_UART_Receive_IT(&huart2, UART_Rx_data, UART_LEN);	//activate UART receive interrupt every time

  /* USER CODE BEGIN 2 */

  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  HAL_Delay(1000); // wait for USB (CDC) init, TODO make better

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  //pin_test();
  zx_init_pins();

  //zx_mem_emu_loop();

  //op_sniff_loop();

  uint8_t host_command_buffer[20];

  int p;

  volatile void* x = *EXTI4_IRQHandler;

  while (1)
  {
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	  HAL_Delay(100);

	  HAL_Delay(1);

	  if (divide_command != 0 && divide_command_status == DIVIDE_COMMAND_ISSUED) {
		  divide_command_status = DIVIDE_COMMAND_IN_PROGRESS;
		  //USB_printf("command: 0x%x LBA addr: %d'%d'%d'%d\r\n", divide_command, divide_lba_0, divide_lba_1, divide_lba_2, divide_lba_3);
		  ide_bytes_received = 0;
		  p = 0;
		  host_command_buffer[p++] = 0;
		  host_command_buffer[p++] = 3; // IDE device emulation
		  host_command_buffer[p++] = 5; // length of request payload
		  host_command_buffer[p++] = divide_command;
		  host_command_buffer[p++] = divide_lba_0;
		  host_command_buffer[p++] = divide_lba_1;
		  host_command_buffer[p++] = divide_lba_2;
		  host_command_buffer[p++] = divide_lba_3;
		  host_command_buffer[p++] = 0;
		  //CDC_Transmit_FS(host_command_buffer, p);
		  //divide_command_status = DIVIDE_COMMAND_DATA_READY;
		  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // 500ms timeout
	  }

	  // send "alive" event
	  p = 0;
	  host_command_buffer[p++] = 0;
	  host_command_buffer[p++] = 1; // alive event
	  host_command_buffer[p++] = 0; // length of request payload
	  host_command_buffer[p++] = 0;
	  //usb_result = CDC_Transmit_FS(host_command_buffer, p);
	  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // ms timeout

	  while (HAL_UART_Receive(&huart2, UART_Rx_data, UART_LEN, 100) == HAL_OK) { // ms timeout
		  ide_bytes_received_total += UART_LEN;

			// copy byte to ide buffer
			for (int i = 0; i < UART_LEN; i++) {
				if (ide_bytes_received+i < 512) {
					ide_drive_buffer[ide_bytes_received+i] = UART_Rx_data[i];
				}
			}
			// DI?
			ide_bytes_received += UART_LEN;
			if (ide_bytes_received >= 512) {
				divide_command_status = DIVIDE_COMMAND_DATA_READY;
				ide_drive_buffer_pointer = 0;
			}
			// EI?
	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  	// __WFI();


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  //RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLN = 230;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_HSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* SDMMC1 init function */
static void MX_SDMMC1_SD_Init(void)
{

  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1, &SDCardInfo1) != SD_OK)
  {
    Error_Handler();
  }

  HAL_SD_WideBusOperation_Config(&hsd1, SDMMC_BUS_WIDE_4B);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_7B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pins : ZX_A2_Pin ZX_A3_Pin ZX_A4_Pin ZX_A5_Pin
	                           ZX_A6_Pin ZX_A7_Pin ZX_A8_Pin ZX_A9_Pin
	                           ZX_A10_Pin ZX_A11_Pin ZX_A12_Pin ZX_A13_Pin
	                           ZX_A14_Pin ZX_A15_Pin ZX_A0_Pin ZX_A1_Pin */
	  GPIO_InitStruct.Pin = ZX_A2_Pin|ZX_A3_Pin|ZX_A4_Pin|ZX_A5_Pin
	                          |ZX_A6_Pin|ZX_A7_Pin|ZX_A8_Pin|ZX_A9_Pin
	                          |ZX_A10_Pin|ZX_A11_Pin|ZX_A12_Pin|ZX_A13_Pin
	                          |ZX_A14_Pin|ZX_A15_Pin|ZX_A0_Pin|ZX_A1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : ZX_D0_Pin ZX_D1_Pin ZX_D2_Pin ZX_D3_Pin
	                           ZX_D4_Pin ZX_D5_Pin ZX_D6_Pin ZX_D7_Pin */
	  GPIO_InitStruct.Pin = ZX_D0_Pin|ZX_D1_Pin|ZX_D2_Pin|ZX_D3_Pin
	                          |ZX_D4_Pin|ZX_D5_Pin|ZX_D6_Pin|ZX_D7_Pin;
	  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  // SPEED is set for later use, switch back to input mode
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : ZX_MEMREQ_Pin ZX_IOREQ_Pin ZX_RESET_Pin */
	  GPIO_InitStruct.Pin = ZX_MEMREQ_Pin|ZX_IOREQ_Pin|ZX_RESET_Pin|ZX_M1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : LED1_Pin */
	  GPIO_InitStruct.Pin = LED1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : ZX_WAIT_Pin ZX_ROMCS_Pin */
	  GPIO_InitStruct.Pin = ZX_WAIT_Pin|ZX_ROMCS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	  /*Configure GPIO pins : ZX_WR_Pin ZX_RD_Pin */
	  GPIO_InitStruct.Pin = ZX_WR_Pin|ZX_RD_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	}


/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
