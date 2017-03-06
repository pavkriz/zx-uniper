/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "usb_device.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */

#include <zx_rd_wr_interrupt.h>
#include "op_sniff_loop.h"
#include "stm32f7xx_hal_conf.h"
#include "pintest.h"
#include <stdarg.h>
#include "usb_host.h"
#include "usbh_msc.h"
#include "emu_divide_ports.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define HANG_LOOP() {HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); while (1) {}}

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define UART_LEN 1
uint8_t UART_Rx_data[UART_LEN];
volatile int rx_timeouts = 0;

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


//Offset(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
//
//00000000  7A 42 FF 3F 37 C8 10 00 00 00 00 00 3F 00 00 00  zB.?7.......?...
//00000010  00 00 00 00 20 20 20 20 57 20 2D 44 4D 57 4E 41  ....    W -DMWNA
//00000020  33 4B 33 30 39 34 33 38 00 00 00 10 32 00 30 32  3K309438....2.02
//00000030  30 2E 4B 30 30 32 44 57 20 43 44 57 35 32 30 30  0.K002DW CDW5200
//00000040  42 42 35 2D 52 35 41 44 20 30 20 20 20 20 20 20  BB5-R5AD 0
//00000050  20 20 20 20 20 20 20 20 20 20 20 20 20 20 10 80                .€
//00000060  00 00 00 2F 01 40 00 00 00 00 07 00 DD 10 0F 00  .../.@......Ý...
//00000070  FF 00 0D F6 FB 00 10 01 FF FF FF 0F 00 00 07 04  ...ö............
//00000080  03 00 78 00 78 00 78 00 78 00 00 00 00 00 00 00  ..x.x.x.x.......
//00000090  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000A0  FE 00 00 00 6B 74 01 7F 33 46 69 74 01 3E 23 46  ....kt..3Fit.>#F
//000000B0  3F 00 00 00 00 00 00 00 FE FF 0D 60 80 80 08 00  ?..........`€€..
//000000C0  00 00 00 00 A0 86 01 00 70 59 1C 1D 00 00 00 00  .... †..pY......
//000000D0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000000F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000100  01 00 00 00 00 00 00 00 00 00 76 12 00 00 00 00  ..........v.....
//00000110  00 00 00 00 00 00 00 00 00 00 00 00 04 00 00 00  ................
//00000120  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000130  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000140  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000150  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000160  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000170  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000180  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//00000190  00 00 00 00 00 00 00 00 00 00 00 00 3F 00 00 00  ............?...
//000001A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001B0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001D0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
//000001F0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 A5 B2  ................


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
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 2 */

  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t host_command_buffer[20];
  uint8_t sector_buffer[1024];
  int p;

  UART2_printf("=========== START ===============\r\n");

  int usb_disk_found = 0;
  int led_status = 0;
  // wait for USB disk while flashing LED
  while (!usb_disk_found) {
	  HAL_Delay(5);
//	  	  	  p = 0;
//	  		  host_command_buffer[p++] = 'A';
//	  		  host_command_buffer[p++] = '0' + GetUsbHostAppliState();
//	  		  host_command_buffer[p++] = 13;
//	  		  host_command_buffer[p++] = 10;
//
//	  		  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // ms timeout
	  if (GetUsbHostAppliState() == APPLICATION_READY) {
		  usb_disk_found = 1;
	  } else {
		  led_status = !led_status;
		  if (led_status) {
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		  } else {
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		  }
	  }
	  MX_USB_HOST_Process();
  }

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);


  MSC_LUNTypeDef lunInfo;
  MSC_GetLUN0Info(&lunInfo);
  UART2_printf("USB block_number=%d, block_size=%d\r\n", lunInfo.capacity.block_nbr, lunInfo.capacity.block_size);

  int startTick = HAL_GetTick();
  int zx_init_done = 0;

  UART2_printf("zx_init_pins start\r\n");
  zx_init_pins();
  UART2_printf("zx_init_pins end\r\n");
  zx_init_done = 1;

  while (1)
  {

//	  if (((HAL_GetTick() - startTick) > 10000) && !zx_init_done) {
//		  UART2_printf("zx_init_pins start\r\n");
//		  zx_init_pins();
//		  UART2_printf("zx_init_pins end\r\n");
//		  zx_init_done = 1;
//	  }

	  if (!zx_init_done) {
	  //if (!zx_init_done || (zx_init_done && (((HAL_GetTick()/1000) % 2) == 0))) {
		  HAL_Delay(100);

		  p = 0;
		  host_command_buffer[p++] = 'A';
		  host_command_buffer[p++] = '0' + GetUsbHostAppliState();
		  host_command_buffer[p++] = 13;
		  host_command_buffer[p++] = 10;

		  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // ms timeout

		  if (GetUsbHostAppliState() == APPLICATION_READY) {
			  // USB Drive ready
			  // get info
			  MSC_LUNTypeDef lunInfo;
			  MSC_GetLUN0Info(&lunInfo);
			  UART2_printf("block_number=%d, block_size=%d\r\n", lunInfo.capacity.block_nbr, lunInfo.capacity.block_size);

			  MSC_Read(0, sector_buffer, 1);
			  UART2_printf("Data sector 0:");
			  for (int i = 0; i < 512; i++) {
				  if (i % 16 == 0) UART2_printf("\r\n%04x:", i);
				  UART2_printf("%02x ", sector_buffer[i]);
			  }
			  UART2_printf("\r\n");

			  MSC_Read(2048, sector_buffer, 1);
			  UART2_printf("Data sector 2048:");
			  for (int i = 0; i < 512; i++) {
				  if (i % 16 == 0) UART2_printf("\r\n%04x:", i);
				  UART2_printf("%02x ", sector_buffer[i]);
			  }
			  UART2_printf("\r\n");
			  HAL_Delay(1000);
		  }
	  }

	  emu_divide_ports_handle_main_loop();

  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
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
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);

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
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ZX_A15_B_Pin ZX_RESET_Pin ZX_IOREQ_Pin ZX_M1_Pin 
                           ZX_MEMREQ_Pin ZX_A13_B_Pin ZX_A14_B_Pin */
  GPIO_InitStruct.Pin = ZX_A15_B_Pin|ZX_RESET_Pin|ZX_IOREQ_Pin|ZX_M1_Pin 
                          |ZX_MEMREQ_Pin|ZX_A13_B_Pin|ZX_A14_B_Pin;
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ZX_WR_Pin ZX_RD_Pin */
  GPIO_InitStruct.Pin = ZX_WR_Pin|ZX_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


int main_UART2_IDE_emu(void)
{

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
  MX_USB_HOST_Init();


  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  HAL_Delay(1000); // wait for USB (CDC) init, TODO make better


  /* Infinite loop */


  //pin_test();
  zx_init_pins();

  //zx_mem_emu_loop();

  //op_sniff_loop();

  int ide_bytes_received_total = 0;

  uint8_t host_command_buffer[20];

  int p;

  int ide_bytes_received = 0;

  while (1)
  {
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//	  HAL_Delay(100);

	  //HAL_Delay(1);

	  //__disable_irq();
	  if (divide_command_status == DIVIDE_COMMAND_ISSUED) {
		  divide_command_status = DIVIDE_COMMAND_IN_PROGRESS;
		  //__enable_irq();
		  //USB_printf("command: 0x%x LBA addr: %d'%d'%d'%d\r\n", divide_command, divide_lba_0, divide_lba_1, divide_lba_2, divide_lba_3);
		  ide_bytes_received = 0;
		  p = 0;
		  host_command_buffer[p++] = 0;
		  host_command_buffer[p++] = 3; // IDE device emulation
		  host_command_buffer[p++] = 6; // length of request payload
		  host_command_buffer[p++] = divide_command;
		  host_command_buffer[p++] = divide_lba_0;
		  host_command_buffer[p++] = divide_lba_1;
		  host_command_buffer[p++] = divide_lba_2;
		  host_command_buffer[p++] = divide_lba_3;
		  host_command_buffer[p++] = divide_sector_count;
		  host_command_buffer[p++] = 0;
		  //CDC_Transmit_FS(host_command_buffer, p);
		  //divide_command_status = DIVIDE_COMMAND_DATA_READY;
		  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // 500ms timeout
		  ide_drive_buffer_pointer = 0;
	  } else {
		  //__enable_irq();
	  }

	  if (divide_command_status != DIVIDE_COMMAND_IN_PROGRESS) {

		  // send "alive" event
		  p = 0;
		  host_command_buffer[p++] = 0;
		  host_command_buffer[p++] = 1; // alive event
		  host_command_buffer[p++] = 0; // length of request payload
		  host_command_buffer[p++] = 0;
		  //usb_result = CDC_Transmit_FS(host_command_buffer, p);
		  HAL_UART_Transmit(&huart2, host_command_buffer, p, 500); // ms timeout
	  } else {
		  while ((divide_command_status == DIVIDE_COMMAND_IN_PROGRESS) && (HAL_UART_Receive(&huart2, UART_Rx_data, UART_LEN, 10) == HAL_OK)) { // ms timeout
			  ide_bytes_received_total += UART_LEN;

				// copy byte to ide buffer
				for (int i = 0; i < UART_LEN; i++) {
					if (ide_bytes_received+i < 512) {
						ide_drive_buffer[ide_bytes_received+i] = UART_Rx_data[i];
					}
				}
				// DI?
				//__disable_irq();
				ide_bytes_received += UART_LEN;
				if (ide_bytes_received >= 512) {
					divide_command_status = DIVIDE_COMMAND_DATA_READY;
				}
				 //__enable_irq();
				// EI?
		  }
		  if (divide_command_status == DIVIDE_COMMAND_IN_PROGRESS) rx_timeouts++;
	  }

	  	// __WFI();


  }

}

void UART2_printf(const char *fmt, ...) {
	char textbuf[2000]; //string buffer

	va_list args;
	va_start(args, fmt);
	vsprintf(textbuf, fmt, args);
	va_end(args);
	HAL_UART_Transmit(&huart2,textbuf, strlen(textbuf), 500); // ms timeout
}

USBH_UserProcess_My(uint8_t id) {
	UART2_printf("USBH_UserProcess_My %d\r\n", id);
}

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
