/*
 * op_sniff_loop.c
 *
 *  Created on: 9. 8. 2016
 *      Author: Kriz
 */

#include "op_sniff_loop.h"

#include "stm32f7xx_hal_conf.h"
#include "usbd_cdc_if.h"


#define LOG_SIZE 1000

__attribute__((section(".fast_data")))
uint8_t log_data[LOG_SIZE];
__attribute__((section(".fast_data")))
uint16_t log_addr[LOG_SIZE];
__attribute__((section(".fast_data")))
uint16_t log_len[LOG_SIZE];


void __attribute__((section(".fast_code"))) op_sniff_loop (void) {
	// control lines on port D
	uint32_t control;
	uint16_t addr;
	uint8_t data;
	uint16_t len = 0;

	uint16_t log_count = 0;

	GPIO_InitTypeDef GPIO_InitStruct;

	// assert ZX_RESET

	HAL_GPIO_WritePin(ZX_RESET_GPIO_Port, ZX_RESET_Pin, GPIO_PIN_RESET); // 0=active

	// make sure it's in output mode
	GPIO_InitStruct.Pin = ZX_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ZX_RESET_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(ZX_RESET_GPIO_Port, ZX_RESET_Pin, GPIO_PIN_RESET); // LOW=active

	HAL_Delay(1000); // wait 1s

	// deassert ZX_RESET

	HAL_GPIO_WritePin(ZX_RESET_GPIO_Port, ZX_RESET_Pin, GPIO_PIN_SET); // HIGH=inactive


	// now log memory reads

	while (1) {

		control = GPIOD->IDR;
		if ((control & (ZX_RD_Pin | ZX_MEMREQ_Pin)) == 0) {
			// it's memory read
			// read ADDR bus
			addr = GPIOE->IDR;
			// wait a little for data to become ready on DATA bus (ZX RAM is slow :) )
			asm volatile("" ::: "memory");
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
					__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();
							__NOP();


			asm volatile("" ::: "memory");
			// read DATA bus
			data = GPIOB->IDR;
			log_data[log_count] = data;
			log_addr[log_count] = addr;
			LED1_GPIO_Port->BSRR = LED1_Pin; // set indication HIGH
			while ((GPIOD->IDR & (ZX_RD_Pin | ZX_MEMREQ_Pin)) == 0) { len++; } // wait this read cycle
			LED1_GPIO_Port->BSRR = (uint32_t)LED1_Pin << 16; // set indication LOW
			log_len[log_count] = len;
			len = 0;
			log_count++;
			if (log_count >= LOG_SIZE) {
				for (int i = 0; i < LOG_SIZE; i++) {
					USB_printf("Read #%4d addr: %5d %#6x data: %3d %#4x len: %3d\r\n", i, log_addr[i], log_addr[i], log_data[i], log_data[i], log_len[i]);
				}
				return;
			}
		}
	}
}
