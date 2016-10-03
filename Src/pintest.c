/*
 * pintest.c
 *
 *  Created on: 23. 7. 2016
 *      Author: Kriz
 */

#include "pintest.h"
#include "stm32f7xx_hal_conf.h"
#include "usbd_cdc_if.h"

void report_pin(const char *pin_name, uint32_t high_count, uint32_t total_samples_count) {
	uint32_t pct = 1000*high_count/total_samples_count;
	USB_printf("Pin %6s samples high: %7ld %3ld.%01ld%%\r\n", pin_name, high_count, pct/10, pct % 10);
}

void pin_test(void) {
	uint32_t pin_counts[32] = { 0 };
	uint32_t data;
	uint32_t count = 0;

	USB_printf("************* sampling...\r\n");
	while (1) {
		// ZX_D0..ZX_D7
		data = GPIOB->IDR;
		if (data & 1)   pin_counts[0]++;
		if (data & 2)   pin_counts[1]++;
		if (data & 4)   pin_counts[2]++;
		if (data & 8)   pin_counts[3]++;
		if (data & 16)  pin_counts[4]++;
		if (data & 32)  pin_counts[5]++;
		if (data & 64)  pin_counts[6]++;
		if (data & 128) pin_counts[7]++;

		// ZX_A0..ZX_A15
		data = GPIOE->IDR;
		if (data & 1)     pin_counts[8]++;
		if (data & 2)     pin_counts[9]++;
		if (data & 4)     pin_counts[10]++;
		if (data & 8)     pin_counts[11]++;
		if (data & 16)    pin_counts[12]++;
		if (data & 32)    pin_counts[13]++;
		if (data & 64)    pin_counts[14]++;
		if (data & 128)   pin_counts[15]++;
		if (data & 256)   pin_counts[16]++;
		if (data & 512)   pin_counts[17]++;
		if (data & 1024)  pin_counts[18]++;
		if (data & 2048)  pin_counts[19]++;
		if (data & 4096)  pin_counts[20]++;
		if (data & 8192)  pin_counts[21]++;
		if (data & 16384) pin_counts[22]++;
		if (data & 32768) pin_counts[23]++;

		// control lines on port D
		data = GPIOD->IDR;
		if (data & ZX_RESET_Pin) pin_counts[24]++;
		if (data & ZX_MEMREQ_Pin) pin_counts[25]++;
		if (data & ZX_IOREQ_Pin) pin_counts[26]++;
		if (data & ZX_RD_Pin) pin_counts[27]++;
		if (data & ZX_WR_Pin) pin_counts[28]++;

		// control lines on port A
		data = GPIOA->IDR;
		if (data & ZX_ROMCS_Pin) pin_counts[29]++;
		if (data & ZX_WAIT_Pin) pin_counts[30]++;

		if (++count == 1000000) {
			// blink LED
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			// dump pin counts
			USB_printf("************* systick=%ld\r\n", HAL_GetTick());
			report_pin("D0", pin_counts[0], count);
			report_pin("D1", pin_counts[1], count);
			report_pin("D2", pin_counts[2], count);
			report_pin("D3", pin_counts[3], count);
			report_pin("D4", pin_counts[4], count);
			report_pin("D5", pin_counts[5], count);
			report_pin("D6", pin_counts[6], count);
			report_pin("D7", pin_counts[7], count);
			report_pin("A0", pin_counts[8], count);
			report_pin("A1", pin_counts[9], count);
			report_pin("A2", pin_counts[10], count);
			report_pin("A3", pin_counts[11], count);
			report_pin("A4", pin_counts[12], count);
			report_pin("A5", pin_counts[13], count);
			report_pin("A6", pin_counts[14], count);
			report_pin("A7", pin_counts[15], count);
			report_pin("A8", pin_counts[16], count);
			report_pin("A9", pin_counts[17], count);
			report_pin("A10", pin_counts[18], count);
			report_pin("A11", pin_counts[19], count);
			report_pin("A12", pin_counts[20], count);
			report_pin("A13", pin_counts[21], count);
			report_pin("A14", pin_counts[22], count);
			report_pin("A15", pin_counts[23], count);
			report_pin("RESET", pin_counts[24], count);
			report_pin("MEMREQ", pin_counts[25], count);
			report_pin("IOREQ", pin_counts[26], count);
			report_pin("RD", pin_counts[27], count);
			report_pin("WR", pin_counts[28], count);
			report_pin("ROMCS", pin_counts[29], count);
			report_pin("WAIT", pin_counts[30], count);
			USB_printf("************* sampling...\r\n");

			count = 0;

			// zero pin counters
			for (int i = 0; i < 32; i++) {
				pin_counts[i] = 0;
			}
		}
	}

}
