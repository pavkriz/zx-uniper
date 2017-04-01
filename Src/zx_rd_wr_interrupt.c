/*
 * rd_wr_interrupt.c
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#include "zx_signals.h"
#include "emu_memory.h"
#include "emu_divide_ports.h"
#include "zx_rd_wr_interrupt.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"
#include "utils.h"

#define EXTI_SERVICE_TABLE_LENGTH 512

#define DEASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = ZX_WAIT_Pin;}
#define ASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = (uint32_t)ZX_WAIT_Pin << 16;}

zx_control_handler_t zx_io_rd_service_table[256];
zx_control_handler_t zx_io_wr_service_table[256];

volatile int last_m1_addr = 0;
volatile int last_io_op = 0;
volatile int dummy = 0;
uint8_t ide_operation = 0;

volatile int last_port_addr = 0;
volatile int last_port_data = 0;
volatile int last_port_op = 0;

void FAST_CODE zx_noop() {
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE io_wr_hang_loop() {
	last_io_op = 2;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE io_rd_hang_loop() {
	last_io_op = 1;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE zx_io_rd() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
//	if (((address & 1) != 0) && ((address & 0xff) != 0x7f)  && ((address & 0xff) != 0xe3)) {
//		last_port_addr = address;
//		last_port_op = 1;
//	}
	// decode ports using 8 bits only
	zx_io_rd_service_table[address & 0xff]();
}

void FAST_CODE zx_io_wr() {
	// TODO register
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
//	if (((address & 1) != 0) && ((address & 0xff) != 0x7f)  && ((address & 0xff) != 0xe3)) {
//		last_port_addr = address;
//		last_port_op = 2;
//	}
//	if (address & 1) {
//		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//		while (1) {
//
//		}
//	}
	// decode ports using 8 bits only
	zx_io_wr_service_table[address & 0xff]();
}


//static const
zx_control_handler_t exti_service_table[EXTI_SERVICE_TABLE_LENGTH];

void clear_zx_control_lines_handlers() {
	for (int i = 0; i < EXTI_SERVICE_TABLE_LENGTH; i++) {
		exti_service_table[i] = zx_noop; // do nothing by default
	}
}

void register_zx_control_lines_handler(uint16_t lines_to_be_low, uint16_t lines_to_be_high, zx_control_handler_t handler) {
	for (int i = 0; i < EXTI_SERVICE_TABLE_LENGTH; i++) {
		if (((i & lines_to_be_low) == 0) && ((i & lines_to_be_high) == lines_to_be_high)) {
			exti_service_table[i] = handler;
		}
	}
}

void register_zx_control_lines_block_handler(uint16_t lines_to_be_low, uint16_t lines_to_be_high, int num_8k_block, zx_control_handler_t handler) {
	for (int i = 0; i < EXTI_SERVICE_TABLE_LENGTH; i++) {
		// ZX_A13_B_Pin, ZX_A14_B_Pin, ZX_A15_B_Pin need to be in line on the port in order to be able to contruct block number here
		// (may be refactored when different pin-to-signal mapping will be needed)
		volatile int blk = ((i & (ZX_A13_B_Pin | ZX_A14_B_Pin | ZX_A15_B_Pin)) >> 6);
		if (((i & lines_to_be_low) == 0) && ((i & lines_to_be_high) == lines_to_be_high) && (blk == num_8k_block)) {
			UART2_printf("register_zx_control_lines_block_handler hang to blk=%d i=0x%02x\r\n", blk, i);
			exti_service_table[i] = handler;
		}
	}
}

/**
 * Set GPIO as output open-drain with pull-up
 */
void inline init_output_od_pullup(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Set GPIO as output open-drain (without pull-up/down)
 */
void inline init_output_od(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Set GPIO as output push-pull (without pull-up/down)
 */
void inline init_output_pp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Initialize particular pins to output mode,
 * enable GPIO interrupts on ZX_RD and ZX_WRITE,
 * keep ZX Spectrum stopped (via ZX_RESET)
 */
void zx_init_pins(void) {
	// make sure we do not handle any ZX control operations
	clear_zx_control_lines_handlers();

	ASSERT_ZX_RESET();
	init_output_od(ZX_RESET_GPIO_Port, ZX_RESET_Pin);

	DEASSERT_ZX_WAIT();
	init_output_pp(ZX_WAIT_GPIO_Port, ZX_WAIT_Pin);

	// disable internal ZX ROM permanently
	DEASSERT_ZX_ROMCS();
	init_output_pp(ZX_ROMCS_GPIO_Port, ZX_ROMCS_Pin);

	// make sure ZX Spectrum is stopped
	ASSERT_ZX_RESET();

	/* EXTI interrupt init*/
	HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void clear_zx_port_handlers() {
	// by default, do not handle IO operations on any port
	int i;
	for (i = 0; i < 256; i++) zx_io_rd_service_table[i] = zx_noop;
	for (i = 0; i < 256; i++) zx_io_wr_service_table[i] = zx_noop;
}

void register_zx_port_write(uint8_t port, zx_control_handler_t handler) {
	zx_io_wr_service_table[port] = handler;
}

void register_zx_port_read(uint8_t port, zx_control_handler_t handler) {
	zx_io_rd_service_table[port] = handler;
}

void FAST_CODE EXTI4_IRQHandler(void) {

	// read control lines: pins ZX_IOREQ, ZX_M1, (dont-care), ZX_MEMREQ, ZX_WR, ZX_RD, ZX_A13, ZX_A14, ZX_A15
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111111;

	// call registered operation handler
	exti_service_table[control]();
}

void FAST_CODE EXTI9_5_IRQHandler(void) {

	// read control lines: pins ZX_IOREQ, ZX_M1, (dont-care), ZX_MEMREQ, ZX_WR, ZX_RD, ZX_A13, ZX_A14, ZX_A15
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111111;

	// call registered operation handler
	exti_service_table[control]();
}
