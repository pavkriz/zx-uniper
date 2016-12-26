/*
 * rd_wr_interrupt.c
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#include "zx_signals.h"
#include "emu_memory.h"
#include "zx_rd_wr_interrupt.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"

int volatile divide_command = 0;
int volatile divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
int volatile divide_lba_0 = 0;
int volatile divide_lba_1 = 0;
int volatile divide_lba_2 = 0;
int volatile divide_lba_3 = 0;
volatile int divide_sector_count = 1;

#define IDE_STATUS_ERR 0x01
#define IDE_STATUS_DRQ 0x08
#define IDE_STATUS_DRDY 0x40
#define IDE_STATUS_BSY 0x80

#define DEASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = ZX_WAIT_Pin;}
#define ASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = (uint32_t)ZX_WAIT_Pin << 16;}

#define DEASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = ZX_RESET_Pin;}
#define ASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = (uint32_t)ZX_RESET_Pin << 16;}

#define DEASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = ZX_ROMCS_Pin;}
#define ASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = (uint32_t)ZX_ROMCS_Pin << 16;}

#define HANG_LOOP() {HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); while (1) {}}

zx_control_handler_t zx_io_rd_service_table[256];
zx_control_handler_t zx_io_wr_service_table[256];

volatile int last_m1_addr = 0;
volatile int last_io_op = 0;
volatile int dummy = 0;
uint8_t ide_operation = 0;
uint8_t ide_data_ready = 0;
volatile int ide_drive_buffer_pointer = 0;
uint8_t ide_drive_buffer_endian_pointer = 1;

volatile uint8_t ide_drive_buffer[512];
volatile int ide_bytes_received = 0;
volatile int ide_bytes_received_total = 0;
volatile int last_port_addr = 0;
volatile int last_port_data = 0;
volatile int last_port_op = 0;

//int divide_automap = 0;

void  __attribute__((section(".fast_code"))) zx_noop() {
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_wr_hang_loop() {
	last_io_op = 2;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_rd_hang_loop() {
	last_io_op = 1;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_data_register_rd() {
	if (ide_drive_buffer_pointer < 512) {
		ZX_DATA_OUT(ide_drive_buffer[ide_drive_buffer_pointer++]);
	} else {
		ZX_DATA_OUT(0);
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
		ide_data_ready = 0;
	}
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void  __attribute__((section(".fast_code"))) divide_data_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	if (ide_drive_buffer_pointer < 512) {
		ide_drive_buffer[ide_drive_buffer_pointer++] = data;
	}
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_WRITE_BUFFER_FILLED;
	}
	CLEAR_ZX_CONTROL_EXTI();
}


void  __attribute__((section(".fast_code"))) divide_sector_count_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_sector_count = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_drive_head_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_3 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_drive_head_register_rd() {
	ZX_DATA_OUT(divide_lba_3);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_status_register_rd() {
	if (divide_command_status == DIVIDE_COMMAND_DATA_READY) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_DRQ); // data ready (and device ready?)
	} else if ((divide_command_status == DIVIDE_COMMAND_ISSUED) || (divide_command_status == DIVIDE_COMMAND_IN_PROGRESS) || (divide_command_status == DIVIDE_COMMAND_WRITE_BUFFER_FILLED)) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_BSY); // Controller is busy executing a command.
	} else {
		ZX_DATA_OUT(IDE_STATUS_DRDY); // device ready
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_error_register_rd() {
	ZX_DATA_OUT(0); // no error
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void  __attribute__((section(".fast_code"))) divide_command_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_command = data;
	ide_drive_buffer_pointer = 0;
	divide_command_status = DIVIDE_COMMAND_ISSUED;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba0_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_0 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba1_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_1 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba2_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_2 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba0_rd() {
	ZX_DATA_OUT(divide_lba_0);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba1_rd() {
	ZX_DATA_OUT(divide_lba_1);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba2_rd() {
	ZX_DATA_OUT(divide_lba_2);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}


void copy_roms_to_ram() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	uint32_t i;
	// by default, do not handle IO operations on any port
	for (i = 0; i < 256; i++) zx_io_rd_service_table[i] = zx_noop;
	for (i = 0; i < 256; i++) zx_io_wr_service_table[i] = zx_noop;
	emu_memory_init();
	// register particular port handlers
	zx_io_rd_service_table[0x0a3] = divide_data_register_rd;
	zx_io_wr_service_table[0x0a3] = divide_data_register_wr;
	zx_io_rd_service_table[0x0a7] = divide_error_register_rd;
	zx_io_wr_service_table[0x0a7] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0ab] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0ab] = divide_sector_count_register_wr;
	zx_io_rd_service_table[0x0af] = divide_lba0_rd;
	zx_io_wr_service_table[0x0af] = divide_lba0_wr;
	zx_io_rd_service_table[0x0b3] = divide_lba1_rd;
	zx_io_wr_service_table[0x0b3] = divide_lba1_wr;
	zx_io_rd_service_table[0x0b7] = divide_lba2_rd;
	zx_io_wr_service_table[0x0b7] = divide_lba2_wr;
	zx_io_rd_service_table[0x0bb] = divide_drive_head_register_rd;
	zx_io_wr_service_table[0x0bb] = divide_drive_head_register_wr;
	zx_io_rd_service_table[0x0bf] = divide_status_register_rd;
	zx_io_wr_service_table[0x0bf] = divide_command_register_wr;
	// 128k mapovani?
	zx_io_wr_service_table[0x0fd] = zx_noop; // ignore writes
	zx_io_rd_service_table[0x0fd] = zx_noop; // ignore reads
	// nejaka mys?
	zx_io_wr_service_table[0x07f] = zx_noop; // ignore writes
	// kempston joystick
	zx_io_rd_service_table[0x01f] = zx_noop;
	zx_io_wr_service_table[0x01f] = zx_noop;
}

void  __attribute__((section(".fast_code"))) zx_io_rd() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
//	if (((address & 1) != 0) && ((address & 0xff) != 0x7f)  && ((address & 0xff) != 0xe3)) {
//		last_port_addr = address;
//		last_port_op = 1;
//	}
	// decode ports using 8 bits only
	zx_io_rd_service_table[address & 0xff]();
}

void  __attribute__((section(".fast_code"))) zx_io_wr() {
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
zx_control_handler_t exti_service_table[64]={
		// operation  // binary pattern for RD, WR, MEMREQ, dontcare, M1, IOREQ
		zx_noop	,  //	000000
		zx_noop	,  //	000001
		zx_noop	,  //	000010
		zx_noop	,  //	000011
		zx_noop	,  //	000100
		zx_noop	,  //	000101
		zx_noop	,  //	000110
		zx_noop	,  //	000111
		zx_noop	,  //	001000
		zx_noop	,  //	001001
		zx_noop	,  //	001010
		zx_noop	,  //	001011
		zx_noop	,  //	001100
		zx_noop	,  //	001101
		zx_noop	,  //	001110
		zx_noop	,  //	001111
		zx_noop	,  //	010000
		zx_mem_rd_m1	,  //	010001
		zx_noop	,  //	010010
		zx_mem_rd_nonm1	,  //	010011
		zx_noop	,  //	010100
		zx_mem_rd_m1	,  //	010101
		zx_noop	,  //	010110
		zx_mem_rd_nonm1	,  //	010111
		zx_noop	,  //	011000
		zx_noop	,  //	011001
		zx_io_rd	,  //	011010
		zx_noop	,  //	011011
		zx_noop	,  //	011100
		zx_noop	,  //	011101
		zx_io_rd	,  //	011110
		zx_noop	,  //	011111
		zx_noop	,  //	100000
		zx_noop	,  //	100001
		zx_noop	,  //	100010
		zx_mem_wr	,  //	100011
		zx_noop	,  //	100100
		zx_noop	,  //	100101
		zx_noop	,  //	100110
		zx_mem_wr	,  //	100111
		zx_noop	,  //	101000
		zx_noop	,  //	101001
		zx_io_wr	,  //	101010
		zx_noop	,  //	101011
		zx_noop	,  //	101100
		zx_noop	,  //	101101
		zx_io_wr	,  //	101110
		zx_noop	,  //	101111
		zx_noop	,  //	110000
		zx_noop	,  //	110001
		zx_noop	,  //	110010
		zx_noop	,  //	110011
		zx_noop	,  //	110100
		zx_noop	,  //	110101
		zx_noop	,  //	110110
		zx_noop	,  //	110111
		zx_noop	,  //	111000
		zx_noop	,  //	111001
		zx_noop	,  //	111010
		zx_noop	,  //	111011
		zx_noop	,  //	111100
		zx_noop	,  //	111101
		zx_noop	,  //	111110
		zx_noop	,  //	111111
};



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
 * Initialize particular pins to output mode
 */
void zx_init_pins(void) {

	copy_roms_to_ram();


	DEASSERT_ZX_RESET();
	init_output_od(ZX_RESET_GPIO_Port, ZX_RESET_Pin);

	DEASSERT_ZX_WAIT();
	//init_output_od_pullup(ZX_WAIT_GPIO_Port, ZX_WAIT_Pin);
	init_output_pp(ZX_WAIT_GPIO_Port, ZX_WAIT_Pin);

	// disable internal ZX ROM permanently
	DEASSERT_ZX_ROMCS();
	init_output_pp(ZX_ROMCS_GPIO_Port, ZX_ROMCS_Pin);

	// reset ZX Spectrum

	ASSERT_ZX_RESET();


	HAL_Delay(500);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_Delay(500);


	DEASSERT_ZX_RESET();
}

void register_zx_port_write(uint8_t port, zx_control_handler_t handler) {
	zx_io_wr_service_table[port] = handler;
}

void register_zx_port_read(uint8_t port, zx_control_handler_t handler) {
	zx_io_rd_service_table[port] = handler;
}

void __attribute__((section(".fast_code"))) EXTI4_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
	//CLEAR_ZX_CONTROL_EXTI();
}

void __attribute__((section(".fast_code"))) EXTI9_5_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
	//CLEAR_ZX_CONTROL_EXTI();
}
