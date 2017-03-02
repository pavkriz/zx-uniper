/*
 * emu_divide_ports.c
 *
 *  Created on: 28. 12. 2016
 *      Author: Kriz
 */

#include "emu_divide_ports.h"

#include "zx_signals.h"
#include "utils.h"
#include "zx_rd_wr_interrupt.h"

volatile int ide_drive_buffer_pointer = 0;
volatile uint8_t ide_drive_buffer[512];

int volatile divide_command = 0;
int volatile divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
int volatile divide_lba_0 = 0;
int volatile divide_lba_1 = 0;
int volatile divide_lba_2 = 0;
int volatile divide_lba_3 = 0;
volatile int divide_sector_count = 1;

void FAST_CODE divide_data_register_rd() {
	if (ide_drive_buffer_pointer < 512) {
		ZX_DATA_OUT(ide_drive_buffer[ide_drive_buffer_pointer++]);
	} else {
		ZX_DATA_OUT(0);
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
	}
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void FAST_CODE divide_data_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	if (ide_drive_buffer_pointer < 512) {
		ide_drive_buffer[ide_drive_buffer_pointer++] = data;
	}
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_WRITE_BUFFER_FILLED;
	}
	CLEAR_ZX_CONTROL_EXTI();
}


void FAST_CODE divide_sector_count_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_sector_count = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_drive_head_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_3 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_drive_head_register_rd() {
	ZX_DATA_OUT(divide_lba_3);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_status_register_rd() {
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

void FAST_CODE divide_error_register_rd() {
	ZX_DATA_OUT(0); // no error
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void FAST_CODE divide_command_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_command = data;
	ide_drive_buffer_pointer = 0;
	divide_command_status = DIVIDE_COMMAND_ISSUED;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba0_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_0 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba1_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_1 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba2_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_2 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba0_rd() {
	ZX_DATA_OUT(divide_lba_0);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba1_rd() {
	ZX_DATA_OUT(divide_lba_1);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba2_rd() {
	ZX_DATA_OUT(divide_lba_2);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void emu_divide_ports_init() {
	// write to divide_control_register 0x0e3 registered in emu_memory
	register_zx_port_read (0x0a3, divide_data_register_rd);
	register_zx_port_write(0x0a3, divide_data_register_wr);
	register_zx_port_read (0x0a7, divide_error_register_rd);
	register_zx_port_write(0x0a7, io_wr_hang_loop);
	register_zx_port_read (0x0ab, io_rd_hang_loop);
	register_zx_port_write(0x0ab, divide_sector_count_register_wr);
	register_zx_port_read (0x0af, divide_lba0_rd);
	register_zx_port_write(0x0af, divide_lba0_wr);
	register_zx_port_read (0x0b3, divide_lba1_rd);
	register_zx_port_write(0x0b3, divide_lba1_wr);
	register_zx_port_read (0x0b7, divide_lba2_rd);
	register_zx_port_write(0x0b7, divide_lba2_rd);
	register_zx_port_read (0x0bb, divide_drive_head_register_rd);
	register_zx_port_write(0x0bb, divide_drive_head_register_wr);
	register_zx_port_read (0x0bf, divide_status_register_rd);
	register_zx_port_write(0x0bf, divide_command_register_wr);
}
