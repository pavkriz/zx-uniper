/*
 * rd_wr_interrupt.h
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#ifndef ZX_RD_WR_INTERRUPT_H_X_
#define ZX_RD_WR_INTERRUPT_H_X_

#include <stdint.h>

#define ZX_ADDR_GPIO_PORT GPIOE
#define ZX_DATA_GPIO_PORT GPIOB
#define ZX_CONTROL_IN_GPIO_PORT GPIOD

void zx_init_pins(void);
void zx_rd_wr_handler(void);
void zx_mem_emu_loop(void);
void copy_roms_to_ram();

extern volatile int divide_command;
extern volatile int divide_command_status;

#define DIVIDE_COMMAND_DEVICE_READY 1
#define DIVIDE_COMMAND_ISSUED 2
#define DIVIDE_COMMAND_IN_PROGRESS 3
#define DIVIDE_COMMAND_DATA_READY 4
#define DIVIDE_COMMAND_DATA_FETCHED 5

extern volatile int divide_lba_0;
extern volatile int divide_lba_1;
extern volatile int divide_lba_2;
extern volatile int divide_lba_3;
extern volatile int divide_sector_count;

extern volatile uint8_t ide_drive_buffer[512];
extern volatile int ide_bytes_received;
extern volatile int ide_bytes_received_total;
extern volatile int ide_drive_buffer_pointer;

typedef void (*zx_control_handler_t)(void);

extern zx_control_handler_t exti_service_table[64];


#endif /* ZX_RD_WR_INTERRUPT_H_X_ */
