/*
 * emu_divide_ports.h
 *
 *  Created on: 28. 12. 2016
 *      Author: Kriz
 */

#ifndef EMU_DIVIDE_PORTS_H_
#define EMU_DIVIDE_PORTS_H_

#include <stdint.h>

#define IDE_STATUS_ERR 0x01
#define IDE_STATUS_DRQ 0x08
#define IDE_STATUS_DRDY 0x40
#define IDE_STATUS_BSY 0x80

#define DIVIDE_COMMAND_DEVICE_READY 1
#define DIVIDE_COMMAND_ISSUED 2
#define DIVIDE_COMMAND_IN_PROGRESS 3
#define DIVIDE_COMMAND_DATA_READY 4
#define DIVIDE_COMMAND_DATA_FETCHED 5
#define DIVIDE_COMMAND_WRITE_FILLING_BUFFER 6
#define DIVIDE_COMMAND_WRITE_BUFFER_FILLED 7

extern volatile int ide_drive_buffer_pointer;
extern volatile uint8_t ide_drive_buffer[512];

extern volatile int divide_command;
extern volatile int divide_command_status;

extern volatile int divide_lba_0;
extern volatile int divide_lba_1;
extern volatile int divide_lba_2;
extern volatile int divide_lba_3;
extern volatile int divide_sector_count;

void emu_divide_ports_init();

#endif /* EMU_DIVIDE_PORTS_H_ */
