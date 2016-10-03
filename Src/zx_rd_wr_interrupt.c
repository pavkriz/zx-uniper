/*
 * rd_wr_interrupt.c
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#include <raiders_rom.h>
#include <zxtest_rom.h>
#include <esxdos080_rom.h>
#include <didaktik_gama_88_rom.h>
#include <fatware014_rom.h>
#include "zx_rd_wr_interrupt.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"

#define ZX_ADDR_GPIO_PORT GPIOE
#define ZX_DATA_GPIO_PORT GPIOB
#define ZX_CONTROL_IN_GPIO_PORT GPIOD

#define DEASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = ZX_WAIT_Pin;}
#define ASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = (uint32_t)ZX_WAIT_Pin << 16;}

#define DEASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = ZX_RESET_Pin;}
#define ASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = (uint32_t)ZX_RESET_Pin << 16;}

#define DEASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = ZX_ROMCS_Pin;}
#define ASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = (uint32_t)ZX_ROMCS_Pin << 16;}

#define ZX_DATA_OUT(data) {ZX_DATA_GPIO_PORT->BSRR = (uint32_t)0xff << 16; ZX_DATA_GPIO_PORT->MODER = (uint32_t)0b0101010101010101;  ZX_DATA_GPIO_PORT->BSRR = data;  } // ala UNICARD, ale MODER natvrdo misto |=, MODER doprostred
#define ZX_DATA_HI_Z() {ZX_DATA_GPIO_PORT->MODER = 0;}

#define ZX_IS_MEM_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_MEMREQ_Pin)) == 0)
#define ZX_IS_IO_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_IOREQ_Pin)) == 0)

#define CLEAR_ZX_CONTROL_EXTI() {__HAL_GPIO_EXTI_CLEAR_IT(ZX_RD_Pin);__HAL_GPIO_EXTI_CLEAR_IT(ZX_WR_Pin);}
#define HANG_LOOP() {HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); while (1) {}}

typedef void (*zx_control_handler_t)(void);

zx_control_handler_t zx_io_rd_service_table[256];
zx_control_handler_t zx_io_wr_service_table[256];

//0. 16kb = Divide RAM pages 0,1
//1. 16kb = Divide RAM pages 2,3
//2. 16kB = ZX ROM copy
//3. 16kB = Divide ROM
uint8_t device_ram[16384*4];
uint8_t divide_mapped = 0;
uint8_t divide_page = 0;
volatile int last_m1_addr = 0;
volatile int last_io_op = 0;
volatile int dummy = 0;
int32_t low_8k_rom_offset = 0;
int32_t high_8k_rom_offset = 0;
uint8_t ide_operation = 0;
uint8_t ide_data_ready = 0;
uint8_t ide_drive_buffer_pointer = 0;
uint8_t ide_drive_buffer_endian_pointer = 1;

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

uint8_t ide_drive_itentify_buffer[512] = {
		// WD2500BB ATA_CMD_IDENTIFY  0xEC command output buffer, lowbyte, highbyte
		0x7A,0x42,0xFF,0x3F,0x37,0xC8,0x10,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x57,0x20,0x2D,0x44,0x4D,0x57,0x4E,0x41,
		0x33,0x4B,0x33,0x30,0x39,0x34,0x33,0x38,0x00,0x00,0x00,0x10,0x32,0x00,0x30,0x32,
		0x30,0x2E,0x4B,0x30,0x30,0x32,0x44,0x57,0x20,0x43,0x44,0x57,0x35,0x32,0x30,0x30,
		0x42,0x42,0x35,0x2D,0x52,0x35,0x41,0x44,0x20,0x30,0x20,0x20,0x20,0x20,0x20,0x20,
		0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x80,
		0x00,0x00,0x00,0x2F,0x01,0x40,0x00,0x00,0x00,0x00,0x07,0x00,0xDD,0x10,0x0F,0x00,
		0xFF,0x00,0x0D,0xF6,0xFB,0x00,0x10,0x01,0xFF,0xFF,0xFF,0x0F,0x00,0x00,0x07,0x04,
		0x03,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0xFE,0x00,0x00,0x00,0x6B,0x74,0x01,0x7F,0x33,0x46,0x69,0x74,0x01,0x3E,0x23,0x46,
		0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,0x0D,0x60,0x80,0x80,0x08,0x00,
		0x00,0x00,0x00,0x00,0xA0,0x86,0x01,0x00,0x70,0x59,0x1C,0x1D,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x12,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA5,0xB2
};

//int divide_automap = 0;

void __attribute__((section(".fast_code"))) __STATIC_INLINE divide_set_automap_on() {
	low_8k_rom_offset = 3*0x4000; // Divide ROM
	high_8k_rom_offset = divide_page*0x2000; // Divide RAM page (8kB) 0..3
	divide_mapped = 1;
}

void __attribute__((section(".fast_code"))) __STATIC_INLINE divide_set_automap_off() {
	low_8k_rom_offset = 2*0x4000; // ZX ROM copy (low 8kB)
	high_8k_rom_offset = 2*0x4000 + 0x2000; // ZX ROM copy (high 8kB)
	divide_mapped = 0;
}

void  __attribute__((section(".fast_code"))) zx_noop() {
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_wr_hang_loop() {
	last_io_op = 2;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	//HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_rd_hang_loop() {
	last_io_op = 1;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	//HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_data_register_rd() {
//	if (ide_drive_buffer_word_pointer < 256) {
//		// return in order lowbyte, highbyte
//		if (ide_drive_buffer_endian_pointer) {
//			// now return low byte
//			ZX_DATA_OUT(ide_drive_itentify_buffer[ide_drive_buffer_word_pointer*2+1]);
//			ide_drive_buffer_endian_pointer = 0;
//		} else {
//			// now return high byte and move pointer to next word
//			ZX_DATA_OUT(ide_drive_itentify_buffer[ide_drive_buffer_word_pointer*2]);
//			ide_drive_buffer_word_pointer++;
//			ide_drive_buffer_endian_pointer = 1;
//		}
//	} else {
//		ZX_DATA_OUT(0);
//	}
	if (ide_drive_buffer_pointer < 512) {
		ZX_DATA_OUT(ide_drive_itentify_buffer[ide_drive_buffer_pointer++]);
	} else {
		ZX_DATA_OUT(0);
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if (ide_drive_buffer_pointer == 512) ide_data_ready = 0;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_control_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_page = data & 0b11;
	if (divide_mapped) {
		divide_set_automap_on(); // recalculate offsets for a new RAM page
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_drive_head_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	int is_slave = data & 0b00010000;
	if (!is_slave) {
		// do nothing "select master drive"
		// TODO obey LBA bit
	} else {
		//HANG_LOOP();
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_status_register_rd() {
	if (ide_data_ready) {
		ZX_DATA_OUT(0b01001000); // data ready (and device ready?)
	} else {
		ZX_DATA_OUT(0b01000000); // device ready
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
}

void  __attribute__((section(".fast_code"))) divide_error_register_rd() {
	ZX_DATA_OUT(0); // no error
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
}

void  __attribute__((section(".fast_code"))) divide_command_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	if (data == 0xec) { // ATA_CMD_IDENTIFY
		ide_drive_buffer_pointer = 0;
		//ide_drive_buffer_endian_pointer = 1;
		ide_data_ready = 1;
	} else if (data == 0x20) { // ATA_CMD_READ_PIO
		// TODO
	} else {
		HANG_LOOP();
	}
	CLEAR_ZX_CONTROL_EXTI();
}


void copy_roms_to_ram() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	uint32_t i;
	// BANK 0
	// BANK 1
	// BANK 2
	for (i = 0; i < sizeof(didaktik_gama_88_rom) && i < 16384; i++) device_ram[i+2*16384] = didaktik_gama_88_rom[i];
	// BANK 3
	for (i = 0; i < sizeof(esxdos080_rom) && i < 16384; i++) device_ram[i+3*16384] = esxdos080_rom[i];
	//for (i = 0; i < sizeof(raiders_rom) && i < 16384; i++) device_ram[i] = raiders_rom[i];
	//for (i = 0; i < sizeof(zxtest_rom) && i < 16384; i++) device_ram[i+16384] = zxtest_rom[i];

	// by default, do not handle IO operations on any port
	for (i = 0; i < 256; i++) zx_io_rd_service_table[i] = zx_noop;
	for (i = 0; i < 256; i++) zx_io_wr_service_table[i] = zx_noop;
	// register non-ULA ports to noop
	for (i = 1; i < 256; i+=2) {
		zx_io_wr_service_table[i] = zx_noop;
		zx_io_rd_service_table[i] = zx_noop;
	}
	// register particular port handlers
	zx_io_wr_service_table[0x0e3] = divide_control_register_wr;
	zx_io_rd_service_table[0x0a3] = divide_data_register_rd;
	zx_io_wr_service_table[0x0a3] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0a7] = divide_error_register_rd;
	zx_io_wr_service_table[0x0a7] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0ab] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0ab] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0af] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0af] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0b3] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0b3] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0b7] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0b7] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0bb] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0bb] = divide_drive_head_register_wr;
	zx_io_rd_service_table[0x0bf] = divide_status_register_rd;
	zx_io_wr_service_table[0x0bf] = divide_command_register_wr;
	// 128k mapovani?
	zx_io_wr_service_table[0x0fd] = zx_noop; // ignore writes

}

void  __attribute__((section(".fast_code"))) zx_mem_rd_nonm1() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		register uint8_t data = device_ram[low_8k_rom_offset + address];
		ZX_DATA_OUT(data);
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else if (address < 0x4000) { // high 8kB of ROM area
		register uint8_t data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_mem_rd_m1() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		register uint8_t data = device_ram[low_8k_rom_offset + address];
		ZX_DATA_OUT(data);
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
		if ((address & 0xfff8) == 0x1ff8) {
				      divide_set_automap_off();
		} else if ((address == 0x0000) || (address == 0x0008) || (address == 0x0038)
				      || (address == 0x0066) || (address == 0x04c6) || (address == 0x0562)) {
				      divide_set_automap_on();
		}
	} else if (address < 0x4000) { // high 8kB of ROM area
		if ((address & 0xff00) == 0x3d00) {
		      divide_set_automap_on();
		}
		register uint8_t data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	}
	last_m1_addr = address;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_mem_wr() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = ZX_DATA_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		device_ram[low_8k_rom_offset + address] = data;
	} else if (address < 0x4000) { // high 8kB of ROM area
		device_ram[high_8k_rom_offset + address - 0x2000] = data;
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_io_rd() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	// decode ports using 8 bits only
	zx_io_rd_service_table[address & 0xff]();
}

void  __attribute__((section(".fast_code"))) zx_io_wr() {
	// TODO register
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
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


void __attribute__((section(".fast_code"))) EXTI9_5_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
}

void __attribute__((section(".fast_code"))) EXTI4_IRQHandler(void) {

	// read control lines (pins ZX_MEMREQ, ZX_IOREQ, ZX_WR, ZX_RD, ZX_M1)
	register uint32_t control = ZX_CONTROL_IN_GPIO_PORT->IDR & 0b111111;

	exti_service_table[control]();
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


	HAL_Delay(4000);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_Delay(4000);


	DEASSERT_ZX_RESET();
}
