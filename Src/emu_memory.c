/*
 * emu_memory.c
 *
 *  Created on: 19. 11. 2016
 *      Author: Kriz
 */

#include "emu_memory.h"

#include <stdint.h>

#include "roms/raiders_rom.h"
#include "roms/zxtest_rom.h"
#include "roms/esxide085_rom.h"
#include "roms/fatware014_rom.h"
#include "roms/uniper_splash_rom.h"

#include "zx_signals.h"
#include "utils.h"
#include "zx_rd_wr_interrupt.h"
#include "emu_divide_ports.h"

#include "../zx-soft/uniper_splash_common.h"

//0. 16kb = Divide RAM pages 0,1
//1. 16kb = Divide RAM pages 2,3
//2. 16kB = ZX ROM copy
//3. 16kB = Divide ROM (8kb)
//4. 16kB = Uniper Boot ROM
__attribute__((section(".begin_data1"))) uint8_t device_ram[16384*5];


// is DIVIDE memory currently mapped?
__attribute__((section(".begin_data1"))) uint8_t divide_mapped = 0;
// which page is mapped to the second 8kB
__attribute__((section(".begin_data1"))) uint8_t divide_page = 0;
// DIVIDE conmem flag
__attribute__((section(".begin_data1"))) uint8_t divide_conmem = 0;
// DIVIDE mapram flag
__attribute__((section(".begin_data1"))) uint8_t divide_mapram = 0;

#define DIVIDE_EEPROM_WRITABLE 1

__attribute__((section(".begin_data1"))) int divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
__attribute__((section(".begin_data1"))) int divide_highbank_writable = 1;


// offset in device_ram of the first 8kB mapped
__attribute__((section(".begin_data1"))) uint8_t *low_8k_rom_ptr;
// offset in device_ram of the second 8kB mapped
__attribute__((section(".begin_data1"))) uint8_t *high_8k_rom_minus_0x2000_ptr;
//
__attribute__((section(".begin_data1"))) uint8_t *high_8k_rom_divide_page_minus_0x2000_ptr;

__attribute__((section(".begin_data1"))) uint8_t *divide_eeprom_addr = device_ram + MEM_DIVIDE_ROM_OFFSET; // Divide ROM at startup

__attribute__((section(".begin_data1"))) int sniffing_rom_writes = 0;
__attribute__((section(".begin_data1"))) int sniffing_rom_writes_done = 0;


void emu_memory_fill_zx_rom(const char rom[], int length) {
	int i;
	for (i = 0; i < length && i < 0x4000; i++) device_ram[i + MEM_ZX_ROM_OFFSET] = rom[i];
}

void emu_memory_poke_uniper_rom(uint16_t addr, uint8_t value) {
	device_ram[addr + MEM_UNIPER_ROM_OFFSET] = value;
}

void emu_memory_page_divide_in() {
	// init pointers
	low_8k_rom_ptr = device_ram;
	high_8k_rom_minus_0x2000_ptr = device_ram;
}

void emu_memory_page_zx_rom_in() {
	// init pointers
	low_8k_rom_ptr = device_ram + MEM_ZX_ROM_OFFSET;
	high_8k_rom_minus_0x2000_ptr = device_ram + MEM_ZX_ROM_OFFSET;
}

void emu_memory_page_uniper_rom_in() {
	// init pointers
	low_8k_rom_ptr = device_ram + MEM_UNIPER_ROM_OFFSET;
	high_8k_rom_minus_0x2000_ptr = device_ram + MEM_UNIPER_ROM_OFFSET;
}

void emu_memory_fill_mem() {
	uint32_t i;
	// BANK 0
	// empty at startup
	// BANK 1
	// empty at startup
	// BANK 2
	// empty at startup, will be filled during uniper splash boot
	// BANK 3
	for (i = 0; i < sizeof(esxide085_rom) && i < 0x4000; i++) device_ram[i + MEM_DIVIDE_ROM_OFFSET] = esxide085_rom[i];
	// BANK 4
	// uniper splash ROM
	for (i = 0; i < sizeof(uniper_splash_rom) && i < 0x4000; i++) device_ram[i + MEM_UNIPER_ROM_OFFSET] = uniper_splash_rom[i];
	emu_memory_page_divide_in();
}

//void FAST_CODE __STATIC_INLINE divide_memory_set_map_on() {
//	low_8k_rom_offset = divide_eeprom_addr; // Divide ROM or Divide Bank 3 RAM
//	high_8k_rom_offset = divide_page*0x2000; // Divide RAM page (8kB) 0..3
//	divide_mapped = 1;
//}
//
//void FAST_CODE __STATIC_INLINE divide_memory_set_map_off() {
//	if (!divide_conmem) { // do not unmap when conmem == true
//		low_8k_rom_offset = 2*0x4000; // ZX ROM copy (low 8kB)
//		high_8k_rom_offset = 2*0x4000 + 0x2000; // ZX ROM copy (high 8kB)
//		divide_mapped = 0;
//	}
//}

#define divide_memory_set_map_on_1() { \
		high_8k_rom_minus_0x2000_ptr = high_8k_rom_divide_page_minus_0x2000_ptr; \
}

#define divide_memory_set_map_on_2() { \
	low_8k_rom_ptr = divide_eeprom_addr; \
	divide_mapped = 1; \
}

//void divide_memory_set_map_on_2() {
//	low_8k_rom_ptr = device_ram + divide_eeprom_addr;
//	divide_mapped = 1;
//}

#define divide_memory_set_map_off() { \
	if (!divide_conmem) { \
		low_8k_rom_ptr = device_ram + MEM_ZX_ROM_OFFSET; \
		high_8k_rom_minus_0x2000_ptr = device_ram + MEM_ZX_ROM_OFFSET; \
		divide_mapped = 0; \
	} \
}

void FAST_CODE zx_mem_rd_nonm1_8k_block0() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = low_8k_rom_ptr[address];
	ZX_DATA_OUT(data);
	//DEASSERT_ZX_WAIT();
	while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_RD_EXTI();
}

void FAST_CODE zx_mem_rd_nonm1_8k_block1() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = high_8k_rom_minus_0x2000_ptr[address];
	ZX_DATA_OUT(data);
	//DEASSERT_ZX_WAIT();
	while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_RD_EXTI();
}

void FAST_CODE zx_mem_rd_m1_8k_block0() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = low_8k_rom_ptr[address];
	ZX_DATA_OUT(data);
	//DEASSERT_ZX_WAIT();
	while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if ((address & 0xfff8) == 0x1ff8) {
				  divide_memory_set_map_off();
	} else if ((address == 0x0000) || (address == 0x0008) || (address == 0x0038)
				  || (address == 0x0066) || (address == 0x04c6) || (address == 0x0562)) {
				  divide_memory_set_map_on_1();
				  divide_memory_set_map_on_2();
	}
	CLEAR_ZX_RD_EXTI();
}

void FAST_CODE zx_mem_rd_m1_8k_block1() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if ((address & 0xff00) == 0x3d00) {
		divide_memory_set_map_on_1(); // do the minimum work here to obtain valid data (address)
	}
	// this is here due to gcc's way to compile this
	register uint8_t data = high_8k_rom_minus_0x2000_ptr[address];
	ZX_DATA_OUT(data);
	//DEASSERT_ZX_WAIT();
	while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if ((address & 0xff00) == 0x3d00) {
		divide_memory_set_map_on_2(); // do the rest of the work
	}
	CLEAR_ZX_RD_EXTI();
}

void FAST_CODE zx_mem_wr() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = ZX_DATA_GPIO_PORT->IDR;
	if (divide_mapped) { // ignore writes to ROM when Divide not mapped
		if ((address < 0x2000) && divide_lowbank_writable) { // low 8kB of ROM area
			low_8k_rom_ptr[address] = data;
		} else if ((address < 0x4000) && divide_highbank_writable) { // high 8kB of ROM area
			high_8k_rom_minus_0x2000_ptr[address] = data;
		}
	}
	CLEAR_ZX_WR_EXTI();
}

void FAST_CODE divide_control_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	//last_port_data = data;
	divide_page = data & 0b11;
	high_8k_rom_divide_page_minus_0x2000_ptr = device_ram + divide_page*0x2000 - 0x2000;
	divide_conmem = data & 0x10000000; // CONMEM bit
	if (divide_conmem) {
		divide_eeprom_addr = device_ram + MEM_DIVIDE_ROM_OFFSET; // Divide ROM
		divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
		divide_highbank_writable = 1; // Divide RAM writable
	} else {
		if (divide_mapram || (data & 0b01000000)) { // MAPRAM
			divide_mapram = 1;
			divide_eeprom_addr = device_ram + 3*0x2000; // Divide RAM bank 3 (8kB)
			divide_lowbank_writable = 0;
			divide_highbank_writable = (divide_page != 3); // Divide RAM writable only when not bank 3
		} else {
			divide_eeprom_addr = device_ram + MEM_DIVIDE_ROM_OFFSET; // Divide ROM
			divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
			divide_highbank_writable = 1; // Divide RAM writable
		}
	}
	if (divide_mapped || divide_conmem) {
		divide_memory_set_map_on_1(); // recalculate offsets for a new RAM page
		divide_memory_set_map_on_2();
	} else {
		divide_memory_set_map_off(); // unmap divide memory (in case divide_conmem has been turned off)
	}
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE zx_mem_rd_16k_sniff() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = ZX_DATA_GPIO_PORT->IDR;
	register uint8_t data2 = data;
	register uint8_t data3 = data;
	if (!sniffing_rom_writes) {
		register uint8_t data = device_ram[address + MEM_UNIPER_ROM_OFFSET];
		ZX_DATA_OUT(data);
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else {
		// make sure we read the data-bus when data from ROM is present,
		// so we read multiple times and wait just before MEMREQ+RD is deasserted
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) {
			data3 = data2;
			data2 = data;
			data = ZX_DATA_GPIO_PORT->IDR;
		}
		device_ram[address + MEM_ZX_ROM_OFFSET] = data3;
	}
	CLEAR_ZX_RD_EXTI();
}

void FAST_CODE zx_port_254_wr_sniff() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0b111;
	if (data == ZX_SPLASH_ROMREAD_START_BORDER) {
		sniffing_rom_writes = 1;
		HIGHZ_ZX_ROMCS(); // let physical ZX ROM to be paged-in by ULA when necessary
	} else if (data == ZX_SPLASH_ROMREAD_END_BORDER) {
		sniffing_rom_writes = 0;
		sniffing_rom_writes_done = 1;
		DEASSERT_ZX_ROMCS(); // disable physical ZX ROM permanently
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void emu_wait_until_boot_done() {
	while (!sniffing_rom_writes_done) { }
}

void emu_default_start() {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_mem();

	// register all ZX control lines handlers

	clear_zx_control_lines_handlers();

	register_zx_control_lines_handler(ZX_RD_Pin | ZX_IOREQ_Pin, ZX_WR_Pin | ZX_MEMREQ_Pin, zx_io_rd);
	register_zx_control_lines_handler(ZX_WR_Pin | ZX_IOREQ_Pin, ZX_RD_Pin | ZX_MEMREQ_Pin, zx_io_wr);

	register_zx_control_lines_handler(ZX_WR_Pin | ZX_MEMREQ_Pin, ZX_RD_Pin | ZX_IOREQ_Pin, zx_mem_wr);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 0, zx_mem_rd_nonm1_8k_block0);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 1, zx_mem_rd_nonm1_8k_block1);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 0, zx_mem_rd_m1_8k_block0);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 1, zx_mem_rd_m1_8k_block1);

	// register particular port handlers
	clear_zx_port_handlers();
	register_zx_port_write(0x0e3, divide_control_register_wr);
	emu_divide_ports_init();

	HAL_Delay(50);

	// start ZX
	DEASSERT_ZX_RESET();
}
