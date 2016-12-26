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
#include "roms/didaktik_gama_89_mod_rom.h"
#include "roms/fatware014_rom.h"

#include "zx_signals.h"
#include "utils.h"
#include "zx_rd_wr_interrupt.h"

// offset in device_ram of the first 8kB mapped
int32_t low_8k_rom_offset = 0;
// offset in device_ram of the second 8kB mapped
int32_t high_8k_rom_offset = 0;

// is DIVIDE memory currently mapped?
uint8_t divide_mapped = 0;
// which page is mapped to the second 8kB
uint8_t divide_page = 0;
// DIVIDE conmem flag
uint8_t divide_conmem = 0;
// DIVIDE mapram flag
uint8_t divide_mapram = 0;

#define DIVIDE_EEPROM_WRITABLE 1

int divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
int divide_highbank_writable = 1;

//0. 16kb = Divide RAM pages 0,1
//1. 16kb = Divide RAM pages 2,3
//2. 16kB = ZX ROM copy
//3. 16kB = Divide ROM (8kb)
uint8_t device_ram[16384*4];
int divide_eeprom_addr = 3*0x4000; // Divide ROM at startup

void emu_memory_fill_mem() {
	uint32_t i;
	// BANK 0
	// empty at startup
	// BANK 1
	// empty at startup
	// BANK 2
	for (i = 0; i < sizeof(didaktik_gama_89_mod_rom) && i < 16384; i++) device_ram[i+2*16384] = didaktik_gama_89_mod_rom[i];
	// BANK 3
	for (i = 0; i < sizeof(esxide085_rom) && i < 16384; i++) device_ram[i+3*16384] = esxide085_rom[i];
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
	high_8k_rom_offset = divide_page*0x2000; \
}

#define divide_memory_set_map_on_2() { \
	low_8k_rom_offset = divide_eeprom_addr; \
	divide_mapped = 1; \
}

#define divide_memory_set_map_off() { \
	if (!divide_conmem) { \
		low_8k_rom_offset = 2*0x4000; \
		high_8k_rom_offset = 2*0x4000 + 0x2000; \
		divide_mapped = 0; \
	} \
}

void FAST_CODE /*__attribute__((naked))*/ zx_mem_rd_nonm1() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		register uint8_t data = device_ram[low_8k_rom_offset + address];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else if (address < 0x4000) { // high 8kB of ROM area
		register uint8_t data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else {
		//DEASSERT_ZX_WAIT();
	}
	CLEAR_ZX_CONTROL_EXTI();
	//RETURN_FROM_NAKED_ISR();
}

void FAST_CODE /*__attribute__((naked))*/ zx_mem_rd_m1() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data;
	if (address < 0x2000) { // low 8kB of ROM area
		data = device_ram[low_8k_rom_offset + address];
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
	} else if (address < 0x3d00) { // high 8kB of ROM area, mapper's non-exit area
		data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else if (address < 0x4000) { // high 8kB of ROM area, mapper's exit area
		divide_memory_set_map_on_1(); // do the minimum work here to obtain valid data (address)
		// this is here due to gcc's way to compile this
		data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
		divide_memory_set_map_on_2(); // do the rest of the work
	//} else {
		//DEASSERT_ZX_WAIT();
	}
	//last_m1_addr = address;
	CLEAR_ZX_CONTROL_EXTI();
	//RETURN_FROM_NAKED_ISR();
}

void FAST_CODE zx_mem_wr() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = ZX_DATA_GPIO_PORT->IDR;
	if (divide_mapped) { // ignore writes to ROM when Divide not mapped
		if ((address < 0x2000) && divide_lowbank_writable) { // low 8kB of ROM area
			device_ram[low_8k_rom_offset + address] = data;
		} else if ((address < 0x4000) && divide_highbank_writable) { // high 8kB of ROM area
			device_ram[high_8k_rom_offset + address - 0x2000] = data;
		}
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_control_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	//last_port_data = data;
	divide_page = data & 0b11;
	divide_conmem = data & 0x10000000; // CONMEM bit
	if (divide_conmem) {
		divide_eeprom_addr = 3*0x4000; // Divide ROM
		divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
		divide_highbank_writable = 1; // Divide RAM writable
	} else {
		if (divide_mapram || (data & 0b01000000)) { // MAPRAM
			divide_mapram = 1;
			divide_eeprom_addr = 3*0x2000; // Divide RAM bank 3 (8kB)
			divide_lowbank_writable = 0;
			divide_highbank_writable = (divide_page != 3); // Divide RAM writable only when not bank 3
		} else {
			divide_eeprom_addr = 3*0x4000; // Divide ROM
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
	CLEAR_ZX_CONTROL_EXTI();
}

void emu_memory_init() {
	emu_memory_fill_mem();
	register_zx_port_write(0x0e3, divide_control_register_wr);
}
