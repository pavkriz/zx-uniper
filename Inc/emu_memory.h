/*
 * emu_memory.h
 *
 *  Created on: 19. 11. 2016
 *      Author: Kriz
 */

#ifndef EMU_MEMORY_H_
#define EMU_MEMORY_H_

#include <stdint.h>

#include "utils.h"

// Memory map:
//0. 16kb = Divide RAM pages 0,1
//1. 16kb = Divide RAM pages 2,3
//2. 16kB = ZX ROM copy
//3. 16kB = Divide ROM (8kb)
//4. 16kB = Uniper Splash ROM
#define MEM_ZX_ROM_OFFSET		2*0x4000
#define MEM_DIVIDE_ROM_OFFSET	3*0x4000
#define MEM_UNIPER_ROM_OFFSET	4*0x4000

void FAST_CODE zx_mem_rd_nonm1_8k_block0();
void FAST_CODE zx_mem_rd_nonm1_8k_block1();
void FAST_CODE zx_mem_rd_m1_8k_block0();
void FAST_CODE zx_mem_rd_m1_8k_block1();
void FAST_CODE zx_mem_wr();
void FAST_CODE zx_mem_rd_16k_sniff();
void FAST_CODE zx_port_254_wr_sniff();

void emu_default_start();
void emu_memory_fill_zx_rom(const char rom[], int length);
void emu_memory_poke_rom(uint16_t addr, uint8_t value);
void emu_memory_page_zx_rom_in();
void emu_memory_page_uniper_rom_in();
void emu_wait_until_boot_done();

#endif /* EMU_MEMORY_H_ */
