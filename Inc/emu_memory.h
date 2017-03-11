/*
 * emu_memory.h
 *
 *  Created on: 19. 11. 2016
 *      Author: Kriz
 */

#ifndef EMU_MEMORY_H_
#define EMU_MEMORY_H_

#include "utils.h"

void FAST_CODE zx_mem_rd_nonm1_8k_block0();
void FAST_CODE zx_mem_rd_nonm1_8k_block1();
void FAST_CODE zx_mem_rd_m1_8k_block0();
void FAST_CODE zx_mem_rd_m1_8k_block1();
void FAST_CODE zx_mem_wr();

void emu_default_start();
void emu_memory_fill_zx_rom(const char didaktik_gama_89_mod_rom[], int length);
void emu_memory_page_zx_rom_in();

#endif /* EMU_MEMORY_H_ */
