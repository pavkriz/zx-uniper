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

void emu_memory_init();

#endif /* EMU_MEMORY_H_ */
