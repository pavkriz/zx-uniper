/*
 * uniper_splash.c
 *
 *  Created on: 11. 3. 2017
 *      Author: mazlik
 */

#include "roms/uniper_splash_rom.h"
#include "zx_rd_wr_interrupt.h"
#include "emu_memory.h"
#include "zx_signals.h"

void uniper_splash_show() {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_zx_rom(uniper_splash_rom, sizeof(uniper_splash_rom));
	emu_memory_page_zx_rom_in();

	// register all ZX control lines handlers

	clear_zx_control_lines_handlers();

	register_zx_control_lines_handler(ZX_RD_Pin | ZX_IOREQ_Pin, ZX_WR_Pin | ZX_MEMREQ_Pin, zx_io_rd);
	register_zx_control_lines_handler(ZX_WR_Pin | ZX_IOREQ_Pin, ZX_RD_Pin | ZX_MEMREQ_Pin, zx_io_wr);

	register_zx_control_lines_handler(ZX_WR_Pin | ZX_MEMREQ_Pin, ZX_RD_Pin | ZX_IOREQ_Pin, zx_mem_wr);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 0, zx_mem_rd_nonm1_8k_block0);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 1, zx_mem_rd_nonm1_8k_block1);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 0, zx_mem_rd_nonm1_8k_block0);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 1, zx_mem_rd_nonm1_8k_block1);

	// register no port handlers
	clear_zx_port_handlers();

	HAL_Delay(50);

	// start ZX
	DEASSERT_ZX_RESET();
}
