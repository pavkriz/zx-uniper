/*
 * uniper_splash.c
 *
 *  Created on: 11. 3. 2017
 *      Author: mazlik
 */

#include "uniper_splash.h"

#include "roms/uniper_splash_rom.h"
#include "zx_rd_wr_interrupt.h"
#include "emu_memory.h"
#include "zx_signals.h"

void uniper_splash_finish() {
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

void uniper_splash_show_boot() {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_zx_rom(uniper_splash_rom, sizeof(uniper_splash_rom));
	emu_memory_poke_rom(ZX_SPLASH_CODE_ROM_ADDR, ZX_SPLASH_CODE_BOOT);
	emu_memory_page_zx_rom_in();

	// ... start ZX
	uniper_splash_finish();
}

void uniper_splash_bsod(uint8_t message_code, uint16_t param1) {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_zx_rom(uniper_splash_rom, sizeof(uniper_splash_rom));
	emu_memory_poke_rom(ZX_SPLASH_CODE_ROM_ADDR, message_code);
	emu_memory_poke_rom(ZX_SPLASH_PARAM1_ROM_ADDR, param1 & 0xff);	// param1 low byte
	emu_memory_poke_rom(ZX_SPLASH_PARAM1_ROM_ADDR + 1, (param1 & 0xff00) >> 8); // param1 high byte
	emu_memory_page_zx_rom_in();

	// ... start ZX
	uniper_splash_finish();
}
