/*
 * uniper_splash.c
 *
 *  Created on: 11. 3. 2017
 *      Author: mazlik
 */

#include "uniper_splash.h"

#include "zx_rd_wr_interrupt.h"
#include "emu_memory.h"
#include "zx_signals.h"

void uniper_splash_finish(zx_control_handler_t port_254_wr_handler) {
	// register all ZX control lines handlers

	clear_zx_control_lines_handlers();

	register_zx_control_lines_handler(ZX_RD_Pin | ZX_IOREQ_Pin, ZX_WR_Pin | ZX_MEMREQ_Pin, zx_io_rd);
	register_zx_control_lines_handler(ZX_WR_Pin | ZX_IOREQ_Pin, ZX_RD_Pin | ZX_MEMREQ_Pin, zx_io_wr);

	register_zx_control_lines_handler(ZX_WR_Pin | ZX_MEMREQ_Pin, ZX_RD_Pin | ZX_IOREQ_Pin, zx_mem_wr);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 0, zx_mem_rd_16k_sniff);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin, ZX_WR_Pin | ZX_IOREQ_Pin | ZX_M1_Pin, 1, zx_mem_rd_16k_sniff);

	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 0, zx_mem_rd_16k_sniff);
	register_zx_control_lines_block_handler(ZX_RD_Pin | ZX_MEMREQ_Pin | ZX_M1_Pin, ZX_WR_Pin | ZX_IOREQ_Pin, 1, zx_mem_rd_16k_sniff);

	// register no port handlers
	clear_zx_port_handlers();

	register_zx_port_write(254, port_254_wr_handler);

	HAL_Delay(50);

	// start ZX
	DEASSERT_ZX_RESET();
}

void uniper_splash_show_boot() {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_mem();

	emu_memory_poke_uniper_rom(ZX_SPLASH_CODE_ROM_ADDR, ZX_SPLASH_CODE_BOOT);
	emu_memory_page_uniper_rom_in();

	// ... start ZX
	uniper_splash_finish(zx_port_254_wr_sniff);
}

void uniper_splash_bsod(uint8_t message_code, uint16_t param1) {
	// stop ZX
	ASSERT_ZX_RESET();

	emu_memory_fill_mem();

	emu_memory_poke_uniper_rom(ZX_SPLASH_CODE_ROM_ADDR, message_code);
	emu_memory_poke_uniper_rom(ZX_SPLASH_PARAM1_ROM_ADDR, param1 & 0xff);	// param1 low byte
	emu_memory_poke_uniper_rom(ZX_SPLASH_PARAM1_ROM_ADDR + 1, (param1 & 0xff00) >> 8); // param1 high byte
	emu_memory_page_uniper_rom_in();

	// ... start ZX
	uniper_splash_finish(zx_noop);
}
