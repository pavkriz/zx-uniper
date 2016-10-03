/*
 * rd_wr_interrupt.h
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#ifndef ZX_RD_WR_INTERRUPT_H_
#define ZX_RD_WR_INTERRUPT_H_

void zx_init_pins(void);
void zx_rd_wr_handler(void);
void zx_mem_emu_loop(void);
void copy_roms_to_ram();

#endif /* ZX_RD_WR_INTERRUPT_H_ */
