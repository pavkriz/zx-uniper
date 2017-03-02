/*
 * rd_wr_interrupt.h
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#ifndef ZX_RD_WR_INTERRUPT_H_X_
#define ZX_RD_WR_INTERRUPT_H_X_

#include <stdint.h>

#include "utils.h"

void zx_init_pins(void);
void zx_rd_wr_handler(void);
void zx_mem_emu_loop(void);
void copy_roms_to_ram();

typedef void (*zx_control_handler_t)(void);

void register_zx_port_write(uint8_t port, zx_control_handler_t handler);
void register_zx_port_read(uint8_t port, zx_control_handler_t handler);

void FAST_CODE io_wr_hang_loop();
void FAST_CODE io_rd_hang_loop();

#endif /* ZX_RD_WR_INTERRUPT_H_X_ */
