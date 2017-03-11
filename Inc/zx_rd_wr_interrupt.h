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

typedef void (*zx_control_handler_t)(void);

void clear_zx_port_handlers();
void register_zx_port_write(uint8_t port, zx_control_handler_t handler);
void register_zx_port_read(uint8_t port, zx_control_handler_t handler);

void clear_zx_control_lines_handlers();
void register_zx_control_lines_handler(uint16_t lines_to_be_low, uint16_t lines_to_be_high, zx_control_handler_t handler);
void register_zx_control_lines_block_handler(uint16_t lines_to_be_low, uint16_t lines_to_be_high, int num_8k_block, zx_control_handler_t handler);

void FAST_CODE io_wr_hang_loop();
void FAST_CODE io_rd_hang_loop();
void FAST_CODE zx_io_rd();
void FAST_CODE zx_io_wr();

#define DEASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = ZX_RESET_Pin;}
#define ASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = (uint32_t)ZX_RESET_Pin << 16;}


#endif /* ZX_RD_WR_INTERRUPT_H_X_ */
