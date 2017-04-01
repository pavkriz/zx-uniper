/*
 * zx_signals.h
 *
 *  Created on: 19. 11. 2016
 *      Author: Kriz
 */

#ifndef ZX_SIGNALS_H_
#define ZX_SIGNALS_H_

#include "stm32f7xx_hal.h"

#define ZX_ADDR_GPIO_PORT GPIOE
#define ZX_DATA_GPIO_PORT GPIOB
#define ZX_CONTROL_IN_GPIO_PORT GPIOD

// send byte to Z80's data bus
#define ZX_DATA_OUT(data) {ZX_DATA_GPIO_PORT->BSRR = (uint32_t)0xff << 16; ZX_DATA_GPIO_PORT->MODER = (uint32_t)0b10100000000000000101010101010101;  ZX_DATA_GPIO_PORT->BSRR = data;  } // ala UNICARD, ale MODER natvrdo misto |=, MODER doprostred
// switch data bus to high impedance
#define ZX_DATA_HI_Z() {ZX_DATA_GPIO_PORT->MODER = (uint32_t)0b10100000000000000000000000000000;}

// test if Z80 is doing memory read
#define ZX_IS_MEM_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_MEMREQ_Pin)) == 0)
// test if Z80 is doing I/O read
#define ZX_IS_IO_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_IOREQ_Pin)) == 0)
#define ZX_IS_IO_WRITE(control_lines) (((uint16_t)control_lines & (ZX_WR_Pin | ZX_IOREQ_Pin)) == 0)

// must be called at the end of EVERY operation called from RD/WR interrupt
#define CLEAR_ZX_CONTROL_EXTI() {__HAL_GPIO_EXTI_CLEAR_IT(ZX_RD_Pin);__HAL_GPIO_EXTI_CLEAR_IT(ZX_WR_Pin);}
#define CLEAR_ZX_RD_EXTI() {__HAL_GPIO_EXTI_CLEAR_IT(ZX_RD_Pin);}
#define CLEAR_ZX_WR_EXTI() {__HAL_GPIO_EXTI_CLEAR_IT(ZX_WR_Pin);}

#define DEASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = ZX_RESET_Pin;}
#define ASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = (uint32_t)ZX_RESET_Pin << 16;}

#define DEASSERT_ZX_ROMCS() { ZX_ROMCS_GPIO_Port->MODER |= (uint32_t)0b01000000000000000000000000000000; ZX_ROMCS_GPIO_Port->BSRR = ZX_ROMCS_Pin; }
#define HIGHZ_ZX_ROMCS() { ZX_ROMCS_GPIO_Port->MODER &= (uint32_t)0b00111111111111111111111111111111; }

#endif /* ZX_SIGNALS_H_ */
