#ifndef __ZX_IF_H_
#define __ZX_IF_H_

#define ZX_ADDR() (LPC_GPIO_PORT->MPIN[2])
#define ZX_DATA() (LPC_GPIO_PORT->MPIN[3])
#define ZX_IS_MEM_READ() ((LPC_GPIO_PORT->PIN[3] & 0b010100000000) == 0)
#define ZX_IS_IO_READ() ((LPC_GPIO_PORT->PIN[3] & 0b011000000000) == 0)
#define ZX_IS_RESET() ((LPC_GPIO_PORT->PIN[3] & 0b1000000000000) == 0)
//#define ZX_ROM_PAGE_OUT() {LPC_GPIO_PORT->DIR[3] |= (1UL << 13); LPC_GPIO_PORT->PIN[3] |= (1UL << 13);} // output HIGH
//#define ZX_ROM_PAGE_IN() {LPC_GPIO_PORT->DIR[3] |= (1UL << 13); LPC_GPIO_PORT->PIN[3] |= ~(1UL << 13);} // output LOW
#define ZX_ROM_PAGE_OUT() {LPC_GPIO_PORT->DIR[3] |= (1UL << 13); Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 13);} // output HIGH
#define ZX_ROM_PAGE_IN() {LPC_GPIO_PORT->DIR[3] |= (1UL << 13); Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 13);} // output LOW
#define ZX_ROM_ULA_ROMCS() {LPC_GPIO_PORT->DIR[3] &= ~(1UL << 13);} // hi-Z
#define ZX_DATA_OUT(data) {LPC_GPIO_PORT->MPIN[3] = data; LPC_GPIO_PORT->DIR[3] |= 0xffUL;  }
// LPC_GPIO_PORT->PIN[3] = (((LPC_GPIO_PORT->PIN[3]) & 0xff00UL) | data);
#define ZX_DATA_HI_Z() {LPC_GPIO_PORT->DIR[3] &= ~0xffUL; }

#define ZX_ASSERT_RESET() {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 12);}
#define ZX_DEASSERT_RESET() {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 12);}

#define ZX_DATA_READY_FLAG_ON() {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 14);}
#define ZX_DATA_READY_FLAG_OFF() {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 14);}

#define ZX_ASSERT_WAIT() {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 15);}
#define ZX_DEASSERT_WAIT() {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 15);}

#define PININT_INDEX_RD   0	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER_RD  GPIO0_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME_RD    PIN_INT0_IRQn	/* GPIO interrupt NVIC interrupt name */

#define PININT_INDEX_WR   1	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER_WR  GPIO1_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME_WR    PIN_INT1_IRQn	/* GPIO interrupt NVIC interrupt name */

// 0xa3=163
#define ZX_DATA_PORT 0xa3
// 0xe3=227
#define ZX_CONTROL_PORT 0xe3

void zx_interface_init();

#endif
