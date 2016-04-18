#ifndef __ZX_IF_H_
#define __ZX_IF_H_

#define ZX_ADDR() (LPC_GPIO_PORT->PIN[2])
#define ZX_DATA() (LPC_GPIO_PORT->PIN[3])
#define ZX_IS_MEM_READ() ((LPC_GPIO_PORT->PIN[3] & 0b010100000000) == 0)
#define ZX_IS_RESET() ((LPC_GPIO_PORT->PIN[3] & 0b1000000000000) == 0)

#define PININT_INDEX_RD   0	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER_RD  GPIO0_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME_RD    PIN_INT0_IRQn	/* GPIO interrupt NVIC interrupt name */

#define PININT_INDEX_WR   1	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER_WR  GPIO1_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME_WR    PIN_INT1_IRQn	/* GPIO interrupt NVIC interrupt name */



void zx_interface_init();

#endif
