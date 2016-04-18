#include "chip.h"
#include "zx_if.h"

#define SetDigitalInput(pinX, pinY) Chip_SCU_PinMuxSet(pinX, pinY, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));	// Px_y as digital GPIO
#define SetDigitalInputDataBus(pinX, pinY) Chip_SCU_PinMuxSet(pinX, pinY, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));	// Px_y as digital GPIO

void zx_interface_init() {
	// GPIO2 (16bit) = ADDR -> input
	LPC_GPIO_PORT->DIR[2] = 0;
	SetDigitalInput(4,0);
	SetDigitalInput(4,1);
	SetDigitalInput(4,2);
	SetDigitalInput(4,3);
	SetDigitalInput(4,4);
	SetDigitalInput(4,5);
	SetDigitalInput(4,6);
	SetDigitalInput(5,7);
	SetDigitalInput(6,12);
	SetDigitalInput(5,0);
	SetDigitalInput(5,1);
	SetDigitalInput(5,2);
	SetDigitalInput(5,3);
	SetDigitalInput(5,4);
	SetDigitalInput(5,5);
	SetDigitalInput(5,6);
	// GPIO3 (8bit) = DATA -> input
	// GPIO3 (bits 8,9,10,11) = MEMRQ,IORQ,RD,WR -> input
	//LPC_GPIO_PORT->DIR[3] &= ~(0b111111111111);
	LPC_GPIO_PORT->DIR[3] = 0;
	// set Px_y pins as digital input
	// DATA 0..7
	SetDigitalInputDataBus(6,1);
	SetDigitalInputDataBus(6,2);
	SetDigitalInputDataBus(6,3);
	SetDigitalInputDataBus(6,4);
	SetDigitalInputDataBus(6,5);
	SetDigitalInputDataBus(6,9);
	SetDigitalInputDataBus(6,10);
	SetDigitalInputDataBus(6,11);
	// MEMREQ
	SetDigitalInput(7,0);
	// IO REQ
	SetDigitalInput(7,1);
	// RD
	SetDigitalInput(7,2);
	// WR
	SetDigitalInput(7,3);
	// RESET (+pullup)
	Chip_SCU_PinMuxSet(7, 4, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
	//SetDigitalOutput(7,4);
	SetDigitalInput(7,5);
	SetDigitalInput(7,6);
	SetDigitalInput(7,7);

	/* Configure RD interrupt channel for the GPIO3[10] pin in SysCon block */
	Chip_SCU_GPIOIntPinSel(PININT_INDEX_RD, 3, 10);

	/* Configure WR interrupt channel for the GPIO3[11] pin in SysCon block */
	Chip_SCU_GPIOIntPinSel(PININT_INDEX_WR, 3, 11);

	/* Configure channel interrupt for RD as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_RD));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_RD));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_RD));

	/* Configure channel interrupt for WR as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_WR));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_WR));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_WR));

	/* Enable RD interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PININT_NVIC_NAME_RD);
	NVIC_EnableIRQ(PININT_NVIC_NAME_RD);

	/* Enable WR interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PININT_NVIC_NAME_WR);
	NVIC_EnableIRQ(PININT_NVIC_NAME_WR);
}
