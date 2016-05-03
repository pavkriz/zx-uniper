/*
 * @brief UART interrupt example with ring buffers
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"
//#include "board.h"
#include "string.h"
#include "uart.h"
#include "zx_if.h"
#include "leds.h"
#include "qrom.h"
#include "raiders.h"
#include "zxtest.h"
#include "mem_loader.h"
#include "rtype_scr.h"



#define MY_ROM_SIZE 24
uint8_t myrom[MY_ROM_SIZE] = {0xf3, 0x3e, 0x02, 0xd3, 0xfe, 0x06, 0xff, 0x10,         0xfe, 0x3e, 0x00, 0xd3, 0xfe, 0x06, 0xff, 0x10,
				0xfe, 0xc3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 		};


typedef struct {
	uint16_t address;
	uint32_t data;
	uint32_t operation_length;
} memory_operation_t;

#define MEMORY_OPERATIONS_LENGTH 200
memory_operation_t memory_operations[MEMORY_OPERATIONS_LENGTH];

int operations = 0;

void dump_memory_array(void) {
	int i;
	for (i = 0; i < MEMORY_OPERATIONS_LENGTH; i++) {
		//if (ZX_IS_RESET()) goto RESET;
		UART_printf("MEM[%04d] addr=0x%04x data=0x%04x length=%d\r\n",
				i,
				memory_operations[i].address, memory_operations[i].data,
				memory_operations[i].operation_length);
	}
}

void PININT_IRQ_HANDLER_RD_xxxx(void)
{
	// now 12 cycles (ie. 59ns @ 204HMz) elapsed in Core-M4 since interrupt event arose
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_RD));
	// handle interrupt
	// we assume the MEMRQ, IORQ, RD and WR lines are stable here in order to read them and decode the operation
	if (ZX_IS_MEM_READ()) {
		// Z80 is doing the memory-read
		// ZX's 27128 EPROM may read data in max 450ns (since ~G signal) or max 150ns (since ~E signal)
		// if we want read (sniff) the data loaded from ZX memory to DATA bus, we have to wait a little
		// tweak the number of NOPs here to sniff the stable data from the DATA bus
		// 25 NOPs = 122ns @ 204MHz
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		// now we assume the stable data are on the DATA bus
		// we are going to sniff them
		int address = ZX_ADDR();
		int data = ZX_DATA();
		int operation_length = 0;
		while (ZX_IS_MEM_READ()) {
			operation_length++;
			//UART_printf("waiting %d\r\n", operation_length);
		}
		memory_operations[operations].address = address;
		memory_operations[operations].data = data;
		memory_operations[operations].operation_length = operation_length;
		operations++;
		if (operations >= MEMORY_OPERATIONS_LENGTH) {
			// disable RD/WR interrupts and dump the array
			NVIC_DisableIRQ(PININT_NVIC_NAME_RD);
			NVIC_DisableIRQ(PININT_NVIC_NAME_WR);
			dump_memory_array();
		}
	}
}

__attribute__ ((__section__(".data.ramfunc")))
void PININT_IRQ_HANDLER_RD_xxx(void)
{
	// now 12 cycles (ie. 59ns @ 204HMz) elapsed in Core-M4 since interrupt event arose
	// handle interrupt
	// we assume the MEMRQ, IORQ, RD and WR lines are stable here in order to read them and decode the operation
	int address;
//	if (ZX_IS_MEM_READ() && ((address = ZX_ADDR()) == 0x1542)) {
	if (ZX_IS_MEM_READ() && ((address = ZX_ADDR()) < MY_ROM_SIZE)) {
		// Z80 is doing the memory-read
		//ZX_ROM_PAGE_OUT();
		int data;
		//if (address < MY_ROM_SIZE) {
			data = myrom[address];
		//} else {
//			data = 0;
		//}
		ZX_DATA_OUT(data);
		ZX_DATA_READY_FLAG_ON();
		int operation_length = 0;
		while (ZX_IS_MEM_READ()) {
			//operation_length++;
		}
		ZX_DATA_HI_Z();
		ZX_DATA_READY_FLAG_OFF();
		//ZX_ROM_ULA_ROMCS();
		//UART_printf("cnt=%d\r\n", cnt);
//		memory_operations[operations].address = address;
//		memory_operations[operations].data = data;
//		memory_operations[operations].operation_length = operation_length;
//		operations++;
//		if (operations >= MEMORY_OPERATIONS_LENGTH) {
//			ZX_ROM_ULA_ROMCS();
//			// disable RD/WR interrupts and dump the array
//			NVIC_DisableIRQ(PININT_NVIC_NAME_RD);
//			NVIC_DisableIRQ(PININT_NVIC_NAME_WR);
//			LED1_OFF();
//			LED2_ON();
//			dump_memory_array();
//		}
	}
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_RD));
}

void PININT_IRQ_HANDLER_WR(void)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(PININT_INDEX_WR));
	// handle interrupt
}

__attribute__ ((__section__(".data.ramfunc")))
int main(void)
{
	SystemCoreClockUpdate();

	/* Enable Flash acceleration and setup wait states */
	Chip_CREG_SetFlashAcceleration(MAX_CLOCK_FREQ);

	Chip_SetupCoreClock(CLKIN_CRYSTAL, MAX_CLOCK_FREQ, true);
	//Chip_GPIO_Init(LPC_GPIO_PORT);
	UART_Init();
	leds_init();
	zx_interface_init();

	LED1_ON();
	LED2_OFF();

	ZX_ROM_PAGE_OUT();
	ZX_DATA_HI_Z();
	//ZX_ROM_ULA_ROMCS();

	//UART_printf("Started\r\n");
	int w;
	volatile int xx;


//	while (1) {
//		if (Chip_GPIO_GetPinState(LPC_GPIO_PORT,3,9)) {
//			LED1_ON();
//		} else {
//			LED1_OFF();
//		}
//	}

//	while (1) {
//		int data = ZX_DATA();
//		int address = ZX_ADDR();
//		UART_printf("MEM addr=0x%04x data=0x%04x\r\n",	address, data);
//	}

	ZX_ASSERT_RESET();
	for (w = 0; w < 10000000; w++) { xx++; }
	ZX_DEASSERT_RESET();

	LED1_OFF();
	LED2_ON();

	int address;
	int data;
	int data_index = 0;
	volatile int wt;
	while (1) {
		if (ZX_IS_MEM_READ()) {
			if ((address = ZX_ADDR()) < sizeof(mem_loader)) {
				ZX_ASSERT_WAIT();
				data = mem_loader[address];
				ZX_DATA_OUT(data);
				ZX_DATA_READY_FLAG_ON();
				ZX_DEASSERT_WAIT();
				while (ZX_IS_MEM_READ()) { }
				ZX_DATA_HI_Z();
				ZX_DATA_READY_FLAG_OFF();
			}
		} else if (ZX_IS_IO_READ()) {
			address = (ZX_ADDR() & 0xff);
			if (address == ZX_CONTROL_PORT) {
				// return 2 if transfer finished
				ZX_ASSERT_WAIT();
				if (data_index >= sizeof(rtype_scr)) {
					data = 2;
				} else {
					data = 0;
				}
				ZX_DATA_OUT(data);
				ZX_DATA_READY_FLAG_ON();
				ZX_DEASSERT_WAIT();
				while (ZX_IS_IO_READ()) { }
				ZX_DATA_HI_Z();
				ZX_DATA_READY_FLAG_OFF();
			} else if (address == ZX_DATA_PORT) {
				ZX_ASSERT_WAIT();
				if (data_index < sizeof(rtype_scr)) {
					data = rtype_scr[data_index];
				} else {
					data = 0;
				}
				ZX_DATA_OUT(data);
				ZX_DATA_READY_FLAG_ON();
				ZX_DEASSERT_WAIT();
				while (ZX_IS_IO_READ()) { }
				ZX_DATA_HI_Z();
				ZX_DATA_READY_FLAG_OFF();
				data_index++; // increase data pointer on every IN from ZX_DATA_PORT
			}
		}

		//if (ZX_IS_RESET()) goto RESET;
		//__WFI();
		//for (w = 0; w < 1000; w++);
		//LED1_ON();
		//LED2_OFF();
		//for (w = 0; w < 100; w++);
		//LED1_OFF();
		//LED2_ON();
		//__WFI();
	} // done (infinite loop)
}
