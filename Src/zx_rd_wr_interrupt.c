/*
 * rd_wr_interrupt.c
 *
 *  Created on: 14. 8. 2016
 *      Author: Kriz
 */

#include <raiders_rom.h>
#include <zxtest_rom.h>
#include <esxdos080_rom.h>
#include <didaktik_gama_89_mod_rom.h>
#include <fatware014_rom.h>
#include "zx_rd_wr_interrupt.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"

int volatile divide_command = 0;
int volatile divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
int volatile divide_lba_0 = 0;
int volatile divide_lba_1 = 0;
int volatile divide_lba_2 = 0;
int volatile divide_lba_3 = 0;
volatile int divide_sector_count = 1;

#define IDE_STATUS_ERR 0x01
#define IDE_STATUS_DRQ 0x08
#define IDE_STATUS_DRDY 0x40
#define IDE_STATUS_BSY 0x80

#define DEASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = ZX_WAIT_Pin;}
#define ASSERT_ZX_WAIT() {ZX_WAIT_GPIO_Port->BSRR = (uint32_t)ZX_WAIT_Pin << 16;}

#define DEASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = ZX_RESET_Pin;}
#define ASSERT_ZX_RESET() {ZX_RESET_GPIO_Port->BSRR = (uint32_t)ZX_RESET_Pin << 16;}

#define DEASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = ZX_ROMCS_Pin;}
#define ASSERT_ZX_ROMCS() {ZX_ROMCS_GPIO_Port->BSRR = (uint32_t)ZX_ROMCS_Pin << 16;}

#define ZX_DATA_OUT(data) {ZX_DATA_GPIO_PORT->BSRR = (uint32_t)0xff << 16; ZX_DATA_GPIO_PORT->MODER = (uint32_t)0b0101010101010101;  ZX_DATA_GPIO_PORT->BSRR = data;  } // ala UNICARD, ale MODER natvrdo misto |=, MODER doprostred
#define ZX_DATA_HI_Z() {ZX_DATA_GPIO_PORT->MODER = 0;}

#define ZX_IS_MEM_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_MEMREQ_Pin)) == 0)
#define ZX_IS_IO_READ(control_lines) (((uint16_t)control_lines & (ZX_RD_Pin | ZX_IOREQ_Pin)) == 0)

#define CLEAR_ZX_CONTROL_EXTI() {__HAL_GPIO_EXTI_CLEAR_IT(ZX_RD_Pin);__HAL_GPIO_EXTI_CLEAR_IT(ZX_WR_Pin);}
#define HANG_LOOP() {HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); while (1) {}}

#define DIVIDE_EEPROM_WRITABLE 1

zx_control_handler_t zx_io_rd_service_table[256];
zx_control_handler_t zx_io_wr_service_table[256];

//0. 16kb = Divide RAM pages 0,1
//1. 16kb = Divide RAM pages 2,3
//2. 16kB = ZX ROM copy
//3. 16kB = Divide ROM (8kb)
//uint8_t device_ram[16384*6 - 2400];
//uint8_t device_ram[16384*4 - 8192];
uint8_t device_ram[16384*4];
//uint8_t device_ram[512];
uint8_t divide_mapped = 0;
uint8_t divide_page = 0;
uint8_t divide_conmem = 0;
uint8_t divide_mapram = 0;
int divide_eeprom_addr = 3*0x4000; // Divide ROM at startup
volatile int last_m1_addr = 0;
volatile int last_io_op = 0;
volatile int dummy = 0;
int32_t low_8k_rom_offset = 0;
int32_t high_8k_rom_offset = 0;
uint8_t ide_operation = 0;
uint8_t ide_data_ready = 0;
volatile int ide_drive_buffer_pointer = 0;
uint8_t ide_drive_buffer_endian_pointer = 1;
int divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
int divide_highbank_writable = 1;

volatile uint8_t ide_drive_buffer[512];
volatile int ide_bytes_received = 0;
volatile int ide_bytes_received_total = 0;
volatile int last_port_addr = 0;
volatile int last_port_data = 0;
volatile int last_port_op = 0;

//int divide_automap = 0;

void __attribute__((section(".fast_code"))) __STATIC_INLINE divide_set_automap_on() {
	low_8k_rom_offset = divide_eeprom_addr; // Divide ROM or Divide Bank 3 RAM
	high_8k_rom_offset = divide_page*0x2000; // Divide RAM page (8kB) 0..3
	divide_mapped = 1;
}

void __attribute__((section(".fast_code"))) __STATIC_INLINE divide_set_automap_off() {
	if (!divide_conmem) { // do not unmap when conmem == true
		low_8k_rom_offset = 2*0x4000; // ZX ROM copy (low 8kB)
		high_8k_rom_offset = 2*0x4000 + 0x2000; // ZX ROM copy (high 8kB)
		divide_mapped = 0;
	}
}

void  __attribute__((section(".fast_code"))) zx_noop() {
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_wr_hang_loop() {
	last_io_op = 2;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) io_rd_hang_loop() {
	last_io_op = 1;
	volatile int addr = ZX_ADDR_GPIO_PORT->IDR & 0xff;
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	HANG_LOOP();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_data_register_rd() {
//	if (ide_drive_buffer_word_pointer < 256) {
//		// return in order lowbyte, highbyte
//		if (ide_drive_buffer_endian_pointer) {
//			// now return low byte
//			ZX_DATA_OUT(ide_drive_itentify_buffer[ide_drive_buffer_word_pointer*2+1]);
//			ide_drive_buffer_endian_pointer = 0;
//		} else {
//			// now return high byte and move pointer to next word
//			ZX_DATA_OUT(ide_drive_itentify_buffer[ide_drive_buffer_word_pointer*2]);
//			ide_drive_buffer_word_pointer++;
//			ide_drive_buffer_endian_pointer = 1;
//		}
//	} else {
//		ZX_DATA_OUT(0);
//	}
	if (ide_drive_buffer_pointer < 512) {
		ZX_DATA_OUT(ide_drive_buffer[ide_drive_buffer_pointer++]);
	} else {
		ZX_DATA_OUT(0);
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
		ide_data_ready = 0;
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_control_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	last_port_data = data;
	divide_page = data & 0b11;
	divide_conmem = data & 0x10000000; // CONMEM bit
	if (divide_conmem) {
		divide_eeprom_addr = 3*0x4000; // Divide ROM
		divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
		divide_highbank_writable = 1; // Divide RAM writable
	} else {
		if (divide_mapram || (data & 0b01000000)) { // MAPRAM
			divide_mapram = 1;
			divide_eeprom_addr = 3*0x2000; // Divide RAM bank 3 (8kB)
			divide_lowbank_writable = 0;
			divide_highbank_writable = (divide_page != 3); // Divide RAM writable only when not bank 3
		} else {
			divide_eeprom_addr = 3*0x4000; // Divide ROM
			divide_lowbank_writable = DIVIDE_EEPROM_WRITABLE;
			divide_highbank_writable = 1; // Divide RAM writable
		}
	}
	if (divide_mapped || divide_conmem) {
		divide_set_automap_on(); // recalculate offsets for a new RAM page
	} else {
		divide_set_automap_off(); // unmap divide memory (in case divide_conmem has been turned off)
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_sector_count_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_sector_count = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_drive_head_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_3 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_drive_head_register_rd() {
	ZX_DATA_OUT(divide_lba_3);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_status_register_rd() {
	if (divide_command_status == DIVIDE_COMMAND_DATA_READY) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_DRQ); // data ready (and device ready?)
	} else if (divide_command_status == DIVIDE_COMMAND_ISSUED || divide_command_status == DIVIDE_COMMAND_IN_PROGRESS) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_BSY); // Controller is busy executing a command.
	} else {
		ZX_DATA_OUT(IDE_STATUS_DRDY); // device ready
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_error_register_rd() {
	ZX_DATA_OUT(0); // no error
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_command_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_command_status = DIVIDE_COMMAND_ISSUED;
	divide_command = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba0_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_0 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba1_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_1 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba2_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_2 = data;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba0_rd() {
	ZX_DATA_OUT(divide_lba_0);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba1_rd() {
	ZX_DATA_OUT(divide_lba_1);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) divide_lba2_rd() {
	ZX_DATA_OUT(divide_lba_2);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}


void copy_roms_to_ram() {
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	uint32_t i;
	// BANK 0
	// BANK 1
	// BANK 2
	for (i = 0; i < sizeof(didaktik_gama_89_mod_rom) && i < 16384; i++) device_ram[i+2*16384] = didaktik_gama_89_mod_rom[i];
	// BANK 3
	for (i = 0; i < sizeof(esxdos080_rom) && i < 16384; i++) device_ram[i+3*16384] = esxdos080_rom[i];
	//for (i = 0; i < sizeof(fatware014_rom) && i < 16384; i++) device_ram[i+3*16384] = fatware014_rom[i];
	//for (i = 0; i < sizeof(raiders_rom) && i < 16384; i++) device_ram[i] = raiders_rom[i];
	//for (i = 0; i < sizeof(zxtest_rom) && i < 16384; i++) device_ram[i+16384] = zxtest_rom[i];

	// by default, do not handle IO operations on any port
	for (i = 0; i < 256; i++) zx_io_rd_service_table[i] = zx_noop;
	for (i = 0; i < 256; i++) zx_io_wr_service_table[i] = zx_noop;
	// register non-ULA ports to noop
	for (i = 1; i < 256; i+=2) {
		zx_io_wr_service_table[i] = zx_noop;
		zx_io_rd_service_table[i] = zx_noop;
	}
	// register particular port handlers
	zx_io_wr_service_table[0x0e3] = divide_control_register_wr;
	zx_io_rd_service_table[0x0a3] = divide_data_register_rd;
	zx_io_wr_service_table[0x0a3] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0a7] = divide_error_register_rd;
	zx_io_wr_service_table[0x0a7] = io_wr_hang_loop;
	zx_io_rd_service_table[0x0ab] = io_rd_hang_loop;
	zx_io_wr_service_table[0x0ab] = divide_sector_count_register_wr;
	zx_io_rd_service_table[0x0af] = divide_lba0_rd;
	zx_io_wr_service_table[0x0af] = divide_lba0_wr;
	zx_io_rd_service_table[0x0b3] = divide_lba1_rd;
	zx_io_wr_service_table[0x0b3] = divide_lba1_wr;
	zx_io_rd_service_table[0x0b7] = divide_lba2_rd;
	zx_io_wr_service_table[0x0b7] = divide_lba2_wr;
	zx_io_rd_service_table[0x0bb] = divide_drive_head_register_rd;
	zx_io_wr_service_table[0x0bb] = divide_drive_head_register_wr;
	zx_io_rd_service_table[0x0bf] = divide_status_register_rd;
	zx_io_wr_service_table[0x0bf] = divide_command_register_wr;
	// 128k mapovani?
	zx_io_wr_service_table[0x0fd] = zx_noop; // ignore writes
	zx_io_rd_service_table[0x0fd] = zx_noop; // ignore reads
	// nejaka mys?
	zx_io_wr_service_table[0x07f] = zx_noop; // ignore writes
	// kempston joystick
	zx_io_rd_service_table[0x01f] = zx_noop;
	zx_io_wr_service_table[0x01f] = zx_noop;
}

void  __attribute__((section(".fast_code"))) zx_mem_rd_nonm1() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		register uint8_t data = device_ram[low_8k_rom_offset + address];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else if (address < 0x4000) { // high 8kB of ROM area
		register uint8_t data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else {
		//DEASSERT_ZX_WAIT();
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_mem_rd_m1() {
	//ASSERT_ZX_WAIT();
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	if (address < 0x2000) { // low 8kB of ROM area
		register uint8_t data = device_ram[low_8k_rom_offset + address];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
		if ((address & 0xfff8) == 0x1ff8) {
				      divide_set_automap_off();
		} else if ((address == 0x0000) || (address == 0x0008) || (address == 0x0038)
				      || (address == 0x0066) || (address == 0x04c6) || (address == 0x0562)) {
				      divide_set_automap_on();
		}
	} else if (address < 0x4000) { // high 8kB of ROM area
		if ((address & 0xff00) == 0x3d00) {
		      divide_set_automap_on();
		}
		register uint8_t data = device_ram[high_8k_rom_offset + address - 0x2000];
		ZX_DATA_OUT(data);
		//DEASSERT_ZX_WAIT();
		while (ZX_IS_MEM_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
		ZX_DATA_HI_Z();
	} else {
		//DEASSERT_ZX_WAIT();
	}
	last_m1_addr = address;
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_mem_wr() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
	register uint8_t data = ZX_DATA_GPIO_PORT->IDR;
	if (divide_mapped) { // ignore writes to ROM when Divide not mapped
		if ((address < 0x2000) && divide_lowbank_writable) { // low 8kB of ROM area
			device_ram[low_8k_rom_offset + address] = data;
		} else if ((address < 0x4000) && divide_highbank_writable) { // high 8kB of ROM area
			device_ram[high_8k_rom_offset + address - 0x2000] = data;
		}
	}
	CLEAR_ZX_CONTROL_EXTI();
}

void  __attribute__((section(".fast_code"))) zx_io_rd() {
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
//	if (((address & 1) != 0) && ((address & 0xff) != 0x7f)  && ((address & 0xff) != 0xe3)) {
//		last_port_addr = address;
//		last_port_op = 1;
//	}
	// decode ports using 8 bits only
	zx_io_rd_service_table[address & 0xff]();
}

void  __attribute__((section(".fast_code"))) zx_io_wr() {
	// TODO register
	register uint16_t address = ZX_ADDR_GPIO_PORT->IDR;
//	if (((address & 1) != 0) && ((address & 0xff) != 0x7f)  && ((address & 0xff) != 0xe3)) {
//		last_port_addr = address;
//		last_port_op = 2;
//	}
//	if (address & 1) {
//		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//		while (1) {
//
//		}
//	}
	// decode ports using 8 bits only
	zx_io_wr_service_table[address & 0xff]();
}


//static const
zx_control_handler_t exti_service_table[64]={
		// operation  // binary pattern for RD, WR, MEMREQ, dontcare, M1, IOREQ
		zx_noop	,  //	000000
		zx_noop	,  //	000001
		zx_noop	,  //	000010
		zx_noop	,  //	000011
		zx_noop	,  //	000100
		zx_noop	,  //	000101
		zx_noop	,  //	000110
		zx_noop	,  //	000111
		zx_noop	,  //	001000
		zx_noop	,  //	001001
		zx_noop	,  //	001010
		zx_noop	,  //	001011
		zx_noop	,  //	001100
		zx_noop	,  //	001101
		zx_noop	,  //	001110
		zx_noop	,  //	001111
		zx_noop	,  //	010000
		zx_mem_rd_m1	,  //	010001
		zx_noop	,  //	010010
		zx_mem_rd_nonm1	,  //	010011
		zx_noop	,  //	010100
		zx_mem_rd_m1	,  //	010101
		zx_noop	,  //	010110
		zx_mem_rd_nonm1	,  //	010111
		zx_noop	,  //	011000
		zx_noop	,  //	011001
		zx_io_rd	,  //	011010
		zx_noop	,  //	011011
		zx_noop	,  //	011100
		zx_noop	,  //	011101
		zx_io_rd	,  //	011110
		zx_noop	,  //	011111
		zx_noop	,  //	100000
		zx_noop	,  //	100001
		zx_noop	,  //	100010
		zx_mem_wr	,  //	100011
		zx_noop	,  //	100100
		zx_noop	,  //	100101
		zx_noop	,  //	100110
		zx_mem_wr	,  //	100111
		zx_noop	,  //	101000
		zx_noop	,  //	101001
		zx_io_wr	,  //	101010
		zx_noop	,  //	101011
		zx_noop	,  //	101100
		zx_noop	,  //	101101
		zx_io_wr	,  //	101110
		zx_noop	,  //	101111
		zx_noop	,  //	110000
		zx_noop	,  //	110001
		zx_noop	,  //	110010
		zx_noop	,  //	110011
		zx_noop	,  //	110100
		zx_noop	,  //	110101
		zx_noop	,  //	110110
		zx_noop	,  //	110111
		zx_noop	,  //	111000
		zx_noop	,  //	111001
		zx_noop	,  //	111010
		zx_noop	,  //	111011
		zx_noop	,  //	111100
		zx_noop	,  //	111101
		zx_noop	,  //	111110
		zx_noop	,  //	111111
};



/**
 * Set GPIO as output open-drain with pull-up
 */
void inline init_output_od_pullup(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Set GPIO as output open-drain (without pull-up/down)
 */
void inline init_output_od(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Set GPIO as output push-pull (without pull-up/down)
 */
void inline init_output_pp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * Initialize particular pins to output mode
 */
void zx_init_pins(void) {

	copy_roms_to_ram();


	DEASSERT_ZX_RESET();
	init_output_od(ZX_RESET_GPIO_Port, ZX_RESET_Pin);

	DEASSERT_ZX_WAIT();
	//init_output_od_pullup(ZX_WAIT_GPIO_Port, ZX_WAIT_Pin);
	init_output_pp(ZX_WAIT_GPIO_Port, ZX_WAIT_Pin);

	// disable internal ZX ROM permanently
	DEASSERT_ZX_ROMCS();
	init_output_pp(ZX_ROMCS_GPIO_Port, ZX_ROMCS_Pin);

	// reset ZX Spectrum

	ASSERT_ZX_RESET();


	HAL_Delay(4000);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_Delay(4000);


	DEASSERT_ZX_RESET();
}
