/*
 * emu_divide_ports.c
 *
 *  Created on: 28. 12. 2016
 *      Author: Kriz
 */

#include "emu_divide_ports.h"

// ================ ARM <--> ZX interface ================
#include "zx_signals.h"
#include "utils.h"
#include "zx_rd_wr_interrupt.h"

// ======= ARM <--> emulated IDE device interface ========
#include "usb_host.h"
#include "usbh_msc.h"

#define PIO_PHASE_READY 1
#define PIO_PHASE_IN 	2
#define PIO_PHASE_OUT 	3

// Structs for various ATA versions:
//   https://github.com/ARM-software/edk2/blob/master/MdePkg/Include/IndustryStandard/Atapi.h
// Explanation of some fields and sample data from real drive:
//   https://github.com/krieger-od/whdd/issues/57#issuecomment-41500941

///
/// ATA5_IDENTIFY_DATA is defined in ATA-5.
/// (This structure is provided mainly for backward-compatibility support.
/// Old drivers may reference fields that are marked "obsolete" in
/// ATA_IDENTIFY_DATA, which currently conforms to ATA-8.)
///
typedef struct {
  uint16_t  config;             ///< General Configuration.
  uint16_t  cylinders;          ///< Number of Cylinders.
  uint16_t  reserved_2;
  uint16_t  heads;              ///< Number of logical heads.
  uint16_t  vendor_data1;
  uint16_t  vendor_data2;
  uint16_t  sectors_per_track;
  uint16_t  vendor_specific_7_9[3];
  char   	SerialNo[20];       ///< ASCII (word 10)
  uint16_t  vendor_specific_20_21[2];
  uint16_t  ecc_bytes_available;
  char   	FirmwareVer[8];     ///< ASCII
  char   	ModelName[40];      ///< ASCII
  uint16_t  multi_sector_cmd_max_sct_cnt;
  uint16_t  reserved_48;
  uint16_t  capabilities;
  uint16_t  reserved_50;
  uint16_t  pio_cycle_timing;
  uint16_t  reserved_52;
  uint16_t  field_validity;
  uint16_t  current_cylinders;
  uint16_t  current_heads;
  uint16_t  current_sectors;
  uint16_t  CurrentCapacityLsb;
  uint16_t  CurrentCapacityMsb;
  uint16_t  reserved_59;
  uint16_t  user_addressable_sectors_lo;
  uint16_t  user_addressable_sectors_hi;
  uint16_t  reserved_62;
  uint16_t  multi_word_dma_mode;
  uint16_t  advanced_pio_modes;
  uint16_t  min_multi_word_dma_cycle_time;
  uint16_t  rec_multi_word_dma_cycle_time;
  uint16_t  min_pio_cycle_time_without_flow_control;
  uint16_t  min_pio_cycle_time_with_flow_control;
  uint16_t  reserved_69_79[11];
  uint16_t  major_version_no;
  uint16_t  minor_version_no;
  uint16_t  command_set_supported_82;    ///< word 82
  uint16_t  command_set_supported_83;    ///< word 83
  uint16_t  command_set_feature_extn;    ///< word 84
  uint16_t  command_set_feature_enb_85;  ///< word 85
  uint16_t  command_set_feature_enb_86;  ///< word 86
  uint16_t  command_set_feature_default; ///< word 87
  uint16_t  ultra_dma_mode;              ///< word 88
  uint16_t  reserved_89_127[39];
  uint16_t  security_status;
  uint16_t  vendor_data_129_159[31];
  uint16_t  reserved_160_255[96];
} __attribute__((packed, aligned(4))) ATA_IDENTIFY_DATA;

int volatile divide_command = 0;
int volatile divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
int volatile divide_lba_0 = 0;
int volatile divide_lba_1 = 0;
int volatile divide_lba_2 = 0;
int volatile divide_lba_3 = 0;
int volatile divide_sector_count = 1;

volatile int ide_drive_buffer_pointer;
volatile uint8_t ide_drive_buffer[512];
volatile int ide_drive_pio_phase = PIO_PHASE_READY;
volatile uint32_t lba_address;

ATA_IDENTIFY_DATA ideIdentifyBuffer;

void dump_ide_partition_entry(uint8_t ide_drive_buffer_mbr[], int entry_number) {
	 int offset = 0x1be + entry_number*16;
	 uint32_t lba_address = ide_drive_buffer_mbr[offset + 0x8 + 0] + ide_drive_buffer_mbr[offset + 0x8 + 1]*256 + ide_drive_buffer_mbr[offset + 0x8 + 2]*256*256 + (ide_drive_buffer_mbr[offset + 0x8 + 3] & 0b1111)*256*256*256;
	 UART2_printf("  Partition entry %d: status 0x%02x type 0x%02x chs_address: 0x%02x.%02x.%02x, lba_address 0x%08x\r\n", entry_number+1, ide_drive_buffer_mbr[offset + 0x0], ide_drive_buffer_mbr[offset + 0x4], ide_drive_buffer_mbr[offset + 0x1], ide_drive_buffer_mbr[offset + 0x2], ide_drive_buffer_mbr[offset + 0x3], lba_address);
}

void dump_ide_block(uint32_t lba_address, uint8_t ide_drive_buffer[]) {
	int i = 0;
	while (i < 512) {
		int j = 0;
		UART2_printf("  %04x:", i);
		while ((j < 32) && (i < 512)) {
			UART2_printf(" %02x", ide_drive_buffer[i]);
			j++;
			i++;
		}
		UART2_printf("\r\n");
	}
	if (lba_address == 0) {
		UART2_printf("  Master boot sector signature: 0x%02x 0x%02x (0x55 0xaa expected)\r\n", ide_drive_buffer[510], ide_drive_buffer[511]);
		dump_ide_partition_entry(ide_drive_buffer, 0);
		dump_ide_partition_entry(ide_drive_buffer, 1);
		dump_ide_partition_entry(ide_drive_buffer, 2);
		dump_ide_partition_entry(ide_drive_buffer, 3);
	} else if ((ide_drive_buffer[510] == 0x55) && (ide_drive_buffer[511] == 0xaa)) {
		UART2_printf("  First sector of a partition, i.e. boot record(?):\r\n");
		UART2_printf("    oem_name=");
		for (int i = 0; i < 8; i++) {
			UART2_printf("%c", ide_drive_buffer[0x3+i]);
		}
		UART2_printf("\r\n");
		int directory_item_size = 32;
		uint32_t bytes_per_sector = ide_drive_buffer[0xB]+ide_drive_buffer[0xC]*256;
		UART2_printf("    bytes_per_sector=%d (usually 512 B)\r\n", bytes_per_sector);
		UART2_printf("    sectors_per_cluster=%d\r\n", ide_drive_buffer[0xd]);
		UART2_printf("    reserved_sectors=%d (min. 1)\r\n", ide_drive_buffer[0xe]);
		UART2_printf("    number_of_fats=%d (usually 2)\r\n", ide_drive_buffer[0x10]);
		uint32_t number_of_root_entries = ide_drive_buffer[0x11] + ide_drive_buffer[0x12]*256;
		UART2_printf("    number_of_root_entries=%d\r\n", number_of_root_entries);
		UART2_printf("    sectors_per_fat=%d\r\n", ide_drive_buffer[0x16]);
		uint32_t root_dir_sect = lba_address + ide_drive_buffer[0xe] + ide_drive_buffer[0x10]*ide_drive_buffer[0x16];
		UART2_printf("    root_directory_sector=%d (0x%08x)\r\n", root_dir_sect, root_dir_sect);
		uint32_t root_directory_sectors = number_of_root_entries*32/512;
		UART2_printf("    root_directory_no_of_sectors=%d\r\n", root_directory_sectors);
	} else if (lba_address < 2600) { // kind-of-heuristic address guess
		UART2_printf("  Directory items?:\r\n");
		for (int i = 0; i < 512/32; i++) {
			int dir_entry_offset = i*32;
			if (ide_drive_buffer[dir_entry_offset] != 0) {
				// directory entry is valid
				UART2_printf("    ");
				for (int j = 0; j < 8; j++) {
					UART2_printf("%c", ide_drive_buffer[dir_entry_offset+j]);
				}
				UART2_printf(".");
				UART2_printf("%c%c%c", ide_drive_buffer[dir_entry_offset+8], ide_drive_buffer[dir_entry_offset+9], ide_drive_buffer[dir_entry_offset+10]);
				UART2_printf("   ");
				uint32_t start_cluster = ide_drive_buffer[dir_entry_offset+0x1A] + ide_drive_buffer[dir_entry_offset+0x1B]*256;
				uint32_t file_size = ide_drive_buffer[dir_entry_offset+0x1C] + ide_drive_buffer[dir_entry_offset+0x1D]*256 + ide_drive_buffer[dir_entry_offset+0x1E]*256*256 + ide_drive_buffer[dir_entry_offset+0x1F]*256*256*256;
				if ((ide_drive_buffer[dir_entry_offset+0xb] & 0x10) > 0) {
					UART2_printf("[DIR]      ");
					UART2_printf(" start=%d (0x%08x)", start_cluster, start_cluster);
				} else {
					UART2_printf("%010dB", file_size);
					UART2_printf(" start=%d (0x%08x)", start_cluster, start_cluster);
				}
				UART2_printf("\r\n");
			}
		}
	}
}

void seek() {
	lba_address =
			divide_lba_0
			+ divide_lba_1 * 256
			+ divide_lba_2 * 256 * 256
			+ (divide_lba_3 & 0b1111) * 256 * 256 * 256;
}

void FAST_CODE divide_data_register_rd() {
	if (ide_drive_pio_phase == PIO_PHASE_IN) {
		ZX_DATA_OUT(ide_drive_buffer[ide_drive_buffer_pointer++]);
	} else {
		ZX_DATA_OUT(0xff);
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	if (ide_drive_buffer_pointer == 512) {
		divide_command_status = DIVIDE_COMMAND_DEVICE_READY;
		ide_drive_pio_phase = PIO_PHASE_READY;
	}
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void FAST_CODE divide_data_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	if (ide_drive_pio_phase == PIO_PHASE_OUT) {
		ide_drive_buffer[ide_drive_buffer_pointer++] = data;
	}
	if (ide_drive_buffer_pointer == 512) {
		ide_drive_pio_phase = PIO_PHASE_READY;
	}
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}


void FAST_CODE divide_sector_count_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_sector_count = data;
	if (data > 1) {
		HANG_LOOP();
	}
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_drive_head_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_3 = data;
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_drive_head_register_rd() {
	ZX_DATA_OUT(divide_lba_3);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_error_register_rd() {
	ZX_DATA_OUT(0); // no error TODO obtain from USB MSC HAL
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba0_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_0 = data;
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba1_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_1 = data;
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba2_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_lba_2 = data;
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba0_rd() {
	ZX_DATA_OUT(divide_lba_0);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba1_rd() {
	ZX_DATA_OUT(divide_lba_1);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_lba2_rd() {
	ZX_DATA_OUT(divide_lba_2);
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

// TODO work-in-progress
void FAST_CODE divide_command_register_wr() {
	volatile int data = ZX_DATA_GPIO_PORT->IDR & 0xff;
	divide_command = data;
	seek(); // TODO maybe only for some relevant operations
	ide_drive_buffer_pointer = 0;
	divide_command_status = DIVIDE_COMMAND_ISSUED;
	while (ZX_IS_IO_WRITE(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	CLEAR_ZX_CONTROL_EXTI();
}

void FAST_CODE divide_status_register_rd() {
	if (divide_command_status == DIVIDE_COMMAND_DATA_READY) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_DRQ); // data ready (and device ready?)
	} else if (divide_command_status == DIVIDE_COMMAND_WRITE_FILLING_BUFFER) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_DRQ); // data request
	} else if ((divide_command_status == DIVIDE_COMMAND_ISSUED) || (divide_command_status == DIVIDE_COMMAND_IN_PROGRESS) || (divide_command_status == DIVIDE_COMMAND_WRITE_BUFFER_FILLED)) {
		ZX_DATA_OUT(IDE_STATUS_DRDY | IDE_STATUS_BSY); // Controller is busy executing a command.
	} else {
		ZX_DATA_OUT(IDE_STATUS_DRDY); // device ready
	}
	while (ZX_IS_IO_READ(ZX_CONTROL_IN_GPIO_PORT->IDR)) { }
	ZX_DATA_HI_Z();
	CLEAR_ZX_CONTROL_EXTI();
}

/*
 Translate LBA size to CHS
 https://en.wikipedia.org/wiki/Logical_block_addressing#LBA-assisted_translation
 Disk size					Sectors/track	Heads	Cylinders
 1 < X <= 504 MiB				63				16		X / (63 * 16 * 512)
 504 MiB < X <= 1008 MiB		63				32		X / (63 * 32 * 512)
 1008 MiB < X <= 2016 MiB		63				64		X / (63 * 64 * 512)
 2016 MiB < X <= 4032 MiB		63				128		X / (63 * 128 * 512)
 4032 MiB < X <= 8032.5 MiB		63				255		X / (63 * 255 * 512)
 for larger capacities, indicate S=63 H=16 C=16383 S*H*C=16514064
 According to the ATA specifications, "If the content of words (61:60) is
 greater than or equal to 16,514,064, then the content of word 1
 [the number of logical cylinders] shall be equal to 16,383."
 Therefore, for LBA 16450559, an ATA drive may actually respond with the
 CHS tuple (16319, 15, 63), and the number of cylinders in this
 scheme must be much larger than 1024 allowed by INT 13h.[a]
*/
void fill_chs(ATA_IDENTIFY_DATA *ataIdentifyBuffer, uint32_t lba_blocks) {
	ataIdentifyBuffer->sectors_per_track = 63;
	long lba_bytes = lba_blocks * 512; // TODO BSOD on sector size different from 512
	if 		(lba_bytes <= 504*1024*1024) 	ataIdentifyBuffer->heads = 16;
	else if (lba_bytes <= 1008*1024*1024) 	ataIdentifyBuffer->heads = 32;
	else if (lba_bytes <= 2016*1024*1024) 	ataIdentifyBuffer->heads = 64;
	else if (lba_bytes <= 4032*1024*1024) 	ataIdentifyBuffer->heads = 128;
	else if (lba_bytes <= 8032.5*1024*1024) ataIdentifyBuffer->heads = 255;
	if (lba_bytes <= 8032.5*1024*1024) {
		ataIdentifyBuffer->cylinders = lba_blocks/63/ataIdentifyBuffer->heads;
	} else {
		ataIdentifyBuffer->heads = 16;
		ataIdentifyBuffer->cylinders = 16383;
	}
}

void fill_ata_identify_data(ATA_IDENTIFY_DATA *ataIdentifyBuffer) {
	MSC_LUNTypeDef lunInfo;
	MSC_GetLUN0Info(&lunInfo);

	char dst[41] = "";
	char tmp[41] = "";

	// dst = dst + vendor
	strlcpy(tmp, lunInfo.inquiry.vendor_id, sizeof(tmp));
	strtrim(tmp);
	strlcat(dst, tmp, sizeof(dst));

	// dst = dst + ""
	strlcat(dst, " ", sizeof(dst));

	// dst = dst + product
	strlcpy(tmp, lunInfo.inquiry.product_id, sizeof(tmp));
	strtrim(tmp);
	strlcat(dst, tmp, sizeof(dst));

	// dst = dst + ""
	strlcat(dst, " ", sizeof(dst));

	// dst = dst + revision
	strlcpy(tmp, lunInfo.inquiry.revision_id, sizeof(tmp));
	strtrim(tmp);
	strlcat(dst, tmp, sizeof(dst));

	ataIdentifyBuffer->config = 0x427a;
	fill_chs(ataIdentifyBuffer, lunInfo.capacity.block_nbr);

	strlcpy(ataIdentifyBuffer->SerialNo, "                    ", sizeof(ataIdentifyBuffer->SerialNo)); // left padded serial no (not available via USB host)

	copy_right_padded(ataIdentifyBuffer->FirmwareVer, lunInfo.inquiry.revision_id, sizeof(ataIdentifyBuffer->FirmwareVer));
	scramble_ata_string(ataIdentifyBuffer->FirmwareVer, sizeof(ataIdentifyBuffer->FirmwareVer));

	copy_right_padded(ataIdentifyBuffer->ModelName, dst, sizeof(ataIdentifyBuffer->ModelName)); // right padded
	scramble_ata_string(ataIdentifyBuffer->ModelName, sizeof(ataIdentifyBuffer->ModelName));

	ataIdentifyBuffer->multi_sector_cmd_max_sct_cnt = 0x8010; // max. 16 sectors per READ/WRITE MULTIPLE
	ataIdentifyBuffer->capabilities = 0x2f00; // or 0x0200
	ataIdentifyBuffer->reserved_50 = 0x4000;
	ataIdentifyBuffer->pio_cycle_timing = 0x0200;
	ataIdentifyBuffer->field_validity = 1;
	ataIdentifyBuffer->current_cylinders = ataIdentifyBuffer->cylinders;
	ataIdentifyBuffer->current_heads = ataIdentifyBuffer->heads;
	ataIdentifyBuffer->current_sectors = ataIdentifyBuffer->sectors_per_track;
	long total_sectors_count_chs = ataIdentifyBuffer->cylinders * ataIdentifyBuffer->heads * ataIdentifyBuffer->sectors_per_track;
	ataIdentifyBuffer->CurrentCapacityLsb = total_sectors_count_chs & 0x0000ffff;
	ataIdentifyBuffer->CurrentCapacityMsb = (total_sectors_count_chs & 0xffff0000) >> 16;
	// User addressable sectors for 28-bit commands (DWord): (may be higher than CurrentCapacity*)
	long total_sectors_count_28bit = lunInfo.capacity.block_nbr & 0xfffffff;
	ataIdentifyBuffer->user_addressable_sectors_lo = total_sectors_count_28bit & 0x0000ffff;
	ataIdentifyBuffer->user_addressable_sectors_hi = (total_sectors_count_28bit & 0xffff0000) >> 16;

}

void emu_divide_ports_handle_main_loop() {

	if (divide_command_status == DIVIDE_COMMAND_ISSUED) {
		if (GetUsbHostAppliState() == APPLICATION_READY) {
			divide_command_status = DIVIDE_COMMAND_IN_PROGRESS;
			if (divide_sector_count > 1) {
				HANG_LOOP();
			}
			if (divide_command == 0x20) {
				UART2_printf("USB reading LBA=%d\r\n", lba_address);
				// ATA READ BLOCK
				USBH_StatusTypeDef operation_status = MSC_Read(lba_address,
						ide_drive_buffer, 1);
				if (operation_status != USBH_OK) {
					UART2_printf("  USB ERROR #%d while reading LBA=%d\r\n",
							operation_status, lba_address);
					//HANG_LOOP();
				} else {
					UART2_printf("  USB DONE\r\n", operation_status,
							lba_address);
					//dump_ide_block(lba_address, ide_drive_buffer);
				}
				ide_drive_buffer_pointer = 0;
				ide_drive_pio_phase = PIO_PHASE_IN;
				divide_command_status = DIVIDE_COMMAND_DATA_READY;
			} else if (divide_command == 0x30) {
				UART2_printf("USB writing LBA=%d waiting for buffer fill\r\n",
						lba_address);
				ide_drive_buffer_pointer = 0;
				ide_drive_pio_phase = PIO_PHASE_OUT;
				divide_command_status = DIVIDE_COMMAND_WRITE_FILLING_BUFFER;
			} else if (divide_command == 0xEC) {
				// ATA IDENTIFY CMD
				UART2_printf("USB reading ATA IDENTIFY\r\n");
				fill_ata_identify_data(&ideIdentifyBuffer);
				memcpy(ide_drive_buffer, &ideIdentifyBuffer, 512);
				ide_drive_buffer_pointer = 0;
				ide_drive_pio_phase = PIO_PHASE_IN;
				divide_command_status = DIVIDE_COMMAND_DATA_READY;
			} else {
				UART2_printf("UKNOWN IDE COMMAND %d\r\n", divide_command);
			}
		} else {
			UART2_printf("DEVICE NOT READY, COMMAND WAITING: %d\r\n",
					divide_command);
			HAL_Delay(50);
		}
	} else if ((divide_command == 0x30) && (divide_command_status == DIVIDE_COMMAND_WRITE_FILLING_BUFFER)) {
		if (ide_drive_buffer_pointer == 512) {
			divide_command_status = DIVIDE_COMMAND_WRITE_BUFFER_FILLED;
		}
	} else if ((divide_command == 0x30) && (divide_command_status == DIVIDE_COMMAND_WRITE_BUFFER_FILLED)) {
		// ATA WRITE BLOCK
		UART2_printf("USB writing LBA=%d to disk\r\n", lba_address);
		//dump_ide_block(lba_address, ide_drive_buffer);
		USBH_StatusTypeDef operation_status = MSC_Write(lba_address,
				ide_drive_buffer, 1);
		if (operation_status != USBH_OK) {
			UART2_printf("  USB ERROR #%d while writing LBA=%d\r\n",
					operation_status, lba_address);
			//HANG_LOOP();
		} else {
			UART2_printf("  USB DONE\r\n", operation_status, lba_address);
		}
		ide_drive_buffer_pointer = 0;
		divide_command_status = DIVIDE_COMMAND_DATA_READY;

	}
}

void emu_divide_ports_init() {
	// write to divide_control_register 0x0e3 registered in emu_memory
	register_zx_port_read (0x0a3, divide_data_register_rd);
	register_zx_port_write(0x0a3, divide_data_register_wr);
	register_zx_port_read (0x0a7, divide_error_register_rd);
	register_zx_port_write(0x0a7, io_wr_hang_loop);
	register_zx_port_read (0x0ab, io_rd_hang_loop);
	register_zx_port_write(0x0ab, divide_sector_count_register_wr);
	register_zx_port_read (0x0af, divide_lba0_rd);
	register_zx_port_write(0x0af, divide_lba0_wr);
	register_zx_port_read (0x0b3, divide_lba1_rd);
	register_zx_port_write(0x0b3, divide_lba1_wr);
	register_zx_port_read (0x0b7, divide_lba2_rd);
	register_zx_port_write(0x0b7, divide_lba2_wr);
	register_zx_port_read (0x0bb, divide_drive_head_register_rd);
	register_zx_port_write(0x0bb, divide_drive_head_register_wr);
	register_zx_port_read (0x0bf, divide_status_register_rd);
	register_zx_port_write(0x0bf, divide_command_register_wr);
}
