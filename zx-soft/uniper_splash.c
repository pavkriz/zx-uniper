//#include <font/fzx.h>
#include <arch/zx.h>
#include <stdio.h>
#include <stdlib.h>
#include <z80.h>

#include "uniper_splash_common.h"

extern void copy_rom2rom();

#define printInk(k)          printf("\x10%c", '0'+k)
#define printPaper(k)        printf("\x11%c", '0'+k)
#define printAt(row, col)    printf("\x16%c%c", (col), (row)) 

//struct fzx_state fs; 
//struct r_Rect16 screen = { 0, 256, 0, 192 };

void boot_screen() {
  zx_border(INK_BLACK);
  zx_cls(INK_WHITE | PAPER_BLACK);
     
  printPaper(INK_BLACK);
  
  printInk(INK_RED);
  printf("ZX");  
  printInk(INK_WHITE);
  printf("-");
  printInk(INK_YELLOW);
  printf("UN");
  printInk(INK_GREEN);
  printf("IP");
  printInk(INK_CYAN);
  printf("ER\n");

  printInk(INK_WHITE);
     
  //printf ("\n\n\n\n\n\n\n\nHello world!\n");
  //printf ("\nTHIS TEST ROM\n");
  //printf ("\n\n\nProgram end\n");
  
  printf ("\n\nStarting...\n");
  //printf ("Byte at 0x000: %d\n", *pointer); 
  
  //fzx_state_init(&fs, &ff_ao_Grotesk, &screen);

  //fs.fgnd_attr = INK_WHITE | PAPER_BLUE;
  //fs.fgnd_mask = 0;
   
  //fs.y=0; 
   
  //fzx_puts(&fs, "ZX-UNIPER\n\nStarting...");
  
  //while (1) {
  //}
  copy_rom2rom();
} 

void bsod(char *message, uint16_t param1) {
  zx_border(INK_BLUE);
  zx_cls(INK_WHITE | PAPER_BLUE);
     
  printPaper(INK_BLUE);
  printInk(INK_WHITE);
  
  printf("ZX-UNIPER\n\n");  
  printf(":( Blue screen of death\n\n\n\n");  

  printf (message, param1);
  
  while (1) {
  }
} 

void main(void)
{
  uint8_t* code = (uint8_t *)ZX_SPLASH_CODE_ROM_ADDR;
  uint16_t* param1 = (uint16_t *)ZX_SPLASH_PARAM1_ROM_ADDR;
  
  switch(*code) {
  	case ZX_SPLASH_CODE_BOOT:
        boot_screen();
  	case ZX_SPLASH_CODE_UNSUPPORTED_ATA_COMMAND:
        bsod("Unsupported ATA command 0x%x", *param1);
  	case ZX_SPLASH_CODE_UNSUPPORTED_SECTOR_COUNT:
        bsod("Unsupported sector count %d", *param1);
  	case ZX_SPLASH_CODE_PORT_READ:
        bsod("Unsupported read from port 0x%x", *param1);
  	case ZX_SPLASH_CODE_PORT_WRITE:
        bsod("Unsupported read from port 0x%x", *param1);
  	default:
        bsod("Unknown BSOD code 0x%x", code);  
  }

  zx_border(INK_BLACK);
  zx_cls(INK_WHITE | PAPER_BLACK);
     
  printPaper(INK_BLACK);
  
  printInk(INK_RED);
  printf("ZX");  
  printInk(INK_WHITE);
  printf("-");
  printInk(INK_YELLOW);
  printf("UN");
  printInk(INK_GREEN);
  printf("IP");
  printInk(INK_CYAN);
  printf("ER\n");

  printInk(INK_WHITE);
     
  //printf ("\n\n\n\n\n\n\n\nHello world!\n");
  //printf ("\nTHIS TEST ROM\n");
  //printf ("\n\n\nProgram end\n");
  
  printf ("\n\nStarting...\n");
  //printf ("Byte at 0x000: %d\n", *pointer); 
  
  //fzx_state_init(&fs, &ff_ao_Grotesk, &screen);

  //fs.fgnd_attr = INK_WHITE | PAPER_BLUE;
  //fs.fgnd_mask = 0;
   
  //fs.y=0; 
   
  //fzx_puts(&fs, "ZX-UNIPER\n\nStarting...");
  
  while (1) {
      //zx_border(3);
      //z80_delay_ms(1);
      //zx_border(5);
      //z80_delay_ms(1);
  }
}
