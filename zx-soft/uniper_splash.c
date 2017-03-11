//#include <font/fzx.h>
#include <arch/zx.h>
#include <stdio.h>
#include <stdlib.h>
#include <z80.h>

#define printInk(k)          printf("\x10%c", '0'+k)
#define printPaper(k)        printf("\x11%c", '0'+k)
#define printAt(row, col)    printf("\x16%c%c", (col), (row)) 

//struct fzx_state fs; 
//struct r_Rect16 screen = { 0, 256, 0, 192 };
 

void main(void)
{
  uint8_t* pointer = (int *)0x0000;

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
