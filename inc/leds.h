#ifndef __LEDS_H_
#define __LEDS_H_

#define LED1_PORT 0      // Port for led
#define LED2_PORT 0
#define LED3_PORT 0

#define LED1_BIT 12       // Bit on port for led
#define LED2_BIT 13       // Bit on port for led
#define LED3_BIT 15

#define LED1_ON() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED1_PORT, LED1_BIT, 1);}
#define LED1_OFF() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED1_PORT, LED1_BIT, 0);}
#define LED2_ON() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED2_PORT, LED2_BIT, 1);}
#define LED2_OFF() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED2_PORT, LED2_BIT, 0);}
#define LED3_ON() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED3_PORT, LED3_BIT, 1);}
#define LED3_OFF() {Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED3_PORT, LED3_BIT, 0);}

void leds_init();

#endif
