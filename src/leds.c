#include "leds.h"
#include "chip.h"

void leds_init() {
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED1_PORT, LED1_BIT);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED2_PORT, LED2_BIT);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED3_PORT, LED3_BIT);

    // set P1_17 (GPIO0[12]) to 14mA high-drive mode
    LPC_SCU->SFSP[1][17] = SCU_MODE_FUNC0|SCU_MODE_20MA_DRIVESTR;
    // set P1_18 (GPIO0[13]) to 14mA high-drive mode
    LPC_SCU->SFSP[1][18] = SCU_MODE_FUNC0|SCU_MODE_20MA_DRIVESTR;
    // set P1_20 (GPIO0[15]) to 14mA high-drive mode
    LPC_SCU->SFSP[1][20] = SCU_MODE_FUNC0|SCU_MODE_20MA_DRIVESTR;
}
