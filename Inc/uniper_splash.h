/*
 * uniper_splash.h
 *
 *  Created on: 11. 3. 2017
 *      Author: mazlik
 */

#ifndef UNIPER_SPLASH_H_
#define UNIPER_SPLASH_H_

#include <stdint.h>

#include "../zx-soft/uniper_splash_common.h"

void uniper_splash_show_boot();
void uniper_splash_bsod(uint8_t message_code, uint16_t param1);

#endif /* UNIPER_SPLASH_H_ */
