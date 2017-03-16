/*
 * utils.h
 *
 *  Created on: 19. 11. 2016
 *      Author: Kriz
 */

#ifndef UTILS_H_
#define UTILS_H_

#define FAST_CODE __attribute__((section(".fast_code")))
#define HANG_LOOP() {HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); while (1) {}}

void strtrim(char *str);
void copy_right_padded(char *dst, char *src, int size_of_dst);
void scramble_ata_string(char *str, int size);

#endif /* UTILS_H_ */
