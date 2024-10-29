/*
 * buzzer.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Ama
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "stm32f1xx_hal.h" // Include this if `TIM_HandleTypeDef` is defined here

extern TIM_HandleTypeDef htim1; // Declare `htim1` as extern

void buzzer_power_up_tone();
void buzzer_tone_0();
void buzzer_tone_1();

#endif /* INC_BUZZER_H_ */
