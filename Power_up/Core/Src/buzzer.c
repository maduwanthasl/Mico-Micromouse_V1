/*
 * buzzer.c
 *
 *  Created on: Sep 17, 2024
 *      Author: Ama
 */

#include "buzzer.h"

// You might need to include this to use HAL functions
#include "stm32f1xx_hal.h"

// Melody frequencies (in Hz)
int melody[] = {
    440, 494, 466, 440, 392, 659, 784, 880, 698, 784, 659, 523, 587, 494
};

int noteDurations[] = {
    150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150
};

void buzzer_power_up_tone(){
    int size = sizeof(melody) / sizeof(melody[0]);

    for (int i = 0; i < size; i++) {
        if (melody[i] == 0) {
            // Pause for a rest
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        } else {
            // Calculate the compare value for the desired frequency
            uint32_t compareValue = (90000 / melody[i]) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim1, compareValue);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compareValue / 2); // Adjust duty cycle for better sound
        }
        HAL_Delay(noteDurations[i]);
    }

    // Silence the buzzer after the melody
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

void buzzer_tone_0() {
	    for (int i = 0; i < 1; i++) {
	        if (melody[i] == 0) {
	            // Pause for a rest
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	        } else {
	            // Calculate the compare value for the desired frequency
	            uint32_t compareValue = (90000 / melody[i]) - 1;
	            __HAL_TIM_SET_AUTORELOAD(&htim1, compareValue);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compareValue / 2); // Adjust duty cycle for better sound
	        }
	        HAL_Delay(noteDurations[i]);
	    }

	    // Silence the buzzer after the melody
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

void buzzer_tone_1() {
	    for (int i = 0; i < 3; i++) {
	        if (melody[i] == 0) {
	            // Pause for a rest
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	        } else {
	            // Calculate the compare value for the desired frequency
	            uint32_t compareValue = (90000 / melody[i]) - 1;
	            __HAL_TIM_SET_AUTORELOAD(&htim1, compareValue);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compareValue / 2); // Adjust duty cycle for better sound
	        }
	        HAL_Delay(noteDurations[i]);
	    }

	    // Silence the buzzer after the melody
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}
