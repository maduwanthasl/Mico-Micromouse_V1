/*
 * motor_control.c
 *
 *  Created on: Sep 17, 2024
 *      Author: Ama
 */

#include "motor_control.h"
#include "stm32f1xx_hal.h"  // For HAL functions

// Function to move forward
void forward(uint16_t lm, uint16_t rm) {
    lm = lm * 6;  // Scale PWM value
    rm = rm * 6.18;  // Scale PWM value
    HAL_GPIO_WritePin(LMF_GPIO_Port, LMF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RMF_GPIO_Port, RMF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LMB_GPIO_Port, LMB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RMB_GPIO_Port, RMB_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, rm);  // Set PWM for left motor
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, lm);  // Set PWM for right motor
}

// Function to move backward
void backward(uint16_t lm, uint16_t rm) {
    lm = lm * 6;  // Scale PWM value
    rm = rm * 6.18;  // Scale PWM value
    HAL_GPIO_WritePin(LMF_GPIO_Port, LMF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RMF_GPIO_Port, RMF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMB_GPIO_Port, LMB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RMB_GPIO_Port, RMB_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, rm);  // Set PWM for left motor
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, lm);  // Set PWM for right motor
}

void stop(){
    HAL_GPIO_WritePin(LMF_GPIO_Port, LMF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RMF_GPIO_Port, RMF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMB_GPIO_Port, LMB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RMB_GPIO_Port, RMB_Pin, GPIO_PIN_RESET);
}
