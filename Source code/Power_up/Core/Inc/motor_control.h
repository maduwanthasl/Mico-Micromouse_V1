/*
 * motor_control.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Ama
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim4;  // Declare the correct timer handle as extern

void forward(uint16_t lm, uint16_t rm);
void backward(uint16_t lm, uint16_t rm);
void turn_90(uint16_t lm, uint16_t rm);
void turn_180(uint16_t lm, uint16_t rm);
void turn_270(uint16_t lm, uint16_t rm);
void stop();

#endif /* INC_MOTOR_CONTROL_H_ */
