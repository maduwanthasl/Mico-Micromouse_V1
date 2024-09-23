/*
 * pid_position.h
 *
 *  Created on: Sep 23, 2024
 *      Author: Ama
 */

#ifndef INC_PID_POSITION_H_
#define INC_PID_POSITION_H_

#include "stm32f1xx_hal.h"  // For HAL functions
#include <motor_control.h>

// PID controller structure
typedef struct {
    float Kp;              // Proportional gain
    float Ki;              // Integral gain
    float Kd;              // Derivative gain
    float setpoint;        // Target count value
    float integral;        // Integral term accumulator
    float previous_error;  // Error from the previous loop iteration
} PID_Controller;

// Function prototypes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim3);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Update(PID_Controller *pid, float current_value);
void motor_control_loop_with_pid(void);

#endif /* INC_PID_POSITION_H_ */
