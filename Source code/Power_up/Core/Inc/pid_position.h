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
#include "variables.h"
#include "get_ir_reading.h"
#include "main.h"

// PID controller structure
typedef struct {
    float Kp;              // Proportional gain
    float Ki;              // Integral gain
    float Kd;              // Derivative gain
    float setpoint;        // Target count value
    float integral;        // Integral term accumulator
    float previous_error;  // Error from the previous loop iteration
} PID_Controller;

// Struct to store PID controller parameters
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
} IR_Control_PIDController;

// Function prototypes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim3);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Update(PID_Controller *pid, float current_value);
void motor_control_loop_with_pid(void);

// Function prototypes
void ir_control_pid_init(IR_Control_PIDController *ir_control_pid, float Kp, float Ki, float Kd);
float ir_control_pid_function(IR_Control_PIDController *ir_control_pid, uint16_t LD_reading, uint16_t RD_reading, uint16_t LD_min, uint16_t RD_min);

#endif /* INC_PID_POSITION_H_ */
