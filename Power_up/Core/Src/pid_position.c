/*
 * pid_position.c
 *
 *  Created on: Sep 23, 2024
 *      Author: Ama
 */

/*
 * pid_position.c
 *
 *  Created on: Sep 23, 2024
 *      Author: Ama
 */

#include "pid_position.h"

// Define variables
uint32_t counter = 0;
uint32_t count2 = 0;
volatile uint32_t last_interrupt_time = 0;  // To store the time of the last valid interrupt
int16_t count = 0;

// Callback function to capture encoder count
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim3) {
    counter = __HAL_TIM_GET_COUNTER(htim3);
    count = (int16_t)counter;
}

// External interrupt callback (for debouncing)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();  // Get current system tick (milliseconds)

    if (current_time - last_interrupt_time > 2) {  // Debounce logic
        count2 += 1;
        last_interrupt_time = current_time;
    }
}

// Initialize the PID controller
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

// Compute the PID output
float PID_Update(PID_Controller *pid, float current_value) {
    float error = pid->setpoint - current_value;

    // Integral term with windup guard
    pid->integral += error;
    if (pid->integral > 1000) pid->integral = 1000;  // Clamp integral
    if (pid->integral < -1000) pid->integral = -1000;

    float derivative = error - pid->previous_error;

    // Calculate control signal
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    pid->previous_error = error;

    return output;
}

// Define PID controller instance
PID_Controller pid;

#define ERROR 10
#define TARGET_POS 200
#define TARGET_COUNT (TARGET_POS + ERROR)   // Target encoder count

#define PID_KP 0.2f          // Proportional gain
#define PID_KI 0.005f        // Integral gain
#define PID_KD 0.05f         // Derivative gain

float control_signal;

// Motor control loop with PID and overshoot correction
void motor_control_loop_with_pid(void) {
    PID_Init(&pid, PID_KP, PID_KI, PID_KD, TARGET_COUNT);

    while (1) {
        control_signal = PID_Update(&pid, (float)count);  // Get control signal from PID

        if (count < TARGET_COUNT) {
            // Clamp control signal for PWM
            if (control_signal > 20) control_signal = 20;
            if (control_signal < -20) control_signal = -20;

            if ((count <= TARGET_COUNT) && (count > TARGET_POS)) {
                count = 0;
                stop();   // Stop the motor
                HAL_Delay(50);
                HAL_GPIO_WritePin(LED_SPD_GPIO_Port, LED_SPD_Pin, GPIO_PIN_SET);  // Indicate stop with LED
                break;
            }

            // Adjust motor speed based on control signal
            if (control_signal > 0) {
                forward((uint8_t)control_signal, (uint8_t)control_signal);  // Move forward
            } else {
                backward((uint8_t)(-control_signal), (uint8_t)(-control_signal));  // Move backward
            }
        }
        HAL_Delay(10);  // Prevent CPU overload
    }
}
