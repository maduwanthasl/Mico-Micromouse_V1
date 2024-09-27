/*
 * rotate_angle.c
 *
 *  Created on: Sep 26, 2024
 *      Author: Ama
 */


#include "rotate_angle.h"

// Initialize external variables
float desiredAngle;
Rotate_PIDController rotate_pid;
float rotate_error;
float currentAngle;
float rotate_offset = 0.82; // Default offset

// Initialize PID controller
void Rotate_PID_Init(Rotate_PIDController *rotate_pid, float Kp, float Ki, float Kd) {
    rotate_pid->rotate_Kp = Kp;
    rotate_pid->rotate_Ki = Ki;
    rotate_pid->rotate_Kd = Kd;
    rotate_pid->rotate_previous_error = 0;
    rotate_pid->rotate_integral = 0;
}

// Rotate function with PID control
void rotate(float targetAngle, uint16_t lm_base, uint16_t rm_base) {
    // Initialize PID controller parameters
    Rotate_PID_Init(&rotate_pid, 0.05f, 0.00f, 0.00f); // You can tune these parameters as needed

    // Adjust rotate offset for negative angles
    if (targetAngle < 0)
        rotate_offset = 0.77;

    desiredAngle = targetAngle * rotate_offset;
    float pwm_left, pwm_right;

    // Start rotating until the error is minimal
    while (1) {
        // Read the current angle using gyro_loop
        currentAngle = gyro_loop();

        // Calculate the error
        rotate_error = fabs(desiredAngle - currentAngle);

        HAL_Delay(10); // Small delay

        // Stop when the error is within tolerance
        if ((fabs(desiredAngle) - fabs(currentAngle)) < 2.0f) {
            // Stop motors when within tolerance
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // Stop left motor
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // Stop right motor
            stop();
            HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_SET);
            HAL_Delay(2000);
            break;
        }

        // PID calculations
        rotate_pid.rotate_integral += rotate_error; // Accumulate the integral
        float derivative = rotate_error - rotate_pid.rotate_previous_error; // Derivative
        float output = rotate_pid.rotate_Kp * rotate_error +
                       rotate_pid.rotate_Ki * rotate_pid.rotate_integral +
                       rotate_pid.rotate_Kd * derivative; // Calculate PID output

        // Calculate PWM values
        pwm_left = lm_base + output;   // Adjust left motor PWM
        pwm_right = rm_base + output;  // Adjust right motor PWM

        // Limit PWM values to avoid negative values
        if (pwm_left < 0) pwm_left = 0;
        if (pwm_right < 0) pwm_right = 0;

        // Set motor speeds based on the target angle
        if (targetAngle == 90) {
            turn_90(pwm_left, pwm_right);
        } else if (targetAngle == 180) {
            turn_180(pwm_left, pwm_right);
        } else if (targetAngle == -90) {
            turn_270(pwm_left, pwm_right);
        }

        // Store current error for next iteration
        rotate_pid.rotate_previous_error = rotate_error;

        // Small delay for stability
        HAL_Delay(10);
    }
}
