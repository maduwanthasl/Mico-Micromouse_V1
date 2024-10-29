/*
 * rotate_angle.h
 *
 *  Created on: Sep 26, 2024
 *      Author: Ama
 */

#ifndef INC_ROTATE_ANGLE_H_
#define INC_ROTATE_ANGLE_H_

#include "stm32f1xx_hal.h"
#include "math.h" // For fabs()
#include "motor_control.h"

// Struct for storing PID controller parameters
typedef struct {
    float rotate_Kp;            // Proportional gain
    float rotate_Ki;            // Integral gain
    float rotate_Kd;            // Derivative gain
    float rotate_previous_error; // Store the previous error
    float rotate_integral;      // Integral sum
} Rotate_PIDController;

// Function prototypes
void Rotate_PID_Init(Rotate_PIDController *rotate_pid, float Kp, float Ki, float Kd);
void rotate(float targetAngle, uint16_t lm_base, uint16_t rm_base);
float gyro_loop(void); // This should be declared in the appropriate gyro file

// External variables
extern float desiredAngle;     // Target angle to reach
extern Rotate_PIDController rotate_pid;
extern float rotate_error;
extern float currentAngle;
extern float rotate_offset;



#endif /* INC_ROTATE_ANGLE_H_ */
