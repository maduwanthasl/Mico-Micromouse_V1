/*
 * mpu6050.h
 *
 *  Created on: Sep 23, 2024
 *      Author: Ama
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>  // Include stdbool.h for bool, true, false

// MPU6050 addresses and register definitions
#define MPU6050_ADDR 0x68 << 1 // Shifted for HAL I2C
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_ZOUT_H 0x47
#define ACCEL_XOUT_H 0x3B

#define GYRO_Z_OFFSET 16

extern float gyroCalibrationFactor; // Use extern keyword
extern float gyroDegree;
extern int isFirstLoopComplete;
extern uint32_t previousTime;


// Define external variables for I2C and time
extern I2C_HandleTypeDef hi2c2;

// Function prototypes
void MPU6050_Init(void);
void MPU6050_Read_GyroZ(int16_t *gyroZ);
void MPU6050_CalibrateGyro(void);
float gyro_loop(void);
void delayMicroseconds(uint32_t micros);

#endif /* INC_MPU6050_H_ */
