
/*
 * mpu6050.c
 *
 *  Created on: Sep 23, 2024
 *      Author: Ama
 */


#include "mpu6050.h"

float gyroCalibrationFactor = 0.0f;
float gyroDegree = 0.0f;
int isFirstLoopComplete = 0;
uint32_t previousTime = 0;

void MPU6050_Init(void) {
    uint8_t check;
    uint8_t data;

    // Check if MPU6050 is connected
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    if (check == 0x68) {
        // Wake up MPU6050
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        // Set gyro configuration, full scale = ±250 degree/sec
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

        // Set accelerometer configuration, full scale = ±2g
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
    }
}

void MPU6050_Read_GyroZ(int16_t *gyroZ) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_ZOUT_H, 1, data, 2, 1000);

    *gyroZ = (int16_t)(data[0] << 8 | data[1]);
}

void MPU6050_CalibrateGyro(void) {
	HAL_Delay(1000);
	buzzer_tone_0();
    // Calibrate gyro here by taking multiple readings and averaging
    int32_t sum = 0;
    int16_t gyroZ;
    for (int i = 0; i < 5000; i++) {
        MPU6050_Read_GyroZ(&gyroZ);
        sum += gyroZ;
        HAL_Delay(1);
    }
    gyroCalibrationFactor = sum / 5000.0;
    buzzer_tone_1();
    HAL_Delay(1000);
}

float gyro_loop(void) {
    int16_t gyroZRaw;
    MPU6050_Read_GyroZ(&gyroZRaw);

    // Adjust for calibration and apply the calibration factor
    float gyroZ = (gyroZRaw - gyroCalibrationFactor) / 131.0f;

    // Calculate time difference
    uint32_t currentTime = HAL_GetTick(); // Get current tick (ms)
    float timeForOneLoop = (currentTime - previousTime) * 0.001f; // Convert ms to seconds

    // Update degree based on gyroscope reading
    if (isFirstLoopComplete) {
        gyroDegree += gyroZ * timeForOneLoop; // Calculate degree change
    }

    previousTime = currentTime;

    if (!isFirstLoopComplete) {
        isFirstLoopComplete = true;
    }

    return gyroDegree;
}

void delayMicroseconds(uint32_t micros) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < (micros / 1000)) {
        // Wait
    }
}

void ResetGyroDegree(void) {
    gyroDegree = 0.0f;    // Reset the accumulated angle
    isFirstLoopComplete = false; // Mark the first loop as incomplete
}

