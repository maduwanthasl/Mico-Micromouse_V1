/*
 * get_ir_reading.c
 *
 *  Created on: Sep 24, 2024
 *      Author: Ama
 */

#include "get_ir_reading.h"
#include "main.h"
#include <stdio.h>  // For printf

// Define variables to store IR sensor readings
uint16_t LF_reading = 0;
uint16_t LD_reading = 0;
uint16_t RD_reading = 0;
uint16_t RF_reading = 0;

/* Function to read from a specified ADC channel */
uint16_t Read_ADC(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);  // Wait for conversion
    uint16_t value = HAL_ADC_GetValue(hadc);  // Get ADC value
    HAL_ADC_Stop(hadc);

    return value;
}

/* Function to get IR sensor readings */
void Get_IR_Readings(void)
{
    HAL_GPIO_WritePin(LF_EMMITER_GPIO_Port, LF_EMMITER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_EMMITER_GPIO_Port, D_EMMITER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RF_EMMITER_GPIO_Port, RF_EMMITER_Pin, GPIO_PIN_SET);
    // Read the values from each IR sensor
    LF_reading = Read_ADC(&hadc1, ADC_CHANNEL_0);  // Left Front sensor
    LD_reading = Read_ADC(&hadc1, ADC_CHANNEL_1);  // Left Down sensor
    RD_reading = Read_ADC(&hadc2, ADC_CHANNEL_4);  // Right Down sensor
    RF_reading = Read_ADC(&hadc2, ADC_CHANNEL_5);  // Right Front sensor

}
