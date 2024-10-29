/*
 * get_ir_reading.c
 *
 *  Created on: Sep 24, 2024
 *      Author: Ama
 */

#include "get_ir_reading.h"
#include "main.h"
#include "variables.h"
#include <stdio.h>  // For printf


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
    HAL_Delay(2);
    HAL_GPIO_WritePin(LF_EMMITER_GPIO_Port, LF_EMMITER_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EMMITER_GPIO_Port, D_EMMITER_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RF_EMMITER_GPIO_Port, RF_EMMITER_Pin, GPIO_PIN_RESET);

}


/* Function to calibrate IR sensors by finding min and max values */
void Calibrate_IR_Sensors(void)
{
	HAL_Delay(1000);
	buzzer_tone_0();
    for (int i = 0; i < 1000; i++)
    {
        // Set the emitters on
        HAL_GPIO_WritePin(LF_EMMITER_GPIO_Port, LF_EMMITER_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D_EMMITER_GPIO_Port, D_EMMITER_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RF_EMMITER_GPIO_Port, RF_EMMITER_Pin, GPIO_PIN_SET);

        // Read the values from each IR sensor
        LF_reading = Read_ADC(&hadc1, ADC_CHANNEL_0);  // Left Front sensor
        LD_reading = Read_ADC(&hadc1, ADC_CHANNEL_1);  // Left Down sensor
        RD_reading = Read_ADC(&hadc2, ADC_CHANNEL_4);  // Right Down sensor
        RF_reading = Read_ADC(&hadc2, ADC_CHANNEL_5);  // Right Front sensor

        // Update the min and max values for each sensor
        if (LF_reading < LF_min) LF_min = LF_reading;
        if (LF_reading > LF_max) LF_max = LF_reading;

        if (LD_reading < LD_min) LD_min = LD_reading;
        if (LD_reading > LD_max) LD_max = LD_reading;

        if (RD_reading < RD_min) RD_min = RD_reading;
        if (RD_reading > RD_max) RD_max = RD_reading;

        if (RF_reading < RF_min) RF_min = RF_reading;
        if (RF_reading > RF_max) RF_max = RF_reading;

        HAL_Delay(10);  // Optional delay between readings
    }
    buzzer_tone_1();
    HAL_Delay(1000);

}
