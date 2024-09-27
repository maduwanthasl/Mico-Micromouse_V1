/*
 * get_ir_reading.h
 *
 *  Created on: Sep 24, 2024
 *      Author: Ama
 */

#ifndef INC_GET_IR_READING_H_
#define INC_GET_IR_READING_H_

#include "stm32f1xx_hal.h"  // For HAL functions
#include "buzzer.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

// Declare variables to store IR sensor readings
extern uint16_t LF_reading;
extern uint16_t LD_reading;
extern uint16_t RD_reading;
extern uint16_t RF_reading;

// Min and Max values for calibration
extern uint16_t LF_min, LF_max;
extern uint16_t LD_min, LD_max;
extern uint16_t RD_min, RD_max;
extern uint16_t RF_min, RF_max;

// Declare function prototypes
uint16_t Read_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
void Get_IR_Readings(void);
void Calibrate_IR_Sensors(void);


#endif /* INC_GET_IR_READING_H_ */
