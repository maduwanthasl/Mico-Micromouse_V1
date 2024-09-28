/*
 * variables.h
 *
 *  Created on: Sep 27, 2024
 *      Author: Ama
 */

#ifndef VARIABLES_H
#define VARIABLES_H

#include <stdint.h>

// Declare variables to store IR sensor readings
extern uint16_t LF_reading;
extern uint16_t LD_reading;
extern uint16_t RD_reading;
extern uint16_t RF_reading;

// Declare the min and max values for each sensor
extern uint16_t LF_min, LF_max;
extern uint16_t LD_min, LD_max;
extern uint16_t RD_min, RD_max;
extern uint16_t RF_min, RF_max;

// Declare the control output variable
extern float ir_control_output;

extern uint32_t counter;

#endif // VARIABLES_H
