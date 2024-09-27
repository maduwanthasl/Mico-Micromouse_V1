/*
 * variables.c
 *
 *  Created on: Sep 27, 2024
 *      Author: Ama
 */


#include "variables.h"

// Define variables to store IR sensor readings
uint16_t LF_reading = 0;
uint16_t LD_reading = 0;
uint16_t RD_reading = 0;
uint16_t RF_reading = 0;

// Initialize the min and max values for each sensor
uint16_t LF_min = 0xFFFF, LF_max = 0;
uint16_t LD_min = 0xFFFF, LD_max = 0;
uint16_t RD_min = 0xFFFF, RD_max = 0;
uint16_t RF_min = 0xFFFF, RF_max = 0;

// Define the control output variable
float ir_control_output = 0.0f;
