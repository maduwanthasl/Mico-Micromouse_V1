/*
 * push_btn_library.h
 *
 *  Created on: Sep 16, 2024
 *      Author: Ama
 */

#ifndef INC_PUSH_BTN_LIBRARY_H_
#define INC_PUSH_BTN_LIBRARY_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

extern uint8_t state_btn;

void push_btn(GPIO_TypeDef *btn_port, uint16_t btn_pin, GPIO_TypeDef *need_port, uint16_t need_pin);

#endif /* INC_PUSH_BTN_LIBRARY_H_ */
