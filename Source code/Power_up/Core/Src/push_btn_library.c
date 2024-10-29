#include "push_btn_library.h"

uint8_t state_btn;  // Define the variable here

void push_btn(GPIO_TypeDef *btn_port, uint16_t btn_pin, GPIO_TypeDef *need_port, uint16_t need_pin)
{
    state_btn = HAL_GPIO_ReadPin(btn_port, btn_pin);

    if (state_btn == GPIO_PIN_SET)
    {
        HAL_GPIO_WritePin(need_port, need_pin, GPIO_PIN_SET);  // Turn ON LED
    }
    else
    {
        HAL_GPIO_WritePin(need_port, need_pin, GPIO_PIN_RESET);  // Turn OFF LED
    }
}
