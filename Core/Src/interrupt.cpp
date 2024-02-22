#include "interrupt.h"

Switch::Switch()
{
    interrupt = true;
}

Switch::~Switch()
{
    interrupt = false;
}


void Switch::read_state()
{
    interrupt = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
}


bool Switch::get_state()
{
    return interrupt;
}