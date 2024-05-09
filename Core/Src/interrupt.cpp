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
    interrupt = HAL_GPIO_ReadPin(KILL_SWITCH_GPIO_Port, KILL_SWITCH_Pin);
}


bool Switch::get_state()
{
    return interrupt;
}