#include "Propulsion_Sys/t200.h"

T200::T200()
{
}

T200::T200(TIM_HandleTypeDef* t, uint32_t c)
{
    this->set(t, c);
}

T200::~T200()
{
    motor.output(1500);
}

/**
 * @brief Set timer and channel
 * @param t TIM handle
 * @param  c TIM Channels to be enabled
 *          This parameter can be one of the following values:
 *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
 *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
 *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
 *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
 */ 
void T200::set(TIM_HandleTypeDef* t, uint32_t c)
{
    motor.set(t, c);
    motor.output(1500);
}

/**
 * @brief Transform force value to PWM signal then output
 * @param force force in kgf
 * @retval none
 */ 
void T200::output(const float &force)
{
    int signal = 1500;
    int high = 200;
    int low = 0;
    int mid = 100;

    //use binary search to find the closest region 
    while (low < high)
    {
        mid = (high + low) / 2;
        if (force > FORCE[mid])
            low = mid + 1;
        else if (force < FORCE[mid])
            high = mid - 1;
        else
            break;
    }
    //find the closet value
    if (fabs(force - FORCE[mid]) > fabs(force - FORCE[mid - 1]))
        signal = SIGNAL[mid - 1];
    else if (fabs(force - FORCE[mid]) > fabs(force - FORCE[mid + 1]))
        signal = SIGNAL[mid + 1];
    else
        signal = SIGNAL[mid];

    // Boundary 1244-1752 
    if (signal > 1752)      signal = 1752;
    else if (signal < 1244) signal = 1244;
    /*
    // abs(signal diff) < 90 (avoid inrush current)
    int diff = signal - prev_signal;
    if(diff > DIFF_MAX)    signal = prev_signal + DIFF_MAX;
    else if(diff<DIFF_MAX) signal = prev_signal - DIFF_MAX;
    */

    prev_signal = signal;
    //output PWM signal
    motor.output(signal);
}