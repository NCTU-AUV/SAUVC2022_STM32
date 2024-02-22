#include "robot_arm.h"

Robot_Arm::Robot_Arm()
{
    speed = 20;
}

Robot_Arm::Robot_Arm(TIM_HandleTypeDef *t, const int sp)
{
    speed = sp;
    this->set(t, sp);
}

Robot_Arm::~Robot_Arm()
{
    this->move(1500);
}

/**
 * @brief Set timer and channel
 * @param t TIM handle
 * @param  i_a initial angle of joints
 * @retval none
 */ 
void Robot_Arm::set(TIM_HandleTypeDef *t, const int sp)
{
    motor[0].set(t, TIM_CHANNEL_1);
    motor[1].set(t, TIM_CHANNEL_2);
    speed = sp;
    this->move(speed + 1500);
    this->rotate(0);
}

/**
 * @brief a function to move joint to desired angle
 * @param angle angle in degree from -90 to 90
 * @retval none
 */
void Robot_Arm::move(const int mode)
{
    // mode 0 long 1 short (PD9 -> AIN2 PD10 -> AIN1 black -> AO1 red -> A02) 
    motor[0].output(speed + 1500);
    if (mode == 0){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
    
    }
    else if(mode == 1){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        
    }
    else if(mode == 2){
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        
    }
}
void Robot_Arm::rotate(const int angle)
{
    motor[1].output(angle * 100 / 9 + 1500);
}

void Robot_Arm::move_to(float distance)
{
    if(distance > 0)
    {
        this->move(0); 
    }
    else
    {
        this->move(1);
        distance *= -1;
    }

    int num_pulse = distance * 2.27;
    int cnt = 0;

    int prev_val, hall_val;
    while(1){
        hall_val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
        if(hall_val == 1 && prev_val == 0){
            cnt++;
        }
        if(cnt == num_pulse)
        {
            this->move(2);
            break;
        }
        prev_val = hall_val;
}
}

void Robot_Arm::reset()
{
    this->move(2);
    int prev_val, current_val;
    prev_val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
    current_val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);

    while(1){
        current_val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);;
        if(prev_val == 0 && current_val ==1)
            return;
        prev_val = current_val;
    }
}
