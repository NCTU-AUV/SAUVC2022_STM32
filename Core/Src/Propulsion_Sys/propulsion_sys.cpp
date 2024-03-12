#include "Propulsion_Sys/propulsion_sys.h"


extern int operate;

Propulsion_Sys::Propulsion_Sys()
{
    for (int i = 0; i < 8; i++)
        thrust[i] = 0;
}

Propulsion_Sys::~Propulsion_Sys()
{
    for (int i = 0; i < 8; i++)
        motor[i].output(0);
}

/**
 * @brief Set timer and channel
 * @param tim1 TIM handle
 * @param tim2 TIM handle
 * @retval none
 */
void Propulsion_Sys::set_timer(TIM_HandleTypeDef *tim1, TIM_HandleTypeDef *tim2)
{
    //timer2
    motor[0].set(tim1, TIM_CHANNEL_1);
    motor[1].set(tim1, TIM_CHANNEL_2);
    motor[2].set(tim1, TIM_CHANNEL_3);
    motor[3].set(tim1, TIM_CHANNEL_4);

    //timer8
    motor[4].set(tim2, TIM_CHANNEL_1);
    motor[5].set(tim2, TIM_CHANNEL_2);
    motor[6].set(tim2, TIM_CHANNEL_3);
    motor[7].set(tim2, TIM_CHANNEL_4); 
}

/**
 * @brief allocate control_input(6D force and moment) to each motor thrust then output
 * @param ctrl_input Kinematics class
 * @retval none
 */ 
void Propulsion_Sys::allocate(const Kinematics &ctrl_input)
{
    //allocation matrix expand
    /*
    thrust[0] = ctrl_input.linear.x * 0 + ctrl_input.linear.y * 0 + ctrl_input.linear.z * 0.25
                + ctrl_input.angular.x * 1.1035 + ctrl_input.angular.y * -1.3514 + ctrl_input.angular.z * 0;

    thrust[1] = ctrl_input.linear.x * 0 + ctrl_input.linear.y * 0 + ctrl_input.linear.z * 0.25
                + ctrl_input.angular.x * 1.1035 + ctrl_input.angular.y * 1.3514 + ctrl_input.angular.z * 0;

    thrust[2] = ctrl_input.linear.x * 0 + ctrl_input.linear.y * 0 + ctrl_input.linear.z * 0.25
                + ctrl_input.angular.x * -1.1035 + ctrl_input.angular.y * -1.3514 + ctrl_input.angular.z * 0;

    thrust[3] = ctrl_input.linear.x * 0 + ctrl_input.linear.y * 0 + ctrl_input.linear.z * 0.25
                + ctrl_input.angular.x * -1.1035 + ctrl_input.angular.y * 1.3514 + ctrl_input.angular.z * 0;

    thrust[4] = ctrl_input.linear.x * -0.3536 + ctrl_input.linear.y * 0.3536 + ctrl_input.linear.z * 0 
                + ctrl_input.angular.x * 0 + ctrl_input.angular.y * 0 + ctrl_input.angular.z * 0.783;

    thrust[5] = ctrl_input.linear.x * 0.3536 + ctrl_input.linear.y * 0.3536 + ctrl_input.linear.z * 0 
                + ctrl_input.angular.x * 0 + ctrl_input.angular.y * 0 + ctrl_input.angular.z * -0.783;

    thrust[6] = ctrl_input.linear.x * 0.3536 + ctrl_input.linear.y * 0.3536 + ctrl_input.linear.z * 0 
                + ctrl_input.angular.x * 0 + ctrl_input.angular.y * 0 + ctrl_input.angular.z * 0.783;

    thrust[7] = ctrl_input.linear.x * -0.3536 + ctrl_input.linear.y * 0.3536 + ctrl_input.linear.z * 0 
                + ctrl_input.angular.x * 0 + ctrl_input.angular.y * 0 + ctrl_input.angular.z * -0.783;
    */
    
    thrust[0] = ctrl_input.linear.x  * 0 + ctrl_input.linear.y  *  0 + ctrl_input.linear.z  * 0.25 + 
                ctrl_input.angular.x * -1.0549 + ctrl_input.angular.y * -1.6667 + ctrl_input.angular.z * 0;

    thrust[1] = ctrl_input.linear.x  * 0 + ctrl_input.linear.y  * 0 + ctrl_input.linear.z * 0.25 + 
                ctrl_input.angular.x *  1.0549 + ctrl_input.angular.y * -1.6667 + ctrl_input.angular.z * 0;

    thrust[2] = ctrl_input.linear.x  *  0 + ctrl_input.linear.y  *  0 + ctrl_input.linear.z * 0.25 +
                ctrl_input.angular.x * -1.0549 + ctrl_input.angular.y *  1.6667 + ctrl_input.angular.z * 0;

    thrust[3] = ctrl_input.linear.x  *  0 + ctrl_input.linear.y  * 0 + ctrl_input.linear.z * 0.25 +
                ctrl_input.angular.x *  1.0549 + ctrl_input.angular.y *  1.6667 + ctrl_input.angular.z * 0;

    thrust[4] = ctrl_input.linear.x  * 0.3536 + ctrl_input.linear.y   *  0.3536 + ctrl_input.linear.z * 0 +
                ctrl_input.angular.x * 0      + ctrl_input.angular.y  *  0      + ctrl_input.angular.z * 0.6939;

    thrust[5] = ctrl_input.linear.x  * 0.3536 + ctrl_input.linear.y   * -0.3536 + ctrl_input.linear.z * 0 +
                ctrl_input.angular.x * 0      + ctrl_input.angular.y  *  0      + ctrl_input.angular.z * -0.6939;

    thrust[6] = ctrl_input.linear.x  * 0.3536 + ctrl_input.linear.y   * -0.3536 + ctrl_input.linear.z * 0 +
                ctrl_input.angular.x * 0      + ctrl_input.angular.y  *  0      + ctrl_input.angular.z *  0.6939;

    thrust[7] = ctrl_input.linear.x  * 0.3536 + ctrl_input.linear.y   *  0.3536 + ctrl_input.linear.z * 0 +
                ctrl_input.angular.x * 0      + ctrl_input.angular.y  *  0      + ctrl_input.angular.z * -0.6939;
    
    /*thrust[0] = 0.1;
    thrust[1] = 0.1;
    thrust[2] = 0.1;
    thrust[3] = 0.1;
    thrust[4] = 0.1;
    thrust[5] = 0.1;
    thrust[6] = 0.1;
    thrust[7] = 0.1;*/
    
    //Adjest each motor
    thrust[0] *= 1;
    thrust[1] *= 1;
    thrust[2] *= 1;
    thrust[3] *= 1;
    thrust[4] *= 1;
    thrust[5] *= 1;
    thrust[6] *= 1;
    thrust[7] *= 1;
    
    //Reverse direction
    thrust[0] *= -1;
    thrust[3] *= -1;
    thrust[4] *= -1;
    thrust[7] *= -1;
    
    //output thurst
    
    for(int i=0; i < 8; i++)
        motor[i].output(operate * thrust[i]);
    
    /*for(int i=0; i < 8; i++)
        motor[i].output(operate * 0.2);*/
        
    /*for (int i = 0; i < 8; i++){
        for(int k = 0; k < 8; k++){
            if (k == i)
                motor[k].output(operate * 0.2);
            else
                motor[k].output(operate * 0);
            HAL_Delay(500);
        }
    }*/
        
}