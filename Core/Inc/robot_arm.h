#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "motor.h"
#include "gpio.h"

class Robot_Arm
{
private:
    Motor motor[2];
    int speed;

public:
    Robot_Arm();
    Robot_Arm(TIM_HandleTypeDef *t, const int sp);
    ~Robot_Arm();
    void set(TIM_HandleTypeDef *t, const int sp);
    void move(const int mode);
    void rotate(const int angle);
    void move_to(float distance);
    void reset();
};

#endif