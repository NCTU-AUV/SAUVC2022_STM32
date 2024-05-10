#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "vector.h"
#include "quaternion.h"

struct Kinematics
{
    Vector3D linear;
    Vector3D angular;
};

struct Dynamics
{
    Vector3D position;
    Quaternion orientation;

    Kinematics velocity;
    Kinematics acceleration;
};

#endif