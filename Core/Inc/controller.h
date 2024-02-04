#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Datatype/dynamics.h"

const float weight = 24.3;
const float buoyancy = 25.62;

class Controller
{
private:
    const geometry::Vector Kx;
    const geometry::Vector Kv;
    const geometry::Vector KR;
    const geometry::Vector KOmega;
    const float Alpha_sonar;  //0~1 how much we trust sonar yaw
    float **Rd;
    float **R;
    float **Re;
    geometry::Vector ex;
    geometry::Vector ev;
    geometry::Vector eR;
    geometry::Vector eOmega;

public:
    Controller(geometry::Vector x, geometry::Vector v, geometry::Vector R, geometry::Vector Omega, float alpha);
    ~Controller();
    // void adjust_yaw(Dynamics &s, float ys);
    void set(const Quaternion &qd);
    void update(Dynamics &s, const geometry::Vector &ex, const geometry::Vector &ev, float yaw_sonar, Kinematics &ctrl_input);
    geometry::Vector* get_ex();
};



#endif