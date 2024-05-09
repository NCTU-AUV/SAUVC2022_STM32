#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Datatype/dynamics.h"

const float weight = 24.3;
const float buoyancy = 25.35;

class Controller
{
private:
    geometry::Vector Kx;
    geometry::Vector Kv;
    geometry::Vector KR;
    geometry::Vector KOmega;
    // 0~1 how much we trust sonar yaw
    const float Alpha_sonar;
    float Rd[3][3];
    float R[3][3];
    float Re[3][3];
    geometry::Vector ex;
    geometry::Vector ev;
    geometry::Vector eR;
    geometry::Vector eOmega;

public:
    Controller(geometry::Vector x, geometry::Vector v, geometry::Vector R, geometry::Vector Omega, float alpha);

    void set(const Quaternion &qd);
    void update(Dynamics &s, const geometry::Vector &ex, const geometry::Vector &ev, float yaw_sonar, Kinematics &ctrl_input);
    void set_eR(geometry::Vector eR_rec);
    void set_kR(geometry::Vector kr);
    geometry::Vector *get_ex();
    geometry::Vector get_eR();
};

#endif