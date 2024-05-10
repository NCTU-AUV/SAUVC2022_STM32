#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Datatype/dynamics.h"

const float weight = 24.3;
const float buoyancy = 25.35;

class Controller
{
private:
    Vector3D Kx;
    Vector3D Kv;
    Vector3D KR;
    Vector3D KOmega;
    // 0~1 how much we trust sonar yaw
    const float Alpha_sonar;
    float Rd[3][3];
    float R[3][3];
    float Re[3][3];
    Vector3D ex;
    Vector3D ev;
    Vector3D eR;
    Vector3D eOmega;

public:
    Controller(Vector3D x, Vector3D v, Vector3D R, Vector3D Omega, float alpha);

    void set(const Quaternion &qd);
    void update(Dynamics &s, const Vector3D &ex, const Vector3D &ev, float yaw_sonar, Kinematics &ctrl_input);
    void set_eR(Vector3D eR_rec);
    void set_kR(Vector3D kr);
    Vector3D *get_ex();
    Vector3D get_eR();
};

#endif