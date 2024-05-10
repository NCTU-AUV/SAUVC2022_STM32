#include "controller.h"
#include "math.h"

void er_mul(const float a[3][3], const float b[3][3], float result[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float value = 0;
            for (int k = 0; k < 3; k++)
            {
                value += a[k][i] * b[k][j] - a[i][k] * b[j][k];
            }
            result[i][j] = value;
        }
    }
}

void matrix_mul(const float a[3][3], const float b[3][3], float result[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float value = 0;
            for (int k = 0; k < 3; k++)
                value += a[i][k] * b[k][j];
            result[i][j] = value;
        }
    }
}

void qtoR(const Quaternion &qd, float m[3][3])
{
    m[0][0] = 1 - 2 * (qd.y * qd.y + qd.z * qd.z);
    m[0][1] = 2 * (qd.x * qd.y + qd.w * qd.z);
    m[0][2] = 2 * (qd.x * qd.z - qd.w * qd.y);
    m[1][0] = 2 * (qd.x * qd.y - qd.w * qd.z);
    m[1][1] = 1 - 2 * (qd.x * qd.x + qd.z * qd.z);
    m[1][2] = 2 * (qd.y * qd.z + qd.w * qd.x);
    m[2][0] = 2 * (qd.x * qd.z + qd.w * qd.y);
    m[2][1] = 2 * (qd.y * qd.z - qd.w * qd.x);
    m[2][2] = 1 - 2 * (qd.x * qd.x + qd.y * qd.y);
}

/**
 * @brief Constructor of Controller
 * @param x Kx
 * @param v Kv
 * @param R KR
 * @param Omega KOmega
 * @param alpha 0~1 how much we trust sonar yaw
 */
Controller::Controller(Vector3D x, Vector3D v, Vector3D kR, Vector3D Omega, float alpha) : Kx(x), Kv(v), KR(kR), KOmega(Omega), Alpha_sonar(alpha)
{
}

void Controller::set(const Quaternion &qd)
{
    qtoR(qd, Rd);
}

void Controller::set_eR(Vector3D eR_rec)
{
    eR = eR_rec;
}

void Controller::set_kR(Vector3D kr)
{
    KR = kr;
}

void Controller::update(Dynamics &s, const Vector3D &ex, const Vector3D &ev, float yaw_sonar, Kinematics &ctrl_input)
{
    // Calculate attitude error
    /*qtoR(s.orientation, R);
    er_mul(Rd, R, Re);
    eR.x = Re[2][1];
    eR.y = Re[0][2];
    eR.z = Re[1][0];*/
    //(-1) * yaw_sonar;
    // eR.z = 0.5;

    eOmega.x = s.velocity.angular.x;
    eOmega.y = s.velocity.angular.y;
    eOmega.z = s.velocity.angular.z;

    ctrl_input.linear.x = Kx.x * ex.x + Kv.x * ev.x - 0.099 * abs(ex.y);
    ctrl_input.linear.y = Kx.y * ex.y + Kv.y * ev.y;
    ctrl_input.linear.z = Kx.z * ex.z + Kv.z * ev.z - weight + buoyancy;
    ctrl_input.angular.x = KR.x * eR.x + KOmega.x * eOmega.x;
    ctrl_input.angular.y = KR.y * eR.y + KOmega.y * eOmega.y;
    if ((eR.z > 90) || (eR.z < -90))
    {
        if (eR.z > 0)
            ctrl_input.angular.z = 0.2;
        else
            ctrl_input.angular.z = -0.2;
        ctrl_input.linear.x = 0;
        ctrl_input.linear.y = 0;
    }
    else if ((eR.z > 45) || (eR.z < -45))
    {
        if (eR.z > 0)
            ctrl_input.angular.z = 0.1;
        else
            ctrl_input.angular.z = -0.1;
        ctrl_input.linear.x = 0;
        ctrl_input.linear.y = 0;
    }
    else if ((eR.z > 5) || (eR.z < -5))
    {
        if (eR.z > 0)
            ctrl_input.angular.z = 0.07;
        else
            ctrl_input.angular.z = -0.07;
        ctrl_input.linear.x = 0;
        ctrl_input.linear.y = 0;
    }
    else if ((eR.z < 5) || (eR.z > -5))
        ctrl_input.angular.z = KR.z * eR.z + KOmega.z * eOmega.z;

    /*if((eR.x > 10) || (eR.y > 10))
        ctrl_input.angular.z = 0;*/
}

Vector3D Controller::get_eR()
{
    return eR;
}

Vector3D *Controller::get_ex()
{
    return &ex;
}