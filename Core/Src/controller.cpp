#include "controller.h"
#include "math.h"

void er_mul(float **a, float **b, float **result)
{
    float first[3][3] = {0};
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++)
                first[i][j] += a[k][i] * b[k][j];
        }

    float second[3][3] = {0};
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++)
                second[i][j] += a[i][k] * b[j][k];
        }
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++){
            result[i][j] = first[i][j] - second[i][j];
        }
    }
}

void matrix_mul(float **a, float **b, float **result)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++)
                result[i][j] += a[i][k] * b[k][j];
        }
}

void qtoR(const Quaternion &qd, float **m)
{
    float temp[3][3] = {{(1 - 2 * (qd.y * qd.y + qd.z * qd.z)), (2 * (qd.x * qd.y + qd.w * qd.z)), (2 * (qd.x * qd.z - qd.w * qd.y))},
                        {(2 * (qd.x * qd.y - qd.w * qd.z)), (1 - 2 * (qd.x * qd.x + qd.z * qd.z)), (2 * (qd.y * qd.z + qd.w * qd.x))},
                        {(2 * (qd.x * qd.z + qd.w * qd.y)), (2 * (qd.y * qd.z - qd.w * qd.x)), (1 - 2 * (qd.x * qd.x + qd.y * qd.y))}};
    
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m[i][j] = temp[i][j];
}

/**
 * @brief Constructor of Controller
 * @param x Kx
 * @param v Kv
 * @param R KR
 * @param Omega KOmega
 * @param alpha 0~1 how much we trust sonar yaw
 */ 
Controller::Controller(geometry::Vector x, geometry::Vector v, geometry::Vector kR, geometry::Vector Omega, float alpha): Kx(x), Kv(v), KR(kR), KOmega(Omega), Alpha_sonar(alpha)
{
    Rd = new float*[3];
    for(int i = 0; i < 3; ++i)
        Rd[i] = new float[3];

    R = new float*[3];
    for(int i = 0; i < 3; ++i)
        R[i] = new float[3];

    Re = new float*[3];
    for(int i = 0; i < 3; ++i)
        Re[i] = new float[3];
}

Controller::~Controller()
{
    delete Rd;
    delete R;
    delete Re;
}


void Controller::set(const Quaternion &qd)
{
    qtoR(qd, Rd);
}

// void Controller::adjust_yaw(Dynamics &s, float ys)
// {
//     Quaternion qy(cosf(ys / 2.0), 0, 0, sinf(ys / 2.0));
//     s.orientation = (1.0 - Alpha_sonar) * s.orientation + Alpha_sonar * qy;
// }

void Controller::update(Dynamics &s, const geometry::Vector &ex, const geometry::Vector &ev, float yaw_sonar, Kinematics &ctrl_input)
{
    //Calculate attitude error
    qtoR(s.orientation, R);
    er_mul(Rd, R, Re);
    eR.x = Re[2][1];
    eR.y = Re[0][2];
    eR.z = Re[1][0];    //(-1) * yaw_sonar; 
    // eR.z = 0.5;

    eOmega.x = s.velocity.angular.x;
    eOmega.y = s.velocity.angular.y;
    eOmega.z = s.velocity.angular.z;

    ctrl_input.linear.x = Kx.x * ex.x + Kv.x * ev.x;
    ctrl_input.linear.y = Kx.y * ex.y + Kv.y * ev.y;
    ctrl_input.linear.z = Kx.z * ex.z + Kv.z * ev.z - weight + buoyancy;
    ctrl_input.angular.x = KR.x * eR.x + KOmega.x * eOmega.x;
    ctrl_input.angular.y = KR.y * eR.y + KOmega.y * eOmega.y;
    if (eR.x < 0.001 && eR.y < 0.001)
        ctrl_input.angular.z = KR.z * eR.z + KOmega.z * eOmega.z;
}

geometry::Vector* Controller::get_ex(){
    return &ex;
}