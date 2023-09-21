#include "Sensor/mpu9250.h"
#include "Sensor/mpu9250_register_map.h"

Mpu9250::Mpu9250(/* args */): q_EtoA(0, 0.707, 0.707, 0), q_ItoE(0.707, -0.707, 0, 0) /*q_ItoE(0, -0.707, -0.707, 0)*/
{

}

Mpu9250::~Mpu9250()
{
}

void Mpu9250::set(SPI_HandleTypeDef* spi_h, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
    Spi_Sensor::set(spi_h, cs_port, cs_pin);
    if (read_register(WHO_AM_I_MPU9250) != 0x71)
        return;
    write_register(PWR_MGMT_1, 01);
	write_register(USER_CTRL, 0x20);
	write_register(PWR_MGMT_2, 0x00);
	write_register(ACCEL_CONFIG, 0x00);
	write_register(GYRO_CONFIG, 0x18);
	write_register(ACCEL_CONFIG2, 0x01);
	write_register(CONFIG, 0x01);
	write_register(SMPLRT_DIV, 0x00);
	
	filter.begin(20);
	filter.setBeta(0.3);

	//Let the IMU value converge
	Dynamics temp;
	for (int i = 0; i < 5000; i++)
		update(temp);
}

int16_t Mpu9250::read_value(uint8_t type)
{
	int16_t h = read_register(type);
	int16_t l = read_register(type);
	return ((h<<8 | l));
}

void Mpu9250::update(Dynamics &s)
{
	//read acceleration and gyro
	ax = (float)(read_value(ACCEL_XOUT_H) * 599) / (float)1000000;
	ay = (float)(read_value(ACCEL_YOUT_H) * 599) / (float)1000000;
	az = (float)(read_value(ACCEL_ZOUT_H) * 599) / (float)1000000;
	gx = (float)(read_value(GYRO_XOUT_H) * 133) / (float)1000000;
	gy = (float)(read_value(GYRO_YOUT_H) * 133) / (float)1000000;
	gz = (float)(read_value(GYRO_ZOUT_H) * 133) / (float)1000000;

	ax = (int)(ax * 100) / 100.0;
	ay = (int)(ay * 100) / 100.0;
	az = (int)(az * 100) / 100.0;
	gx = (int)(gx * 100) / 100.0;
	gy = (int)(gy * 100) / 100.0;
	gz = (int)(gz * 100) / 100.0;

	filter.updateIMU(gx, gy, gz, ax, ay, az);
	filter.getQuaternion(&q_filter.w,&q_filter.x, &q_filter.y, &q_filter.z);
	
	//Tansfer quaternion from Earth frame to AUV frame
	s.orientation = q_EtoA.conjugate() * (q_filter * q_ItoE) * q_EtoA;

	//s.orientation =  q_ItoE * q_filter; //* q_ItoE.conjugate();

	s.velocity.angular.x = gx;
	s.velocity.angular.y = gy;
	s.velocity.angular.z = gz;
	//test[0] = filter.getRoll();
	//test[1] = filter.getPitch();

	//test[0] = atan2f(s.orientation.w * s.orientation.x + s.orientation.y * s.orientation.z, 0.5f - s.orientation.x * s.orientation.x - s.orientation.y * s.orientation.y) * 57.29578f;
	//test[1] = asinf(-2.0f * (s.orientation.x * s.orientation.z - s.orientation.w * s.orientation.y)) * 57.29578f;
	//test[2] = filter.getYaw();
}









// void Mpu9250::update(Dynamics &s)
// {
// 	float F_g[3] = {0};
// 	float J_g[3][4] = {0};

// 	//read acceleration and gyro
// 	q_a.w = 0;
// 	q_a.x = (float)(read_value(ACCEL_XOUT_H) * 599) / (float)1000000;
// 	q_a.y = (float)(read_value(ACCEL_YOUT_H) * 599) / (float)1000000;
// 	q_a.z = (float)(read_value(ACCEL_ZOUT_H) * 599) / (float)1000000;
// 	q_a.normalize();

// 	q_w.w = 0;
// 	q_w.x = (float)(read_value(GYRO_XOUT_H) * 133) / (float)1000000;
// 	q_w.y = (float)(read_value(GYRO_YOUT_H) * 133) / (float)1000000;
// 	q_w.z = (float)(read_value(GYRO_ZOUT_H) * 133) / (float)1000000;

// 	q_w *= 0.5;
// 	q_w = q_est_prev * q_w;

// 	for ( int i = 0; i < 1000; i++)
// 	{
// 		q_est_prev = q_est;
// 		gradient.reset();

// 		F_g[0] = 2 * (q_est_prev.x * q_est_prev.z - q_est_prev.w * q_est_prev.y) - q_a.x;
// 		F_g[1] = 2 * (q_est_prev.w * q_est_prev.x + q_est_prev.y * q_est_prev.z) - q_a.y;
// 		F_g[2] = 2 * (0.5 - q_est_prev.x * q_est_prev.x - q_est_prev.y * q_est_prev.y) - q_a.z;

// 		J_g[0][0] = -2 * q_est_prev.y;
// 		J_g[0][1] = 2 * q_est_prev.z;
// 		J_g[0][2] = -2 * q_est_prev.w;
// 		J_g[0][3] = 2 * q_est_prev.x;

// 		J_g[1][0] = 2 * q_est_prev.x;
// 		J_g[1][1] = 2 * q_est_prev.w;
// 		J_g[1][2] = 2 * q_est_prev.z;
// 		J_g[1][3] = 2 * q_est_prev.y;

// 		J_g[2][0] = 0;
// 		J_g[2][1] = -4 * q_est_prev.x;
// 		J_g[2][2] = -4 * q_est_prev.y;
// 		J_g[2][3] = 0;

// 		gradient.w = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
// 		gradient.x = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
// 		gradient.y = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
// 		gradient.z = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];
// 		gradient.normalize();
// 		gradient *= BETA;
// 		q_est = q_est_prev + (q_w - gradient) * 0.000001f;
// 	}
// 	s.orientation = q_est;
// 	q_est.reset(1, 0, 0, 0);
// }