#ifndef MPU9250_H
#define MPU9250_H

#include "spi_sensor.h"
#include "Datatype/dynamics.h"
#include "Sensor/Adafruit_AHRS_Madgwick.h"

class Mpu9250 : Spi_Sensor
{
private:
	Adafruit_Madgwick filter;
	// Earth frame to AUV frame
	Quaternion q_EtoA;
	// Sensor frame to Earth frame
	Quaternion q_ItoE;
	Quaternion q_filter;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;

	int16_t read_value(uint8_t type);

public:
	Mpu9250(SPI_HandleTypeDef *spi_h, GPIO_TypeDef *cs_port, uint16_t cs_pin);
	Dynamics update(Dynamics s);
};

#endif