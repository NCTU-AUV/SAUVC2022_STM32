#ifndef MPU9250_H
#define MPU9250_H

#include "spi_sensor.h"
#include "Datatype/dynamics.h"
#include "Sensor/Adafruit_AHRS_Madgwick.h"

class Mpu9250:Spi_Sensor
{
private:
	Adafruit_Madgwick filter;
	Quaternion q_EtoA;  //Earth frame to AUV frame
	Quaternion q_ItoE;  //Sensor frame to Earth frame
	Quaternion q_filter;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;

public:
    Mpu9250(/* args */);
    ~Mpu9250();
    void set(SPI_HandleTypeDef* spi_h, GPIO_TypeDef* cs_port, uint16_t cs_pin);
    void set_memory_dress(uint8_t num, uint8_t address);
	int16_t read_value(uint8_t type);
	void update(Dynamics &s);

	float test[3];
};

#endif