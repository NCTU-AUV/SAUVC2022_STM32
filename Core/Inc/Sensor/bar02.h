#ifndef BAR02_H
#define BAR02_H

#include "stm32f4xx.h"

class Bar02{
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;
	static const uint8_t MS5837_UNRECOGNISED;

    Bar02();

	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
	 * and MS5837::MS5837_02BA.
	 */
	bool set(I2C_HandleTypeDef*);
	void setModel(uint8_t model);
	uint8_t getModel();
	void setFluidDensity(float density);
    void read_value();

    float pressure(float conversion = 1.0f);
    float temperature();
    float depth();
    float altitude();

private:
    I2C_HandleTypeDef* h;
    uint16_t C[8];
    uint32_t D1_pres, D2_temp;
    int32_t TEMP;
    int32_t P;
    uint8_t _model;
    float depth_offset;

    float fluidDensity;

    void calculate();
    uint8_t crc4(uint16_t n_prom[]);
};

#endif