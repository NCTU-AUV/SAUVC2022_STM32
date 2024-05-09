#ifndef SPI_SENSOR_H
#define SPI_SENSOR_H

#include "stm32f4xx.h"

class Spi_Sensor
{
private:
    SPI_HandleTypeDef *spi_handler;
    GPIO_TypeDef *CS_port;
    uint16_t CS_pin;

protected:
    Spi_Sensor(SPI_HandleTypeDef *spi_h, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    void write_register(uint8_t register_addr, uint8_t value);
    uint8_t read_register(uint8_t register_addr);
};

#endif