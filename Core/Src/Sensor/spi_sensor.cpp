#include "Sensor/spi_sensor.h"

Spi_Sensor::Spi_Sensor(SPI_HandleTypeDef *spi_h, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    spi_handler = spi_h;
    CS_port = cs_port;
    CS_pin = cs_pin;
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

void Spi_Sensor::write_register(uint8_t register_addr, uint8_t value)
{
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handler, (uint8_t *)&register_addr, 1, 1000);
    HAL_SPI_Transmit(spi_handler, (uint8_t *)&value, 1, 1000);
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

uint8_t Spi_Sensor::read_register(uint8_t register_addr)
{
    register_addr |= 0x80;
    uint8_t buffer;
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handler, (uint8_t *)&register_addr, 1, 1000);
    HAL_SPI_Receive(spi_handler, (uint8_t *)&buffer, 1, 1000);
    HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
    return buffer;
}