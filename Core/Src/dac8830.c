#include "dac8830.h"

#include "spi.h"

void dac8830_write_Byte(uint8_t DATA) {
    HAL_GPIO_WritePin(GPIOx, DAC8830_PIN_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &DATA, 1, 0xfff);
}

void dac8830_write_volt(float volt) {
    int data;
    data = (volt / VREF * 65535);
    dac8830_write_Byte((data >> 8) & 0xff);
    dac8830_write_Byte(data & 0xff);
    HAL_GPIO_WritePin(GPIOx, DAC8830_PIN_CS, GPIO_PIN_SET);
}
