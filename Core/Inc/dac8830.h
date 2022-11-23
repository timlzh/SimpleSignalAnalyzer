#ifndef __DAC8830_H
#define __DAC8830_H
#include "stm32f1xx.h"
#define GPIOx GPIOA
#define DAC8830_PIN_CS GPIO_PIN_6
#define VREF 3.3

void dac8830_write_Byte(uint8_t DATA);
void dac8830_write_volt(float volt);
#endif
