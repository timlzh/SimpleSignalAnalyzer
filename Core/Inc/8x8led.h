#ifndef __8x8LED_H
#define __8x8LED_H
#include "stm32f1xx.h"
#define PIN_CLK	GPIO_PIN_5
#define PIN_CS  GPIO_PIN_4
#define PIN_DIN GPIO_PIN_7
#define GPIOx GPIOA

void Write_Byte(uint8_t DATA);
void Write_Max7219(uint8_t addr,uint8_t dat);
void Init_Max7219(void);
#endif
