#include "8x8led.h"
#include "spi.h"


void Write_Byte(uint8_t DATA)
{
	HAL_GPIO_WritePin(GPIOx,PIN_CS,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&DATA,1,0xfff);
}

void Write_Max7219(uint8_t addr,uint8_t dat)
{
	Write_Byte(addr);
	Write_Byte(dat);
	HAL_GPIO_WritePin(GPIOx,PIN_CS,GPIO_PIN_SET);
}

void Init_Max7219(void)
{
	Write_Max7219(0x09, 0x00);
	Write_Max7219(0x0a, 0x03);
	Write_Max7219(0x0b, 0x07);
	Write_Max7219(0x0c, 0x01);
	Write_Max7219(0x0f, 0x00);
    for(uint8_t i=1;i<9;i++) Write_Max7219(i,0x00);
    
}

