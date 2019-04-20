#include "lcd.h"
#include "stm32f4xx_hal.h"

static GPIO_TypeDef* PORT_LCD;	
static uint8_t state;		    	
static uint16_t D[8];	
static GPIO_TypeDef* port_EN;
static GPIO_TypeDef* port_RS;
static uint16_t PIN_RS, PIN_EN;

void lcd_init(lcd_t * lcd)
{
	for(i=0 ;i<8 ;i=i+1 )
	{
		D[i]= lcd_t.data_pins[i];
	}
	PIN_EN=lcd_t.en_port;
	PIN_RS=lcd_t.rs_port;
	state=mode;	
}
