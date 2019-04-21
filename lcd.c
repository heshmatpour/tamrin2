#include "lcd.h"
#include "stm32f4xx_hal.h"

#define D0_PIN_Start  1
#define  LCD_DATA_MASK   (LCD_D7|LCD_D6|LCD_D5|LCD_D4)

static GPIO_TypeDef* PORT_LCD;	
static uint8_t state;		    	
static uint16_t D[8];	
static GPIO_TypeDef* port_EN;
static GPIO_TypeDef* port_RS;
static uint16_t PIN_RS, PIN_EN;

void lcd_init(lcd_t *lcd)
{
	switch(mode)
	{
	 case 1:
	for(uint8_t i=0 ;i<8 ;i=i+1 )
	{
		D[i]= lcd.data_pins[i];
	}
	break;
	 default:
	for(uint8_t i=0 ;i<4 ;i=i+1 )
	{
		D[i]= lcd.data_pins[i];
	}
	break;}
	PIN_EN=lcd.en_pin;
	PIN_RS=lcd.rs_pin;
	state=mode;	
}
void lcd_putchar(lcd_t *lcd, uint8_t character)
{
  unsigned char temp=0;
  unsigned int temp1=0;
  temp=character;
  temp=(temp>>4)&0x0F;
  temp1=(temp<<20)&LCD_DATA_MASK;
  delay(10);
}
void lcd_puts(lcd_t *lcd,char *str)
{
  HAL_Delay(T);
  while(*str != 0)
  {
    lcd_putchar(lcd , *str);
    str++;
  }
}
void lcd_clear (lcd_t *lcd)
{ 
HAL_Delay(T);
HAL_GPIO_WritePin(GPIOB,(1<<rs_pin),GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOD,(0xFF<<D0_PIN_Start),GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOD,(0x01<<D0_PIN_Start),GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,(1<<en_pin),GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,(1<<en_pin),GPIO_PIN_RESET);
}
void lcd_set_curser(lcd_t *lcd, uint16_t row, uint16_t col)
{
uint16_t maskData;
maskData = (col-1)&0x0F;
	 if(row==1)
	              {
		maskData |= (0x80);
        	 write(lcd ,maskData);
	               }
		 	else
	               {
		maskData |= (0xc0);
		write(lcd ,maskData);
			}
}
