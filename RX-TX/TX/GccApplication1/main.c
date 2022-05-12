#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

#define LCD_Dir  DDRC			/* Define LCD data port direction */
#define LCD_Port PORTC			/* Define LCD data port */
#define RS PC2				/* Define Register Select pin */
#define EN PC3 				/* Define Enable signal pin */


void lcdcmd( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);		/* RS=0, command reg. */
	LCD_Port |= (1<<EN);		/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_us(2);
	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}


void lcddata( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Port |= (1<<RS);		/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_us(2);
	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void lcdInit (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;			/* Make LCD port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	lcdcmd(0x02);		/* send for 4 bit initialization of LCD  */
	lcdcmd(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	lcdcmd(0x0c);              /* Display on cursor off*/
	lcdcmd(0x06);              /* Increment cursor (shift cursor to right)*/
	lcdcmd(0x01);              /* Clear display screen*/
	_delay_ms(2);
}


void lcdWrite (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		lcddata (str[i]);
	}
}


void LCD_Clear()
{
	lcdcmd (0x01);		/* Clear display */
	_delay_ms(2);
	lcdcmd (0x80);		/* Cursor at home position */
}

int main()
{
	DDRD = 0b11111111;
	
	//lcdInit();			

	//lcdWrite("AVR display");	
	//lcdcmd(0xC0);		 
	//lcdWrite("Hello World");
	
	

	while (1)
	{
		PORTD = 0b00010000;
		_delay_ms(200);
		PORTD = 0b00100000;
		_delay_ms(200);
		PORTD = 0b01000000;
		_delay_ms(200);
		PORTD = 0b10000000;
		_delay_ms(1000);
		//PORTD = 0xFF;
		/*
		if(PINB.0 == 1)  {
			PORTD = 0x10;
		}
		if(PINB.0 == 0)  {
			PORTD = 0x20;
		}
		*/
	}
	return 0;
}

