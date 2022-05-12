/*
 * ReceiverStation.c
 *
 * Created: 5/12/2022 8:10:12 PM
 * Author : darklane
 */ 
#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>
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


void lcdClear()
{
	lcdcmd (0x01);		/* Clear display */
	_delay_ms(2);
	lcdcmd (0x80);		/* Cursor at home position */
}
int main(void)
{
	DDRD = 0b11000000;
	lcdInit();			/* Initialization of LCD*/
	lcdWrite("Normal ");	/* Write string on 1st line of LCD*/
    while (1) 
    {
		DDRB = 0b00000000;
		if (PINB==0b00000001)
		{
			PORTD = 0b01000000;
			lcdClear();
			lcdWrite("HIGH TEMPERTURE!");
		}
		else if (PINB==0b00000010)
		{
			PORTD = 0b01000000;
			lcdClear();
			lcdWrite("HIGH CO Value!");
		}
		else if (PINB==0b00000011)
		{
			PORTD = 0b01000000;
			lcdClear();
			lcdWrite("HIGH TEMPERTURE!");
			lcdcmd (0xC0);
			lcdWrite("HIGH CO Value!");
		}
		else if (PINB & 0b00000100)
		{
			PORTD = 0b01000000;
			lcdClear();
			lcdWrite("DANGER !!!");
			if(PINB & 0b00000010){
				lcdcmd (0xC0);
				lcdWrite("High: ");
				lcdcmd (0xC6);
				lcdWrite("CO |");
			}
			if(PINB & 0b00000001){
				lcdcmd (0xC0);
				lcdWrite("High: ");
				lcdcmd (0xC9);
				lcdWrite("| Temp");
			}
		}
		else if (PINB==0b00001000)
		{
			PORTD = 0b01000000;
			lcdClear();
			lcdWrite("Head Knocked");
		}
		else{
				PORTD = 0b10000000;
				lcdClear();
				lcdWrite("Normal ");
				
		}
		_delay_ms(100);

    }
}

