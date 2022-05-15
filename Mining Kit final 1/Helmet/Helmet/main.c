
#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

#define LCD_Dir  DDRD			/* Define LCD data port direction */
#define LCD_Port PORTD			/* Define LCD data port */
#define RS PA2				/* Define Register Select pin */
#define EN PA3 				/* Define Enable signal pin */

#include <stdlib.h>




void helmetRemoved(){
	PORTB = 0b10000000;
}
void helmetCollisoin(){
	PORTC = 0b00001000;
	_delay_ms(1000);
	PORTC = 0x00;
}

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

void lcdClear(){
	lcdcmd (0x01);		/* Clear display */
	_delay_ms(2);
	lcdcmd (0x80);		/* Cursor at home position */
}

void adcinit(){
	//make PA0 an analog input
	DDRA &= ~(1<<PA0);
	//enable ADC module, set prescalar of 128 which gives CLK/128
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	//set the voltage reference using REFS1 and REFS0 bits and select the ADC channel using the MUX bits
	ADMUX = 0b01000000;      // set REFS1 = 0 |REFS0 = 1 (Vref as AVCC pin) | ADLAR = 0(right adjusted) |  MUX4 to MUX0 is 0000 for ADC0
}

int adcread(char channel){
	/* set input channel to read */
	ADMUX = 0x40 | (channel & 0x07);   // 0100 0000 | (channel & 0000 0100)
	/* Start ADC conversion */
	ADCSRA |= (1<<ADSC);
	/* Wait until end of conversion by polling ADC interrupt flag */
	while (!(ADCSRA & (1<<ADIF)));
	/* Clear interrupt flag */
	ADCSRA |= (1<<ADIF);
	_delay_ms(1);                      /* Wait a little bit */
	/* Return ADC word */
	return ADCW;
}

int main()
{
	
	DDRA = 0x00;
	DDRC = 0xFF;
	DDRD = 0xFF;
	
	lcdInit();
	adcinit();
	lcdClear();
	char snum[5];

	while(1){

		
		itoa( adcread(0), snum, 10);
		lcdcmd(0x89);
		lcdWrite(snum);

		_delay_ms(200);
		
		{
			if(!((PINA & 0b10000000) == 0b10000000))
			helmetRemoved();
			else
			PORTB = 0b00000000;
			
			if (adcread(0)>300)
			helmetCollisoin();
			
			
		}

	}
	_delay_ms(10);
	
}