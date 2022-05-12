
#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

#define LCD_Dir  DDRA			/* Define LCD data port direction */
#define LCD_Port PORTA			/* Define LCD data port */
#define RS PA2				/* Define Register Select pin */
#define EN PA3 				/* Define Enable signal pin */

#include <stdlib.h>
#include <stdio.h>
#define DHT11_PIN 6
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;


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

void Request()				/* Microcontroller send start pulse/request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTD |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

int main()
{
	char data[5];
	lcdInit();			/* Initialization of LCD*/
	int v = 0;
	lcdWrite("Humidity: ");	/* Write string on 1st line of LCD*/
	lcdcmd(0xC0);		/* Go to 2nd line*/
	lcdWrite("Temp: ");	/* Write string on 2nd line*/

	
	while(1){
		Request();		/* send start pulse */
		Response();		/* receive response */
		I_RH=Receive_data();	/* store first eight bit in I_RH */
		D_RH=Receive_data();	/* store next eight bit in D_RH */
		I_Temp=Receive_data();	/* store next eight bit in I_Temp */
		D_Temp=Receive_data();	/* store next eight bit in D_Temp */
		CheckSum=Receive_data();/* store next eight bit in CheckSum */
		
		if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
		{
			lcdClear();
			lcdWrite("Error");
		}
		
		
		
		
		
		else if(I_Temp>40){
			lcdClear();
			lcdWrite("High Humidity");
			//_delay_us(1000);
			v =1;
		}
		else if(D_Temp>40){
			lcdClear();
			lcdWrite("High Temperture");
			//_delay_us(1000);
			v = 1;
		}
		else
		{
			if(v==1){
				lcdClear();			/* Initialization of LCD*/
				lcdWrite("Humidity: ");	/* Write string on 1st line of LCD*/
				lcdcmd(0xC0);		/* Go to 2nd line*/
				lcdWrite("Temp: ");
				v=0;
			}
			itoa(I_RH,data,10);
			lcdcmd(0x8A);
			lcdWrite(data);
			lcdWrite(".");
			
			itoa(D_RH,data,10);
			lcdWrite(data);
			lcdWrite("%");
			
			
			itoa(I_Temp,data,10);
			lcdcmd(0xC6);
			lcdWrite(data);
			lcdWrite(".");
			
			itoa(D_Temp,data,10);
			lcdWrite(data);
			lcddata(0xDF);
			lcdWrite("C ");
			
			itoa(CheckSum,data,10);
			lcdWrite(data);
			lcdWrite(" ");
		}
		_delay_ms(10);
	}
}