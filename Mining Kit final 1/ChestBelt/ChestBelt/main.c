/*
 * ChestBelt.c
 *
 * Created: 5/12/2022 3:18:46 AM
 * Author : darklane
 */ 
#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#define LCD1_Dir  DDRC			/* Define LCD data port direction */
#define LCD1_Port PORTC			/* Define LCD data port */
#define RS 2				/* Define Register Select pin */
#define EN 3 				/* Define Enable signal pin */
#define LCD2_Dir  DDRD			/* Define LCD data port direction */
#define LCD2_Port PORTD			/* Define LCD data port */

#include <stdlib.h>
#include <stdio.h>
#define DHT11_PIN 5
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;


void lcdcmd(int lcd, unsigned char cmnd )
{
	if(lcd==1)
		{
		LCD1_Port = (LCD1_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
		LCD1_Port &= ~ (1<<RS);		/* RS=0, command reg. */
		LCD1_Port |= (1<<EN);		/* Enable pulse */
		_delay_us(1);
		LCD1_Port &= ~ (1<<EN);
		_delay_us(2);
		LCD1_Port = (LCD1_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
		LCD1_Port |= (1<<EN);
		_delay_us(1);
		LCD1_Port &= ~ (1<<EN);
		_delay_ms(2);
		}
		else{
			LCD2_Port = (LCD2_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
			LCD2_Port &= ~ (1<<RS);		/* RS=0, command reg. */
			LCD2_Port |= (1<<EN);		/* Enable pulse */
			_delay_us(1);
			LCD2_Port &= ~ (1<<EN);
			_delay_us(2);
			LCD2_Port = (LCD2_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
			LCD2_Port |= (1<<EN);
			_delay_us(1);
			LCD2_Port &= ~ (1<<EN);
			_delay_ms(2);
			
		}
}


void lcddata(int lcd, unsigned char data )
{
	if(lcd==1)
		{
		LCD1_Port = (LCD1_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
		LCD1_Port |= (1<<RS);		/* RS=1, data reg. */
		LCD1_Port|= (1<<EN);
		_delay_us(1);
		LCD1_Port &= ~ (1<<EN);
		_delay_us(2);
		LCD1_Port = (LCD1_Port & 0x0F) | (data << 4); /* sending lower nibble */
		LCD1_Port |= (1<<EN);
		_delay_us(1);
		LCD1_Port &= ~ (1<<EN);
		_delay_ms(2);
		}
		else{
			LCD2_Port = (LCD2_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
			LCD2_Port |= (1<<RS);		/* RS=1, data reg. */
			LCD2_Port|= (1<<EN);
			_delay_us(1);
			LCD2_Port &= ~ (1<<EN);
			_delay_us(2);
			LCD2_Port = (LCD2_Port & 0x0F) | (data << 4); /* sending lower nibble */
			LCD2_Port |= (1<<EN);
			_delay_us(1);
			LCD2_Port &= ~ (1<<EN);
			_delay_ms(2);
		}
}

void lcdInit (void)			/* LCD Initialize function */
{
	LCD1_Dir = 0xFF;			/* Make LCD port direction as o/p */
	LCD2_Dir = 0xFF;			/* Make LCD port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	lcdcmd(1,0x02);		/* send for 4 bit initialization of LCD  */
	lcdcmd(1,0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	lcdcmd(1,0x0c);              /* Display on cursor off*/
	lcdcmd(1,0x06);              /* Increment cursor (shift cursor to right)*/
	lcdcmd(1,0x01);              /* Clear display screen*/
	_delay_ms(2);
	
	lcdcmd(2,0x02);		/* send for 4 bit initialization of LCD  */
	lcdcmd(2,0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	lcdcmd(2,0x0c);              /* Display on cursor off*/
	lcdcmd(2,0x06);              /* Increment cursor (shift cursor to right)*/
	lcdcmd(2,0x01);              /* Clear display screen*/
	_delay_ms(2);
		
}


void lcdWrite (int lcd, char *str)		/* Send string to LCD function */
{
	if(lcd==1)
	{
		int i;
		for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
		{
			lcddata (1,str[i]);
		}
	}
	else{
		int j;
		for(j=0;str[j]!=0;j++)		/* Send each char of string till the NULL */
		{
			lcddata (2,str[j]);
		}
	}
}


void lcdClear(int lcd)
{
	if(lcd==1)
	{
		lcdcmd (1,0x01);		/* Clear display */
		_delay_ms(2);
		lcdcmd (1,0x80);		/* Cursor at home position */
	}
	else{
		lcdcmd (2,0x01);		/* Clear display */
		_delay_ms(2);
		lcdcmd (2,0x80);		/* Cursor at home position */
	}
}

void Request()				/* Microcontroller send start pulse/request */
{
	DDRA |= (1<<DHT11_PIN);
	PORTA &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTA |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRA &= ~(1<<DHT11_PIN);
	while(PINA & (1<<DHT11_PIN));
	while((PINA & (1<<DHT11_PIN))==0);
	while(PINA & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PINA & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PINA & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PINA & (1<<DHT11_PIN));
	}
	return c;
}


int main()
{
	DDRB = 0b11110000;
	DDRA = 0b01000000;
	DDRD = 0xFF;
	
	char data[5];
	lcdInit();			/* Initialization of LCD*/
	int v = 0;
	lcdWrite(1,"Humidity: ");	/* Write string on 1st line of LCD*/
	lcdcmd(1,0xC0);		/* Go to 2nd line*/
	lcdWrite(1,"Temp: ");	/* Write string on 2nd line*/
	
	lcdWrite(2,"Humidity: ");	/* Write string on 1st line of LCD*/
	lcdcmd(2,0xC0);		/* Go to 2nd line*/
	lcdWrite(2,"Temp: ");	/* Write string on 2nd line*/
	while(1){
		
		
		//Panic Button
		if ((PINA & 0b10000000)== 0b10000000)
		{
			PORTB |= 0b01000000;
		} 
		else
		{
			PORTB &= ~(0b01000000);
		}
		
		//Piezo sensor
		if ((PINA & 0b00001000)== 0b00001000)
		{
			PORTB |= 0b10000000;
		}
		else
		{
			PORTB &= ~(0b10000000);
		}
		
		
		Request();		/* send start pulse */
		Response();		/* receive response */
		I_RH=Receive_data();	/* store first eight bit in I_RH */
		D_RH=Receive_data();	/* store next eight bit in D_RH */
		I_Temp=Receive_data();	/* store next eight bit in I_Temp */
		D_Temp=Receive_data();	/* store next eight bit in D_Temp */
		CheckSum=Receive_data();/* store next eight bit in CheckSum */
		
		if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
		{
			lcdClear(1);
			lcdWrite(1,"Error");
		}
		
		else if(D_Temp>40){
			lcdClear(1);
			lcdWrite(1,"High Humidity");
			//_delay_us(1000);
			v =1;
		}
		else if(I_Temp>40){
			lcdClear(1);
			lcdWrite(1,"High Temperature");
			PORTB |= 0b00010000;
			PORTA |= 0b01000000;
			//_delay_us(1000);
			v = 1;
		}
		else
		{
			PORTB &= ~(0b00010000);
			PORTA &= ~(0b01000000);
			if(v==1){
				lcdClear(1);			/* Initialization of LCD*/
				lcdWrite(1,"Humidity: ");	/* Write string on 1st line of LCD*/
				lcdcmd(1,0xC0);		/* Go to 2nd line*/
				lcdWrite(1,"Temp: ");
				v=0;
			}
			itoa(I_RH,data,10);
			lcdcmd(1,0x8A);
			lcdWrite(1,data);
			lcdWrite(1,".");
			
			itoa(D_RH,data,10);
			lcdWrite(1,data);
			lcdWrite(1,"%");
			
			
			itoa(I_Temp,data,10);
			lcdcmd(1,0xC6);
			lcdWrite(1,data);
			lcdWrite(1,".");
			
			itoa(D_Temp,data,10);
			lcdWrite(1,data);
			lcddata(1,0xDF);
			lcdWrite(1,"C ");
			
			itoa(CheckSum,data,10);
			lcdWrite(1,data);
			lcdWrite(1," ");
		}
		_delay_ms(10);
	}
}