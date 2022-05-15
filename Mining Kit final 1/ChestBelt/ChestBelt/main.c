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

float Ro=10;    
#include "adc.h"
#include "mq5.h"
#define RL_VALUE (10)     //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR (6.5)  //(Sensor resistance in clean air)/RO,which is derived from the chart in data sheet
#define CO (0)         // Gas identity no.
#define CH4 (1)

//data formula obtained for the MQ5 sensor for measuring different gases.
float Values1[5] = {-2.5279,-2.5474,-4.2302,-4.5008,-7.358};
float Values2[5] = {1.8771,2.2636,3.0935,4.8216,6.4758};


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


// sensor and load resistor forms a voltage divider. so using analog value and load value
// we will find sensor resistor.

float ResistanceCalculation(int raw_adc){
	return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));   // we will find sensor resistor.
}

float SensorCalibration(){

	int i;                                   // This function assumes that sensor is in clean air.
	float val=0;
	
	for (i=0;i<50;i++){                   //take multiple samples and calculate the average value
		val += ResistanceCalculation(adcread(0));
		_delay_ms(50);//change me to 500
	}

	val = val/50;
	val = val/RO_CLEAN_AIR_FACTOR;           //divided by RO_CLEAN_AIR_FACTOR yields the Ro according to the chart in the datasheet
	
	return val;
}

float ReadSensor(){
	int i;
	float rs=0;

	for (i=0;i<5;i++) {                                 // take multiple readings and average it.
		rs += ResistanceCalculation(adcread(0));   // rs changes according to gas concentration.
		_delay_ms(50);
	}

	rs = rs/5;
	
	return rs;
}

int GetGasPercentage(float rs_ro_ratio, int gas_id){
	if ( gas_id == CO ) {
		return GetPercentagee(rs_ro_ratio,Values1[0],Values2[0]);
	}
	else if( gas_id == CH4 ) {
		return GetPercentagee(rs_ro_ratio,Values1[1],Values2[1]);
	}

	return 0;
}


// as in curves are on logarithmic coordinate, power of 10 is taken to convert result to non-logarithmic.
int  GetPercentagee(float rs_ro_ratio, float val1,float val2){
	return (pow(10,( (log(rs_ro_ratio)*val1) + val2)));
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
	DDRB = 0b11110000;
	DDRA = 0b01000000;
	DDRD = 0xFF;
	
	char Res[16], co[16], ch4[16];
	lcdInit();/* Initialization of LCD*/
	adcinit();
	lcdWrite(2,"Calibrating...");
	Ro = SensorCalibration();                       //Please make sure the sensor is in clean air when you perform the calibration
	dtostrf(Ro, 6, 2, Res);
	
	lcdClear(2);
	_delay_ms(2);	//clearing takes around 1.64ms to execute
	lcdWrite(2,"Calibration done...");
	_delay_ms(2000);
	lcdClear(2);
	_delay_ms(2);	//clearing takes around 1.64ms to execute
	lcdWrite(2,"CO:");
	lcdcmd(2,0x8D);
	lcdWrite(2,"PPM");
	lcdcmd(2,0xC0);
	lcdWrite(2,"CH4:");
	lcdcmd(2,0xCD);
	lcdWrite(2,"PPM");
	
	
	
	char data[5];
	int v = 0;
	lcdWrite(1,"Humidity: ");	/* Write string on 1st line of LCD*/
	lcdcmd(1,0xC0);		/* Go to 2nd line*/
	lcdWrite(1,"Temp: ");	/* Write string on 2nd line*/
	
	
	while(1){
		
		itoa(GetGasPercentage(ReadSensor()/Ro,CO), co, 10);
		if (atoi(co)>10)
		{
			lcdClear(2);
			lcdWrite(1,"High Co Values");
			PORTB |= 0b00100000;
		} 
		else
		{
			PORTB &= ~(0b00100000);
			itoa(GetGasPercentage(ReadSensor()/Ro,CO), co, 10);
			lcdcmd(2,0x89);
			lcdWrite(2,co);
			itoa(GetGasPercentage(ReadSensor()/Ro,CH4), ch4, 10);
			lcdcmd(2,0xC9);
			lcdWrite(2,ch4);
		}
		
		
		
		
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
	
			
		}
		_delay_ms(10);
	}
}