
#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

#define LCD_Dir  DDRA			/* Define LCD data port direction */
#define LCD_Port PORTA			/* Define LCD data port */
#define RS PA2				/* Define Register Select pin */
#define EN PA3 				/* Define Enable signal pin */

#include <stdlib.h>
#include "adc.h"
#include "mq5.h"

float Ro=10;                 //Ro is initialized to 10 kilo ohms

#define RL_VALUE (10)     //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR (6.5)  //(Sensor resistance in clean air)/RO,which is derived from the chart in datasheet
#define LPG (0)         // Gas identity no.
#define CH4 (1)

//data formula obtained for the MQ5 sensor for measuring different gases.
float Values1[5] = {-2.5279,-2.5474,-4.2302,-4.5008,-7.358};
float Values2[5] = {1.8771,2.2636,3.0935,4.8216,6.4758};



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
		_delay_ms(50);//change me to 500ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
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
	if ( gas_id == LPG ) {
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
	

	
	unsigned char Res[16], lpg[16], ch4[16];
	lcdInit();
	adcinit();
	lcdWrite("Calibrating...");
	Ro = SensorCalibration();                       //Please make sure the sensor is in clean air when you perform the calibration
	dtostrf(Ro, 6, 2, Res);
	lcdClear();
	_delay_ms(2);	//clearing takes around 1.64ms to execute
	lcdWrite("Calibration done...");
	_delay_ms(2000);
	lcdClear();
	_delay_ms(2);	//clearing takes around 1.64ms to execute
	//lcdcmd(0xC0);
	lcdWrite("LPG:");
	lcdcmd(0x8D);
	lcdWrite("PPM");
	lcdcmd(0xC0);
	lcdWrite("CH4:");
	lcdcmd(0xCD);
	lcdWrite("PPM");
	
	while(1){

		
		itoa(GetGasPercentage(ReadSensor()/Ro,LPG), lpg, 10);
		lcdcmd(0x89);
		lcdWrite(lpg);
		itoa(GetGasPercentage(ReadSensor()/Ro,CH4), ch4, 10);
		lcdcmd(0xC9);
		lcdWrite(ch4);
		_delay_ms(2000);

	}
	_delay_ms(10);
	
}