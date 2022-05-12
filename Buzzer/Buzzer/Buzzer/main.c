#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

int main(void)
{
	DDRC = 0b00000110;
	while (1)
	{
		if((PINC&0b00000001) == 0){
			PORTC = 0b00000100;
			_delay_ms(1000);
			PORTC = 0b00000000;
			_delay_ms(1000);
		}
		else
		PORTC = 0b00000010;
		
	}
	_delay_ms(20);
}
