#define F_CPU 4000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */

void helmetRemoved(){
	PORTC = 0b00000100;
}
int main(void)
{
    /* Replace with your application code */
	//DDRB = 0b00000011;
	DDRC = 0b00000110;
    while (1) 
    {
		if((PINC & 0b00000001) == 0)
			helmetRemoved();
		else
			PORTC = 0b00000010;
			
    }
	_delay_ms(200);
}
