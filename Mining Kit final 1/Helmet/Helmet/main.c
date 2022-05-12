/*
 * Helmet.c
 *
 * Created: 5/11/2022 8:48:38 PM
 * Author : darklane
 */
 #define F_CPU 8000000UL			/* Define CPU Frequency e.g. here 8MHz */
 #include <avr/io.h>			/* Include AVR std. library file */
 #include <util/delay.h>			/* Include Delay header file */

 void helmetRemoved(){
 PORTB = 0b10000000;
 }
 void helmetCollisoin(){
 PORTC = 0b00001000;
 _delay_ms(500);
 PORTC = 0x00;
 }
 int main(void)
 {
 DDRA = 0x00;
 DDRC = 0xFF;
 DDRD = 0xFF;
 while (1)
 {
 if(!((PINA & 0b10000000) == 0b10000000))
 helmetRemoved();
 else
 PORTB = 0b00000000;
 
 if ((PINA & 0b01000000) == 0b01000000)
 {
 helmetCollisoin();
 }
 
 }
 _delay_ms(500);
 }
