#include <avr/io.h>
#include <util/delay.h>

void delay_func();
int main(void) {
	DDRB = 0xFF; // All port-B pins are configured as an output
	DDRD = 0b11111100; // Least two port-D pins are configured as an input
	PORTB = 0x01; // PINB0 pin is configured as a high
    int reading; // current button info
    int lastButtonState = 0x00;
    long int buttonMillisCounter = 0;
	while(1) {
            reading = ( PIND & _BV( PIND0 ));
            if(reading == 0x00 && lastButtonState == 0x00)
            {
                buttonMillisCounter++;
            }
            //_delay_ms(1);

            delay_func();
    	    if( (reading != lastButtonState) || buttonMillisCounter > 250){ // It runs when SW0 is changed
                    if (reading==0x00){
        	        if( ( PIND & _BV( PIND1 ) ) == 0x00 ){ // It runs when SW0 and SW1 are closed

                            if( PORTB == 0b10000000 ){
                                    PORTB = 0b10000000;
                                    PORTB = 0b10000000;
                                    PORTB = 0b10000000;
                                    PORTB = 0b00000001;
                                }
                            else{
                                PORTB = PORTB << 1;
                                }
                    }
        	        else{ // It runs when SW1 is open and SW0 is closed
           	            if( PORTB == 0b00000001 ){
                            PORTB = 0b00000001;
                            PORTB = 0b00000001;
                            PORTB = 0b00000001;
                            PORTB = 0b10000000;
                        }
                        else{
                	    PORTB = PORTB >> 1;  // 1.8 us
                        }
        	        }
                    buttonMillisCounter = 0;
    	    }
    	    }
    	    else{ // It runs when SW0 is open and SW1 position does not matter.
        	       continue;
    	    }

    	    lastButtonState = reading;

	}
	return 0;
}

void delay_func()
{
    uint16_t i=0;
    for(;i<1000;i++)
    {
        asm volatile("NOP");
    }
    /*
     108:	80 e0       	ldi	r24, 0x00	; 0                                     1
 10a:	90 e0       	ldi	r25, 0x00	; 0                                         1
 10c:	00 00       	nop                                                 1
 10e:	01 96       	adiw	r24, 0x01	; 1                             2
 110:	23 e0       	ldi	r18, 0x03	; 3                                 1
 112:	88 3e       	cpi	r24, 0xE8	; 232                               1
 114:	92 07       	cpc	r25, r18                                        1
 116:	d1 f7       	brne	.-12     	; 0x10c <delay_func+0x4>        1/2
 118:	08 95       	ret                                                         4
 */

}
