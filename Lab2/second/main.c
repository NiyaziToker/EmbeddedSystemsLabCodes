#include <avr/io.h>

int main(void) {
	DDRB = 0xFF; // All port-B pins are configured as an output
	DDRD = 0b11111100; // Least two port-D pins are configured as an input
	PORTB = 0x01; // PINB0 pin is configured as a high
	while(1) {
    	    if( ( PIND & _BV( PIND0 ) ) == 0x00 ){ // It runs when SW0 is closed
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
    	    }
    	    else{ // It runs when SW0 is open and SW1 position does not matter.
        	       continue;
    	    }
	}
	return 0;
}
