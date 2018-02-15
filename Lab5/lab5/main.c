/*
 */

#include <avr/io.h>
#include "LCDmodule.h"
void PrintByte(char *,char *,char); // PrintByte function prototype
int main(void)
{
    char LCDtext[16] = "Atten";
    unsigned char Abyte = 1;
    int flag =0;
    unsigned char reading; // current button info
    unsigned char reading2;
    unsigned char lastButtonState = 0x00;
    unsigned char lastButtonState2 = 0x00;

    DDRB = 0xFF;
    DDRC = 0b11111100;
    DDRD = 0xFE;
    ADMUX = 0b00100001; // 01 analog input AREF 1 for ADLAR MSF ADC[9:2]
    //ADMUX = 0b00100000; // part1
    ADCSRA = 0b10000110; // enable ADC disable start conversion 125 kHz 64 prescaler
    ADCSRB = 0b00000000; // disable auto trigger
/*
    LCD_Init();
    LCD_MoveCursor(1,1); //LCD Cursor goes to the 1,1
    LCD_WriteString(LCDtext);*/

    while(1)
    {
        ADCSRA = 0b11000110;
        PORTD |= _BV(PIN5);
        while(ADCSRA & (1<<ADSC));
        PORTD &= ~_BV(PORTD5);
        /*
        PrintByte(LCDtext, "", ADCH);    // 6us
        LCD_MoveCursor(1,7);
        LCD_WriteString(LCDtext);
*/

        reading = (PIND & _BV( PIND0 ));
        reading2 = (PIND & _BV( PIND1 ));
        if((reading != lastButtonState) && Abyte>0)
        {
            if(reading ==0x00)
            {
            Abyte--;
            }
        }
        if((reading2 != lastButtonState2)&& Abyte<6)
        {
            if(reading2 ==0x00)
            {
            Abyte++;
            }
        }
    PORTB = ADCH >> (Abyte-1);
    lastButtonState = reading;
    lastButtonState2 = reading2;
    }

    return 0;
}
