/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "LCDmodule.h"
void PrintByte(char *,char *,char); // PrintByte function prototype
void init_adc();
void init_timers();
char LCDtext[16];
int main(void)
{
    //unsigned char Abyte = 1;
    //int flag =0;
    //unsigned char reading; // current button info
    //unsigned char reading2;
    //unsigned char lastButtonState = 0x00;
    //unsigned char lastButtonState2 = 0x00;
    DDRB = 0xFF;
    DDRC = 0b11111100;
    DDRD = 0xF0;

    init_adc();
    LCD_Init();
    LCD_MoveCursor(1,1); //LCD Cursor goes to the 1,1
    init_timers();
    while(1)
    {

    }
    return 0;
}
void init_timers()
{

    TCCR0A = 0b10000011; // COM0A 10 COM0B 00 -- -- WGM0[1:0] 11
    TCCR0B = 0b00000001; // FOC0A 0 FOC0B 0 -- -- WGM0[2] 0 CS0[2:0] 001 no prescaler
    TCCR1A = 0x00;
    TCCR1B = 0b00001010;
    TCCR1C = 0x00;
    OCR1A = 0x03E8; // 1000 in decimal

    TIMSK1 |= _BV(OCIE1A);
    sei();
}
void init_adc()
{
    ADMUX = 0b00100001; // 01 analog input AREF 1 for ADLAR MSF ADC[9:2]
    ADCSRA = 0b10000110; // enable ADC disable start conversion 125 kHz 64 prescaler
    ADCSRB = 0b00000000; // disable auto trigger
}


ISR(TIMER1_COMPA_vect)
{
    static unsigned char Atten = 0;
    static unsigned char reading;
    static unsigned char reading2;
    static unsigned char lastButtonState;
    static unsigned char lastButtonState2;
    PORTD |= _BV(PORTD4); // Set ISR time marker

    ADCSRA = 0b11000110;
    //PORTD |= _BV(PORTD5);
    while(ADCSRA & (1<<ADSC));
    PrintByte(LCDtext, "", ADCH);    // 6us
    PORTD |= _BV(PORTD5);
    LCD_MoveCursor(2,1);
    LCD_WriteString(LCDtext);
    PORTD &= ~_BV(PORTD5);
    reading = (PIND & _BV( PIND0 ));
    reading2 = (PIND & _BV( PIND1 ));
    if((reading != lastButtonState) && Atten>0)
    {
        if(reading ==0x00)
        {
        Atten--;
        }
    }
    if((reading2 != lastButtonState2)&& Atten<12)
    {
        if(reading2 ==0x00)
        {
        Atten++;
        }
    }
    if(Atten % 2 == 0)
        OCR0A = (ADCH >> (Atten/2)) + (ADCH >> (Atten/2+1));
    else
        OCR0A=ADCH >> ((Atten-1)/2);

    lastButtonState = reading;
    lastButtonState2 = reading2;
    PrintByte(LCDtext, "", Atten);    // 6us
    LCD_MoveCursor(1,1);
    LCD_WriteString(LCDtext);
    PORTD &= ~_BV(PORTD4); // Clear ISR time marker
} // end of ISR
