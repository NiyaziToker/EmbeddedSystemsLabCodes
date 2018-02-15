/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "LCDmodule.h"
#define LCDbufferSize 32

void PrintByte(char *,char *,char); // PrintByte function prototype
void init_adc();
void init_timers();
void DeQueueLCDbuffer(void);
void EnQueueLCDbuffer(unsigned char);

char LCDtext[16];
char *pText;

unsigned char LCDbuffer[LCDbufferSize];
unsigned char Nbyte; // number of bytes stored in LCDbuffer
unsigned char Atten = 1;
unsigned char LCDcontrol=0;


int main(void)
{
    DDRB = 0xFF;
    DDRC = 0b11111100;
    DDRD = 0xF0;

    init_adc();
    LCD_Init();
    LCD_MoveCursor(1,1); //LCD Cursor goes to the 1,1
    init_timers();
    // Insert code
    unsigned char reading;
    unsigned char reading2;
    unsigned char lastButtonState = 0;
    unsigned char lastButtonState2 = 0;

    while(1)
    {

    if(Nbyte>0)
        TIMSK2 = 0x02;
    else
        TIMSK2 = 0x00;
    reading = (PIND & _BV( PIND0 ));
    reading2 = (PIND & _BV( PIND1 ));
    if((reading != lastButtonState) && Atten>0)
    {
        if(reading ==0x00)
        {
        Atten--;
        LCDcontrol = 0;
        PrintByte(LCDtext, "", Atten);    // 6us
        //LCD_MoveCursor(2,1);
        EnQueueLCDbuffer(0xC0);
        //LCD_WriteString(LCDtext);
        pText = LCDtext;
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);
            pText ++;
        }
        TIMSK2 = 0x02;
        LCDcontrol = 1;
        }
    }
    if((reading2 != lastButtonState2)&& Atten<12)
    {
        if(reading2 ==0x00)
        {
        Atten++;
        LCDcontrol = 0;
        PrintByte(LCDtext, "", Atten);    // 6us
        //LCD_MoveCursor(2,1);
        EnQueueLCDbuffer(0xC0);
        //LCD_WriteString(LCDtext);
        pText = LCDtext;
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);
            pText ++;
        }
        TIMSK2 = 0x02;
        LCDcontrol = 1;
        }

    }
    lastButtonState = reading;
    lastButtonState2 = reading2;

    }
    return 0;
}
void init_timers()
{

    TCCR0A = 0b10000011; // COM0A 10 COM0B 00 -- -- WGM0[1:0] 11
    TCCR0B = 0b00000001; // FOC0A 0 FOC0B 0 -- -- WGM0[2] 0 CS0[2:0] 001 no prescaler
    TIMSK0 = 0x00;

    TCCR1A = 0x00;
    TCCR1B = 0b00001010;
    TCCR1C = 0x00;
    TIMSK1 |= _BV(OCIE1A);
    OCR1A = 0x03E8; // 1000 in decimal

    TCCR2A = 0b00000010; // COM0A 10 COM0B 00 -- -- WGM0[1:0] 10
    TCCR2B = 0b00000010; // FOC0A 0 FOC0B 0 -- -- WGM0[2] 0 CS0[2:0] 001 no prescaler
    OCR2A = 0x32; // 50us
    TIMSK2 |= _BV(OCIE2A);
    sei();
}
void init_adc()
{
    ADMUX = 0b00100001; // 01 analog input AREF 1 for ADLAR MSF ADC[9:2]
    ADCSRA = 0b10000110; // enable ADC disable start conversion 125 kHz 64 prescaler
    ADCSRB = 0b00000000; // disable auto trigger
}
void EnQueueLCDbuffer(unsigned char  ByteIn){ //Push data to LCDbuffer
    static unsigned char  *pLCDin = LCDbuffer;
    static unsigned char  Acount = 0;
    *pLCDin = ByteIn;   // data is stored in the buffer
    Nbyte++;
    Acount++;
    if(Acount == LCDbufferSize){ //end of buffer
        pLCDin = LCDbuffer;
        Acount = 0;
    }
    else{
        pLCDin ++;
    }
}


void DeQueueLCDbuffer(void)
/* If there are queued data in LCDbuffer (Nbyte > 0), then sends a
   single byte to the LCD module. If the buffer is empty (Nbyte = 0),
   then returns without doing anything. */
{
  static unsigned char  *pLCDout = LCDbuffer;
  static unsigned char  Bcount = LCDbufferSize;
  unsigned char  ByteOut;

  if (Nbyte > 0)
// First send the 4 most significant bits of the LCD data:
  { ByteOut = *pLCDout >> 4;
// Set the register select bit, RS=1, when sending display data to LCD:
    if ( (*pLCDout & 0x80) == 0 )  ByteOut |= 0x10;
// Write 4 MSBs:
    _LCDport = ByteOut;
    _LCDport = 0x40 | ByteOut;  // Set Enable
    _LCDport = ByteOut;         // Clear Enable
    ByteOut = (ByteOut & 0xF0) | (*pLCDout & 0x0F);
    _LCDport = ByteOut;
    _LCDport = 0x40 | ByteOut;  // Set Enable
    _LCDport = ByteOut;         // Clear Enable
    _LCDport = 0x00;  // clear port output
// Update the LCDbuffer status:
    Bcount --;        // decrement buffer position counter
    if (Bcount == 0)  // check if reached end of buffer array
    { pLCDout = LCDbuffer;     // go back to the first element
      Bcount = LCDbufferSize;  // reload buffer position counter
    }
    else
      pLCDout ++;  // increment dequeue buffer pointer
    Nbyte --;      // decrement number of stored bytes
  }
  return;
}

ISR(TIMER1_COMPA_vect)
{
    PORTD |= _BV(PORTD4); // Set ISR time marker
    TIMSK2 = 0x02;
    ADCSRA = 0b11000110;
    //PORTD |= _BV(PORTD5);
    while(ADCSRA & (1<<ADSC));
    PrintByte(LCDtext, "", ADCH);    // 6us
    if(LCDcontrol==1)
    {
        //LCD_MoveCursor(1,1);
        EnQueueLCDbuffer(0x80);
        //LCD_WriteString(LCDtext);
        pText = LCDtext; // initialize pointer to output text
        // Copy text output to LCDbuffer:
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);
            pText ++;
        };

    }
    if(Atten % 2 == 0)
        OCR0A = (ADCH >> (Atten/2)) + (ADCH >> (Atten/2+1));
    else
        OCR0A=ADCH >> ((Atten-1)/2);

    PORTD &= ~_BV(PORTD4); // Clear ISR time marker
} // end of ISR
ISR(TIMER2_COMPA_vect)
{
//TCNT2 = 0x00;
PORTD |= _BV(PORTD5); // Set second ISR timing marker.
TCNT2 = 0x00;
DeQueueLCDbuffer();
PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
}
