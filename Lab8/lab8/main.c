/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "LCDmodule.h"
#define LCDbufferSize 32
#define USARTbufferSize 32
void PrintByte(char *,char *,char); // PrintByte function prototype
void init_adc();
void init_USART();
void init_timers();
void EnQueueLCDbuffer(unsigned char);
void DeQueueLCDbuffer(void);
void EnQueueUSARTbuffer(unsigned char);
void DeQueueUSARTbuffer(void);

char LCDtext[16];
char USARTtext[16];
char *pText;
char *uText;

unsigned char LCDbuffer[LCDbufferSize];
unsigned char USARTbuffer[USARTbufferSize];
unsigned char Ubyte;
unsigned char Nbyte; // number of bytes stored in LCDbuffer
unsigned char Atten = 1;


int main(void)
{
    DDRB = 0xFF;
    DDRC = 0b11111100;
    DDRD = 0xF0;

    init_adc();
    LCD_Init();
    init_USART();
    init_timers();
    // Insert code
    //unsigned char reading;
    //unsigned char reading2;
    //unsigned char lastButtonState = 0;
    //unsigned char lastButtonState2 = 0;

    while(1)
    { PORTD |= _BV(PORTD4); // Set timing marker.
    _delay_ms(50);
    PORTD &= ~_BV(PORTD4); // Clear timing marker.
    _delay_ms(50);
    }
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
    OCR1A = 1000; // 10000 in decimal

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
void init_USART()
{
    UBRR0H = 0x00;
    UBRR0L = 51;
    UCSR0A = 0x00;
    UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
    //UCSR0B = 0b10011000; // interrupt enable receiver - enable trans - enable receiver transmitter
    UCSR0C = 0b00000110; // async mode no parity  1bit stop bit

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

void EnQueueUSARTbuffer(unsigned char  ByteIn){ //Push data to LCDbuffer
    static unsigned char  *pUSARTin = USARTbuffer;
    static unsigned char  Ucount = 0;
    *pUSARTin = ByteIn;   // data is stored in the buffer
    Ubyte++;
    Ucount++;
    if(Ucount == USARTbufferSize){ //end of buffer
        pUSARTin = USARTbuffer;
        Ucount = 0;
    }
    else{
        pUSARTin ++;
    }
}


void DeQueueUSARTbuffer(void)
{
  static unsigned char  *pUSARTout = USARTbuffer;
  static unsigned char  UScount = USARTbufferSize;
  if (Ubyte > 0)
  { UDR0 = *pUSARTout;
    UScount --;
    if (UScount == 0)
    {
        pUSARTout = USARTbuffer;
      UScount = USARTbufferSize;
    }
    else
      pUSARTout ++;
    Ubyte --;
  }
  return;
}

ISR(TIMER1_COMPA_vect)
{
    static unsigned char counter = 0;
    //PORTD |= _BV(PORTD4); // Set ISR time marker
    TIMSK2 = 0x02;
    ADCSRA = 0b11000110;


    //PORTD |= _BV(PORTD5);
    while(ADCSRA & (1<<ADSC));
    counter++;

    if(Atten % 2 == 0)
        OCR0A = (ADCH >> (Atten/2)) + (ADCH >> (Atten/2+1));
    else
        OCR0A = ADCH >> ((Atten-1)/2);

    PrintByte(LCDtext, "ADC ", OCR0A);    // 6us

    //LCD_MoveCursor(1,1);
    EnQueueLCDbuffer(0x80);
    //LCD_WriteString(LCDtext);
    pText = LCDtext; // initialize pointer to output text
    // Copy text output to LCDbuffer:
    while (*pText != 0x00)
    {
        if(counter==10)
        {
            EnQueueUSARTbuffer(*pText);
        }
        EnQueueLCDbuffer(*pText);
        pText ++;
    }
    if(counter == 10)
    {
        EnQueueUSARTbuffer(0x20);
        counter =0;
    }
    UCSR0B |= _BV(UDRIE0);



   // PORTD &= ~_BV(PORTD4); // Clear ISR time marker
} // end of ISR
ISR(TIMER2_COMPA_vect)
{
//TCNT2 = 0x00;
//PORTD |= _BV(PORTD5); // Set second ISR timing marker.
TCNT2 = 0x00;
if(Nbyte==0)
    TIMSK2 &= ~_BV(OCIE2A);
else
    DeQueueLCDbuffer();
//PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
}
ISR(USART_RX_vect)
{
    //PORTD |= _BV(PORTD5); // Set timing marker.
    static unsigned char received;
    received = UDR0;
    if((received&0xDF) == 'U')
        if(Atten>=0 && Atten<12)
            Atten++;

    if((received&0xDF) == 'D')
        if(Atten>0 && Atten<=12)
            Atten--;

    PrintByte(LCDtext, "Atten", Atten);    // 6us
    //LCD_MoveCursor(2,1);
    EnQueueLCDbuffer(0xC0);
    EnQueueUSARTbuffer(0x0D);
    //LCD_WriteString(LCDtext);
    pText = LCDtext;
    while (*pText != 0x00)
    {
        EnQueueLCDbuffer(*pText);
        EnQueueUSARTbuffer(*pText);
        pText ++;
    }
    TIMSK2 = 0x02;
    UCSR0B |= _BV(UDRIE0);//enable USART data ready interrupt
    //PORTD &= ~_BV(PORTD5); // Clear timing marker.
}

ISR(USART_UDRE_vect)
{
PORTD |= _BV(PORTD5); // Set second ISR timing marker.
if(Ubyte==0)
    UCSR0B &= ~_BV(UDRIE0);
else
    DeQueueUSARTbuffer();
PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
}
