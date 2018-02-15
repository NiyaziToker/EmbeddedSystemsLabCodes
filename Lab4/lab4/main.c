/*
 */

#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

void PrintByte(char *,char *,char); // PrintByte function prototype

int main(void)
{
    char LCDtext[16] = "Atten= ";
    int Abyte = 0;
    int flag =0;

    unsigned char reading; // current button info
    unsigned char reading2;
    unsigned char lastButtonState = 0x00;
    unsigned char lastButtonState2 = 0x00;

    DDRB = 0xFF;
    DDRD = 0b11111100;

    LCD_Init();
    LCD_MoveCursor(1,1); //LCD Cursor goes to the 1,1
    LCD_WriteString(LCDtext);
    while(1)
    {
        PORTD = 0b00100000;
        reading = (PIND & _BV( PIND0 ));
        reading2 = (PIND & _BV( PIND1 ));
        if((reading != lastButtonState) && Abyte>=0)
        {
            if(reading ==0x00)
            {
            Abyte--;
            flag = 1;
            }
            //while(!(PIND & _BV(PIND0)));
        }
        if((reading2 != lastButtonState2)&& Abyte<12)
        {
            if(reading2 ==0x00)
            {
            Abyte++;
            flag = 1;

            }

            //while(!(PIND & _BV(PIND1)));

        }

        if(flag == 1)
        {
            PORTD = 0b00010000; //Time marker for PrintByte
            PrintByte(Abyte, "", Abyte);    // 6us
           // PORTD = 0b00100000; //Time marker for LCD functions
            LCD_MoveCursor(1,7);
            LCD_WriteString(Abyte);
            PORTD = 0b00000000;
            flag = 0; //unset the flag for key pressing operation

        }
        PORTD = 0b00000000;
        //flag = 0; //unset the flag for key pressing operation
        lastButtonState = reading;
        lastButtonState2 = reading2;
        _delay_us(500);
        //flag = 0;

    }
    return 0;
}
