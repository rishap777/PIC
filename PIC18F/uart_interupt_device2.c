/*

Author:  Rishap singh

*/

#define _XTAL_FREQ 8000000
#include<pic18f4550.h>
#include<xc.h>
#define Fosc 8000000
unsigned char rxd[200],z=0,g;


void InitUART(unsigned int baudrate)
{
  TRISCbits.RC6 = 0; // TX pin set as output
  TRISCbits.RC7 = 1; // RX pin set as input

  SPBRG = (unsigned char)(((Fosc /64)/baudrate)-1);
  BAUDCON = 0b00000000;                     //Non-inverted data; 8-bit baudrate generator

  TXSTA = 0b00100000;                       //Asynchronous 8-bit; Transmit enabled; Low speed baudrate select
  RCSTA = 0b10010000;                       //Serial port enabled; 8-bit data; single receive enabled
  INTCONbits.GIE=1;
   INTCONbits.PEIE=1;
   PIE1bits.RCIE=1;
   PIR1bits.RCIF=0;
}

void u_transmit(unsigned char data)
{
    while(TXSTAbits.TRMT == 0);             //Wait while transmit register is empty

    TXREG = data;                           //Transmit data
}
void u_tx_string(unsigned char *data)
{
    while(*data)
    {
        u_transmit(*data);
        data++;
    }
}
void interrupt ISR()
{
        while(RCIF==0);
        g=RCREG;
        
         PIR1bits.RCIF=0;
 }



void main()
{
    InitUART(9600);
    while(1)
    {
        u_transmit('r');
        __delay_ms(10000);
    }


}
