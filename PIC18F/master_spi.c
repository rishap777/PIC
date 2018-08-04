/*

Author:  Rishap singh

*/


#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_EC // Oscillator Selection bits (Internal oscillator, CLKO function on RA6, EC used by USB (INTCKO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF

#define Fosc 8000000
#define _XTAL_FREQ 8000000
#define SPI_CS      LATAbits.LATA5
#include<p18f4550.h>

void delay1(unsigned int time)
{
 unsigned int i,j;
    for(i=0;i<time;i++)
    {
        for(j=0;j<100;j++);
    }
}
void InitUART(unsigned int baudrate)
{
  TRISCbits.RC6 = 0; // TX pin set as output
  TRISCbits.RC7 = 1; // RX pin set as input

  SPBRG = (unsigned char)(((Fosc /64)/baudrate)-1);
  BAUDCON = 0b00000000;                     //Non-inverted data; 8-bit baudrate generator

  TXSTA = 0b00100000;                       //Asynchronous 8-bit; Transmit enabled; Low speed baudrate select
  RCSTA = 0b10010000;                       //Serial port enabled; 8-bit data; single receive enabled
}

void SendChar(unsigned char data)
{
    while(TXSTAbits.TRMT == 0);             //Wait while transmit register is empty

    TXREG = data;                           //Transmit data
}


void InitSPI()
{
    ADCON1 = 0x0f;
    TRISBbits.TRISB0 = 1;   //SDI as input
    TRISBbits.TRISB1 = 0;   //SCK as output
    TRISCbits.TRISC7 = 0;   //SDO as output

    TRISAbits.TRISA5 = 0;   //CS as output
    TRISEbits.RE1 = 0;      //HOLD Pin

     SPI_CS = 1;
 
    SSPCON1bits.SSPEN = 0;
    SSPSTATbits.SMP = 1;    //set data to be sampled at the middle
    SSPSTATbits.CKE = 0;  //Transmition occurs on transition from active to Idle clock state

    SSPCON1bits.CKP   = 1;    //idle state of clock is LOW
    SSPCON1bits.SSPM0 = 0;    //spi clock is Fosc/64
    SSPCON1bits.SSPM1 = 1;
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM3 = 0;

    SSPCON1bits.SSPEN = 1;  //Enable the SPI Interface
}

void SPI_Transmit(unsigned char Data)
{
    PIR1bits.SSPIF = 0;
    SSPBUF = Data;
    while(!PIR1bits.SSPIF);     //wait till transmission activity is over
    //delay1(1);
    PIR1bits.SSPIF = 0;         //clear the interrupt flag
    Data = SSPBUF;              //empty the SSPBUF register
}

unsigned char SPI_Receive()
{
   unsigned char RecData;
     PIR1bits.SSPIF = 0;
    SSPBUF = 0x54;              //send a dummy data
    while(!PIR1bits.SSPIF);     //wait till transmission/reception is complete
   // __delay_us(5);
    RecData = SSPBUF;
    PIR1bits.SSPIF = 0;
    return RecData;
}


void main()
{
 unsigned char i=0;
    OSCCON = 0x72;		/* Use internal frequency 8 MHz */
   // INTCON2bits.RBPU=0;		/* Enable internal Pull-up of PORTB */

  //  lcd_init();
    InitUART(9600);
    InitSPI();
        SPI_CS = 1;

    while (1)
    {
       // SSPCON1bits.SSPEN = 1;
        SPI_CS = 0;
        
       SPI_Transmit('y');
      //  i=SPI_Receive();
       // SendChar(i);
       __delay_ms(50);
      SPI_CS = 1;
  // SSPCON1bits.SSPEN = 0;
   

    }
}