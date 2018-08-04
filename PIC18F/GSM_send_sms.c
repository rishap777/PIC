/*

Author:  Rishap singh

*/

// CONFIG1L
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
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF

#define Fosc 8000000
#define _XTAL_FREQ 8000000
#include<pic18f4550.h>
#include<xc.h>
#include<string.h>
#include<ctype.h>
#define lcd_data(data)   {\
     PORTCbits.RC0=1; \
    PORTD=data;  \
     PORTCbits.RC1=1; \
     delay1(1);  \
      PORTCbits.RC1=0; \
      delay1(1);  \
}


void delay1(unsigned int time)
{
 unsigned int i,j;
    for(i=0;i<time;i++)
    {
        for(j=0;j<100;j++);
    }
}

void lcd_cmd(unsigned char cmd)
{
    PORTCbits.RC0=0;
    PORTD=cmd;
     PORTCbits.RC1=1;
     delay1(1);
      PORTCbits.RC1=0;
      delay1(1);
}

void lcd_string(unsigned char *c)
{
    while(*c)
    {
        lcd_data(*c);
        c++;

    }


}


void lcd_init()
{
    TRISD=0x00;
    TRISCbits.RC0=0;
    TRISCbits.RC1=0;
    lcd_cmd(0x0E);

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
unsigned char u_receive(void)
{
    while(!PIR1bits.RCIF);                  //Wait till receive buffer becomes full
    return RCREG;                           //Returned received data
}

void main()
{
     OSCCON = 0x72;		/* Use internal frequency 8 MHz */
    TRISB=0x00;
    InitUART(9600);
   // lcd_init();

   
        u_tx_string("at+cmgs=\"+919876543210\"\r");
     // u_transmit(0x1A);
    // u_tx_string("\r");
       __delay_ms(2500);
     u_tx_string("hello//\n by ba");
   // u_tx_string(26);
     // __delay_ms(10);
     u_transmit(0x1A);
      //lcd_data('u');
     
    
    while(1);


}