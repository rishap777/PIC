/*

Author:  Rishap singh

*/

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_EC    // Oscillator Selection bits (EC oscillator, CLKO function on RA6 (EC))
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
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)


#define _XTAL_FREQ 8000000
#define freq 8000000
#include<pic18f4550.h>
#include<xc.h>
#define lcd_data(data)   {\
    PORTCbits.RC1=0;\
     PORTCbits.RC0=1; \
    PORTD=data;  \
     PORTCbits.RC2=1; \
     delay1(1);  \
      PORTCbits.RC2=0; \
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
    PORTCbits.RC1=0;
    PORTCbits.RC0=0;
    PORTD=cmd;
     PORTCbits.RC2=1;
     delay1(1);
      PORTCbits.RC2=0;
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
    TRISCbits.RC2=0;
    lcd_cmd(0x0E);
    lcd_cmd(0x38);

}

void InitUART(unsigned int baudrate)
{
  TRISCbits.RC6 = 0; // TX pin set as output
  TRISCbits.RC7 = 1; // RX pin set as input

  SPBRG = (unsigned char)(((freq/64)/baudrate)-1);
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


void I2C_Init()
{
    TRISB0=1;		/* Set up I2C lines by setting as input */
    TRISB1=1;
    SSPSTAT=80;		/* Slew rate disabled, other bits are cleared */
    SSPCON1=0x28;	/* Enable SSP port for I2C Master mode,
			clock = FOSC / (4 * (SSPADD+1))*/
    SSPCON2=0;
    SSPADD=0x4E;	/* Clock 100 kHz */
    SSPIE=1;		/* Enable SSPIF interrupt */
    SSPIF=0;
}

void I2C_Ready()
{
    while(BCLIF);	/* Wait if bit collision interrupt flag is set*/

    /* Wait for Buffer full and read write flag*/
    while(SSPSTATbits.BF || (SSPSTATbits.R_nW));
    SSPIF=0;  		/* Clear SSPIF interrupt flag*/
}

char I2C_Write(unsigned char data)
{
      SSPBUF=data;	/* Write data to SSPBUF*/
      I2C_Ready();
      if (ACKSTAT==1)	/* Check for acknowledge bit*/
        return 0;
      else
        return 1;
}

char I2C_Start()
{
    SSPCON2bits.SEN=1;		/* Send start pulse */
    while(SSPCON2bits.SEN);	/* Wait for completion of start pulse */
    SSPIF=0;
    __delay_us(4.7);
    if(!SSPSTATbits.S)		/* Check whether START detected last */
    return 0;	
    else
        return 1;/* Return 0 to indicate start failed */
//    return (I2C_Write(slave_write_address));	/* Write slave device address
						//with write to communicate */
}



void I2C_Ready_sspif()
{
    while(!SSPIF);	/* Wait for operation complete */
    SSPIF=0;		/* Clear SSPIF interrupt flag*/
}



char I2C_Stop()
{
    I2C_Ready();
    PEN=1;		/* Stop communication*/
    while(PEN);		/* Wait for end of stop pulse*/
    SSPIF = 0;
    if (!SSPSTATbits.P)/* Check whether STOP is detected last */
        return 0;
    else
        return 1;		/* If not return 0 to indicate start failed*/
}

void I2C_Ack()
{
    ACKDT=0;		/* Acknowledge data 1:NACK,0:ACK */
    ACKEN=1;		/* Enable ACK to send */
    while(ACKEN);
 }

void I2C_Nack()
{
    ACKDT=1;		/* Acknowledge data 1:NACK,0:ACK */
    ACKEN=1;		/* Enable ACK to send */
    while(ACKEN);
}



char I2C_Read(char flag)
{
        int buffer=0;
        RCEN=1;			/* Enable receive */

	/* Wait for buffer full flag which when complete byte received */
        while(!SSPSTATbits.BF);
        buffer=SSPBUF;		/* Copy SSPBUF to buffer */

	/* Send acknowledgment or negative acknowledgment after read to
	continue or stop reading */
        if(flag==0)
            I2C_Ack();
        else
            I2C_Nack();
        I2C_Ready();
        return(buffer);
}

void bcd_dec(unsigned char value)
{
    unsigned char a,b,c;
    a=(value & 0x0F);
    b=((value>>4) & 0x0F);
    c=(b*10)+a;
    lcd_data(b+48);
    lcd_data(a+48);
    u_transmit(b+48);
    u_transmit(a+48);
}



void main()
{
    unsigned char sec,min,hour,day,date,month,year,s,a,b,c;
    OSCCON=0x73;
    I2C_Init();
    InitUART(9600);
    lcd_init();

   
       // u_transmit('k');
         I2C_Ready();
    s=I2C_Start();
a=I2C_Write(0xD0);     /* address from where time needs to be read*/
b=I2C_Write(0x00);   //pointer
    c=I2C_Stop();
    lcd_cmd(0x01);
 while(1)
    {
      I2C_Ready();
        s=I2C_Start();
   

        a=I2C_Write(0xD0);     /* address from where time needs to be read*/

        b=I2C_Write(0x00);
           b=I2C_Write(0x00);       //seconds
   b=I2C_Write(0x11);        // minutes
   b=I2C_Write(0x06);          // hours//
    b=I2C_Write(0x02);              // day
    b=I2C_Write(0x27);              //date
    b=I2C_Write(0x03);              //month
     b=I2C_Write(0x18);             // year

        c=I2C_Stop();

        I2C_Ready();
    s=I2C_Start();

    a=I2C_Write(0xD1);

    sec = I2C_Read(0);                 /*read data and send ack for continuous reading*/
   min = I2C_Read(0);                 /*read data and send ack for continuous reading*/
   hour = I2C_Read(0);
   day = I2C_Read(0);
   date = I2C_Read(0);
   month = I2C_Read(0);
   year  = I2C_Read(1);
    I2C_Stop();

    lcd_cmd(0x80);
    u_tx_string("\n");
     u_tx_string("hour=");
     lcd_string("TIME:");
    bcd_dec(hour);
    lcd_data(':');
    u_transmit(' ');
    u_tx_string("min=");
    bcd_dec(min);
     lcd_data(':');
    u_transmit(' ');
    u_tx_string("sec=");
    bcd_dec(sec);


    lcd_cmd(0xC0);
    u_tx_string("\n");
     u_tx_string("date=");
       lcd_string("DATE:");
    bcd_dec(date);
    lcd_data(':');
    u_transmit(' ');
    u_tx_string("month=");
    bcd_dec(month);
     lcd_data(':');
    u_transmit(' ');
    u_tx_string("year=");
    bcd_dec(year);
    switch(day)
    {
        case 1:
            lcd_string("Mon");
        break;

        case 2:
            lcd_string("Tue");
        break;

        case 3:
            lcd_string("Wed");
        break;
        case 4:
            lcd_string("Thu");
        break;
        case 5:
            lcd_string("Fri");
        break;
        case 6:
            lcd_string("Sat");
        break;
        case 7:
            lcd_string("Sun");
        break;

        default:
        break;
       }
    
   
    __delay_ms(100);
    __delay_ms(10);
    }

}