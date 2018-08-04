/*

Author:  Rishap singh

*/

#define Fosc 8000000
#define _XTAL_FREQ 8000000
#include<pic18f4550.h>
#include<xc.h>

void InitUART(unsigned int baudrate)
{
  TRISCbits.TRISC6 = 0; // TX pin set as output
  TRISCbits.TRISC7 = 1; // RX pin set as input

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

void EEPROM_Write (int address, char data)
{
    /*Write Operation*/
    EEADR=address;	/* Write address to the EEADR register*/
    EEDATA=data;	/* Copy data to the EEDATA register for write */
    EECON1bits.EEPGD=0;	/* Access data EEPROM memory*/
    EECON1bits.CFGS=0;	/* Access flash program or data memory*/
    EECON1bits.WREN=1;	/* Allow write to the memory*/
    INTCONbits.GIE=0;	/* Disable global interrupt*/

    /* Below sequence in EECON2 Register is necessary
    to write data to EEPROM memory*/
    EECON2=0x55;
    EECON2=0xaa;

    EECON1bits.WR=1;	/* Start writing data to EEPROM memory*/
    INTCONbits.GIE=1;	/* Enable interrupt*/

    while(PIR2bits.EEIF==0);/* Wait for write operation complete */
    PIR2bits.EEIF=0;	/* Reset EEIF for further write operation */

}

void EEPROM_WriteString(int address,char *data)
{
    /*Write Operation for String*/
    while(*data!=0)
    {
        EEPROM_Write(address,*data);
        address++;
        *data++;
    }
}

char EEPROM_Read (int address)
{
    /*Read operation*/
    EEADR=address;	/* Read data at location 0x00*/
    EECON1bits.WREN=0;	/* WREN bit is clear for Read operation*/
    EECON1bits.EEPGD=0;	/* Access data EEPROM memory*/
    EECON1bits.RD=1;	/* To Read data of EEPROM memory set RD=1*/
    return(EEDATA);
}
void main()
{
    unsigned char data,read;
    InitUART(9600);

    while(1)
    {
        data=u_receive();
        EEPROM_Write(0x00,data);
        read=EEPROM_Read(0x00);
        u_transmit(read);
//      __delay_ms(1000);
    }




}