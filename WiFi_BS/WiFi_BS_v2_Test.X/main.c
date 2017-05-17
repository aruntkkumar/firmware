/*
 * File:   main.c
 * Author: arun.kumar
 *
 * Created on 11 May 2017, 11:33
 */


#include <xc.h>

#include "p18f23k22.h"

//#include "main.h"
//#include "delays.h"
//#include "PIC18_Serial.h"

char Hello[5]={'H','E','L','L','O'};
/*****************************************************************************/
//#pragma code
//#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh () {

	if(PIR1bits.RC1IF)
	{
		Serial_echo();
		
	}
}

void Serial_Init(void)
{
	/*§20.0 manuel PIC18f4550 */
 	TRISCbits.TRISC6=1;
	TRISCbits.TRISC7=1;
	ANSELC=0x00;
//	RCSTA1=0x90;	// SPEN=1 valide le port série, CREN=1 valide la réception de donnée
	RCSTA1 = 0b10010000;
	TXSTA1=0b00100010;	// Low speed mode brgh1 , TXEN 1 autorise émission de donnée

	BAUDCON1=0x00;
	SPBRG1=25;		// 9600 baud

	INTCONbits.GIEH=1;
	INTCONbits.PEIE_GIEL=1;
	INTCON=0xC0;

	PIE1bits.RC1IE = 1;

}


void Serial_Putchar(unsigned char cD)
{
	RCSTA1bits.CREN=0;
	while(!PIR1bits.TX1IF);	// si 1 registre d'émission vide, donc on sort de la boucle                 	
	TXREG1=cD;	// On place le caractère cD à envoyer dans le registre TXREG
	while(!TXSTA1bits.TRMT);	// Lorsque TRMT passe à 1 TXREG recoit la donnée cD                         
	RCSTA1bits.CREN=1;
}



void Serial_Putchar_parity(unsigned char cD)
{
//	while(!PIR1bits.TXIF);	// si 1 registre d'émission vide, donc on sort de la boucle    
	TXSTAbits.TX9D=Check_Parity(cD);             	
	TXREG=cD;	// On place le caractère cD à envoyer dans le registre TXREG
	while(!TXSTAbits.TRMT);	// Lorsque TRMT passe à 1 TXREG recoit la donnée cD                         

}


char Serial_Getchar(void)
{
	unsigned char cData;

	cData=RCREG1;   	// Place le caractère reçu dans cData
		RCSTA1bits.CREN=0;	// Désactive la réception de donnée
		PIR1bits.RC1IF=0;	
 		RCSTA1bits.CREN=1;	// Active la réception de donnée
			return cData;	
	
}

char Serial_Getchar_with_parity(void)
{
	unsigned char cData;
	BIT parity;	      	         // Octet de réception de donnée 
	cData=RCREG;   	// Place le caractère reçu dans cData
	parity=RCSTAbits.RX9D;

	       PIR1bits.RCIF=0;	
 			RCSTAbits.CREN=1;	// Active la réception de donnée
			return cData;		
}


void Serial_SendBuff( char* pc_Buff, char s_length)
{
	char i;
	for(i=0;i<s_length;i++)
	{
		Serial_Putchar(pc_Buff[i]);
	}
}

void Serial_echo(void)
{
	Serial_Putchar(Serial_Getchar());

 RCSTAbits.CREN=1;	// Active la réception de donnée
}


BIT Check_Parity(char n)

{
    BIT parity = 0;
    while (n)
    {
        parity = !parity;
        n      = n & (n - 1);
    }       
    return parity;
}

void Serial_SendBuff_Parity(char* pc_Buff, char s_length)
{char i;
	PIE1bits.RCIE = 0;
	
	for(i=0;i<s_length;i++)
	{
		Serial_Putchar_parity(pc_Buff[i]);
	}
	PIE1bits.RCIE = 1;
}

void Activate_RC(void)
{

	INTCONbits.PEIE=1;
	PIE1bits.RCIE = 1;
	RCSTAbits.CREN=1;	//Active la réception de donnée
}

void Deactivate_RC(void)
{

	INTCONbits.PEIE=0;
	PIE1bits.RCIE = 0;
	RCSTAbits.CREN=0;	//Active la réception de donnée
}

void Stop_echo(void)
{
	Deactivate_RC();
	Delay10KTCYx(2000);
	Activate_RC();
}


void init_main()
{
	//Init Internat clock to 16Mhz
	
/*	OSCCONbits.IRCF0=1;//HFINTOSC ? (16 MHz)
	OSCCONbits.IRCF1=1;
	OSCCONbits.IRCF2=1;
	OSCCONbits.OSTS=0;//Device is running from the internal oscillator (HFINTOSC, MFINTOSC or LFINTOSC)
	OSCCONbits.HFIOFS=1;//HFINTOSC frequency is stable
	OSCCONbits.SCS0=1;*///Internal oscillator block
	OSCCON=0x70;	
	OSCCON2=0;

	Serial_Init();
}





void main(void)
{
	init_main();

	Serial_SendBuff(Hello,5);

	while(1)
	{

		//Delay10KTCYx(1000);
        __delay_us(1000);
		Serial_Getchar();
	}
	
}
