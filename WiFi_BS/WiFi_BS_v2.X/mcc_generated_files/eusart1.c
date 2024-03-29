/**
  EUSART1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart1.c

  @Summary
    This is the generated driver implementation file for the EUSART1 driver using MPLAB� Code Configurator

  @Description
    This header file provides implementations for driver APIs for EUSART1.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC18F23K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
 */
#include "eusart1.h"

/**
  Section: EUSART1 APIs
 */

void EUSART1_Initialize(void) {
    // Set the EUSART1 module to the options selected in the user interface.

    // ABDOVF no_overflow; RCIDL idle; BRG16 16bit_generator; WUE disabled; CKTXP async_noninverted_sync_fallingedge; ABDEN disabled; DTRXP not_inverted; 
    BAUD1CON = 0x48;

    // SPEN enabled; OERR no_error; RX9 8-bit; RX9D 0x0; CREN enabled; ADDEN disabled; SREN disabled; FERR no_error; 
    RC1STA = 0x90;

    // TRMT TSR_empty; TX9 8-bit; TX9D 0x0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave_mode; 
    TX1STA = 0x26;

    // Baud Rate = 9600; SPBRGL 130; 
    SPBRG1 = 0x82;

    // Baud Rate = 9600; SPBRGH 6; 
    SPBRGH1 = 0x06;

}

uint8_t EUSART1_Read(void) {

    while (PIR1bits.RC1IF == 0) {
    }

    if (RC1STAbits.OERR == 1) {
        // EUSART1 error - restart
        RCSTA1bits.OERR = 0;
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
    }

    return RCREG1;
}

void EUSART1_Write(uint8_t txData) {
    //while (0 == PIR1bits.TX1IF) {
    //}
    while (TXSTA1bits.TRMT == 0) {
    }

    TXREG1 = txData; // Write the data byte to the USART.
}
/**
  End of File
 */
