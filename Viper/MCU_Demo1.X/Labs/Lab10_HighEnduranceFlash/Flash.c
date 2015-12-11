/**
  Self Write Flash Support Functions Source File

  Company:
    Microchip Technology Inc.

  File Name:
    Flash.c

  Summary:
    This is the source file for Self Write Flash support functions

  Description:
    This source file contains the codes that supports Self Write Flash taken from AN1673.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1619
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v3.05
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

#include <xc.h>
#include "../../Flash.h"

/**
  Section: Generic Flash Functions
 */

unsigned FLASH_readConfig( unsigned address)
{
    // 1. Load the address pointers
    PMADR = address;
    
    // Select the configuration flash address space
    PMCON1bits.CFGS = 1;  
    
    // Next operation will be a read
    PMCON1bits.RD = 1;      
    NOP();
    NOP();

    // 2. Return value read
    return PMDAT;
} // FLASH_config


unsigned FLASH_read( unsigned address)
{
    // 1. Load the address pointers
    PMADR = address;
    
    // Select the flash address space
    PMCON1bits.CFGS = 0; 
    
    // Next operation will be a read
    PMCON1bits.RD = 1;      
    NOP();
    NOP();

    // 2. Return value read
    return PMDAT;
} // FLASH_read


void FLASH_readBlock( unsigned *buffer, unsigned address, char count)
{
    while ( count > 0)
    {
        *buffer++ = FLASH_read( address++);
        count--;
    }
} // FLASH_readBLock


/**
  Section: Unlock Flash Sequence
 */

void _unlock( void)
{
    #asm
        BANKSEL     PMCON2
        MOVLW       0x55
        MOVWF       PMCON2 & 0x7F
        MOVLW       0xAA
        MOVWF       PMCON2 & 0x7F
        BSF         PMCON1 & 0x7F,1    ; set WR bit
        NOP
        NOP
    #endasm
} // unlock


void FLASH_write( unsigned address, unsigned data, char latch)
{
    // 1. Disable interrupts (remember setting)
    char temp = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    // 2. Load the address pointers
    PMADR = address;
    PMDAT = data;
    
    // 1 = latch, 0 = write row
    PMCON1bits.LWLO = latch;
    
    // Select the flash address space
    PMCON1bits.CFGS = 0;  
    
    // Next operation will be a write
    PMCON1bits.FREE = 0; 
    
    // Enable flash memory write/erase
    PMCON1bits.WREN = 1;    

    // 3. Perform unlock sequence
    _unlock();

    // 4. Restore interrupts
    if ( temp)
        INTCONbits.GIE = 1;

} // FLASH_write


void FLASH_erase( unsigned address)
{
    // 1. Disable interrupts (remember setting)
    char temp = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    
    // 2. Load the address pointers
    PMADR = address;
    
    // Select the flash address space
    PMCON1bits.CFGS = 0;  
    
    // Next operation will be an erase
    PMCON1bits.FREE = 1;  
    
    // Nnable flash memory write/erase
    PMCON1bits.WREN = 1;    

    // 3. Perform unlock sequence and erase
    _unlock();

    // 4. Disable writes and restore interrupts
    // Disable flash memory write/erase
    PMCON1bits.WREN = 0;    
    if ( temp)
        INTCONbits.GIE = 1;

} // FLASH_erase
/**
 End of File
 */