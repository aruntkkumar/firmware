/**
  High Endurance Flash Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    HEFlash.c

  Summary:
    This is the source file for High Endurance Flash - EEPROM emulation and support routines.

  Description:
    This source file contains EEPROM emulation and support routines for High Endurance Flash, taken from AN1673.
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
#include "../../HEFlash.h"


/**
  Section: High Endurance Flash functions
 */

char HEFLASH_writeBlock( char radd, char* data, char count)
{
    // 1. Obtain absolute address in HE FLASH row
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START;

    // 2. Check input parameters
    if ( ( count > FLASH_ROWSIZE) || (radd >= HEFLASH_MAXROWS))
        // Return parameter error
        return -1;  

    // 3. Erase the entire row
    FLASH_erase( add);

    // 4. Fill the latches with data
    while( count > 1)
    {
        // Load data in latches without writing
        FLASH_write( add++, (unsigned) *data++, 1);
        count--;
    }
    // No delay here!!!

    // 5. Last byte of data -> write
    FLASH_write( add, (unsigned) *data, 0);

    // NOTE: 2ms typ. delay here!!!

    // 6. Return success
    // 0 success, 1 = write error
    return PMCON1bits.WRERR;    

} // HEFLASH_writeBlock


char HEFLASH_readBlock( char *buffer, char radd, char count)
{
    // 1. Obtain absolute address in HE FLASH row
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START;

    // 2. Check input parameters
    if ( ( count > FLASH_ROWSIZE) || (radd >= HEFLASH_MAXROWS))
        return -1;

    // 3. Read content
    while ( count > 0)
    {
        *buffer++ = (char) FLASH_read( add++);
        count--;
    }
    
    // 4. Success
    return 0;
} // HEFLASH_readBlock


char HEFLASH_readByte( char radd, char offset)
{
    // 1. Add offset into HE Flash memory
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START + offset;

    // 2. Read content
    return (char) FLASH_read( add);
} // HEFLASH_read
/**
 End of File
 */