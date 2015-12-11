/**
  High Endurance Flash Header File

  @Company
    Microchip Technology Inc.

  @File Name
    HEFlash.h

  @Summary
    This is the header file for the High Endurance Flash lab

  @Description
    This header file provides the type definitions for the HEF lab.
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

#include "Flash.h"

/**
  Section: Macro Declarations
 */
#define HEFLASH_MAXROWS     (HEFLASH_END-HEFLASH_START+1)/FLASH_ROWSIZE

/**
@Description
  Write a block of data to High Endurance Flash the entire block must fit within a row 

@Param 
  radd              HE Flash block number ( 0 - MAXROWS-1)
  buffer            source buffer
  count             number of bytes to write to block (< ROWSIZE)
 
@Return         
  0                 if successful, -1 if parameter error, 1 if write error
*/
char    HEFLASH_writeBlock( char radd, char* buffer, char count);


/**
@Description
  Read a block of data from HE FLASH memory
 
@Param 
  buffer            destination buffer (must be sufficiently large)
  radd              source block of HE FLASH memory ( 0 - MAXROWS-1)
  count             number of bytes to be retrieved ( < ROWSZE)
 
@Return          
  0                 if successful, -1 if parameter error
*/
char    HEFLASH_readBlock( char* buffer, char radd, char count);


/**
@Description
  Read a byte of data from HE FLASH memory

@Param 
  radd              source block of HE FLASH memory ( 0 - MAXROWS-1)
  offset            offset within the HE block ( 0 - ROWSIZE-1)
 
@Return         
                    byte of data retrieved
*/
char    HEFLASH_readByte( char radd, char offset);
/**
 End of File
 */