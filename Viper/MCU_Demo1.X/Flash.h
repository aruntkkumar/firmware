/**
  Flash Memory Program Header File

  @Company
    Microchip Technology Inc.

  @File Name
    Flash.h

  @Summary
    This is the header file for the Flash Memory Program

  @Description
    This header file provides the declaration of 161x Family's memory program.
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
  Section: Macro Declarations
 */

#define FLASH_ROWMASK     FLASH_ROWSIZE-1 

/**
  Section: 2 KW Flash Memory Program
 */
                         
#if defined(__12F1612) || defined(__16F1613)

    // Size of a row
    #define FLASH_ROWSIZE   16    

    // First address in HE Flash memory
    #define HEFLASH_START   0x0780 

    // Last address in HE Flash memory
    #define HEFLASH_END     0x07FF                                              

/**
  Section: 4 KW Flash Memory Program
 */
#elif defined(__16F1614) || defined(__16F1618)

    // Size of a row
    #define FLASH_ROWSIZE   32               

    // First address in HE Flash memory
    #define HEFLASH_START   0x0F80               

    // Last address in HE Flash memory
    #define HEFLASH_END     0x0FFF                                              

/**
  Section: 8 KW Flash Memory Program
 */
#elif defined(__16F1615) || defined(__16F1619)

    // Size of a row
    #define FLASH_ROWSIZE   32      

    // First address in HE Flash memory
    #define HEFLASH_START   0x1F80    

    // Last address in HE Flash memory
    #define HEFLASH_END     0x1FFF                                                
#endif

/**
  Section: Generic Flash functions
 */

/**
 @Description
   Read a word from program Flash memory

 @Param 
   address          source address (absolute FLASH memory address)
 
 @Return          
                    word retrieved from FLASH memory
 */

unsigned FLASH_read( unsigned address);


/**
@Description
  Read a word from configuration Flash memory

@Param 
  address           source address (absolute FLASH memory address)
  
@Return          
                    word retrieved from FLASH memory
*/

unsigned FLASH_readConfig( unsigned address);


/**
@Description
  Read a block of words from program Flash memory

@Param 
  buffer            destination buffer (must be sufficiently large)
  address           source address (absolute FLASH memory address)
  count             number of words to be retrieved
*/

void    FLASH_readBlock( unsigned* buffer, unsigned address, char count);


/**
@Description
  Write a word of data to Flash memory (latches) an actual write is performed only if LWLO = 0, data is latched if LWLO = 1
 
@Param 
  address           destination address (absolute flash memory)
  data              word of data to be written (latched)
  latch             1 = latch, 0 = write
*/

void    FLASH_write( unsigned address, unsigned data, char latch);


/**
@Description
  Erase a row of Flash memory

@Param 
  address           absolute address in Flash contained in selected row
*/
void    FLASH_erase( unsigned address);


/**
 End of File
 */