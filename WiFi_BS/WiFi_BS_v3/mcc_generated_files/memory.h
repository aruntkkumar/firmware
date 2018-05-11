/**
  MEMORY Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    memory.h

  @Summary
    This is the generated header file for the MEMORY driver using MPLAB� Code Configurator

  @Description
    This header file provides APIs for driver for MEMORY.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC18F23K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 *******************************************************************************/

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

#ifndef _MEMORY_H
#define _MEMORY_H

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

    /**
      Section: Macro Declarations
     */

#define WRITE_FLASH_BLOCKSIZE    64
#define ERASE_FLASH_BLOCKSIZE    64
#define END_FLASH                8192

    /**
      Section: Flash Module APIs
     */

    /**
      @Summary
        Reads a data byte from Flash

      @Description
        This routine reads a data byte from given Flash address

      @Preconditions
        None

      @Param
        flashAddr - Flash program memory location from which data has to be read

      @Returns
        Data byte read from given Flash address

      @Example
        <code>
        uint8_t    readByte;
        uint32_t    flashAddr = 0x7D00;

        readByte = FLASH_ReadByte(flashAddr);
        </code>
     */
    uint8_t FLASH_ReadByte(uint32_t flashAddr);

    /**
      @Summary
        Reads a data word from Flash

      @Description
        This routine reads a data word from given Flash address

      @Preconditions
        None

      @Param
        flashAddr - Flash program memory location from which data has to be read

      @Returns
        Data word read from given Flash address

      @Example
        <code>
        uint16_t    readWord;
        uint32_t    flashAddr = 0x7D00;

        readWord = FLASH_ReadWord(flashAddr);
        </code>
     */
    uint16_t FLASH_ReadWord(uint32_t flashAddr);

    /**
     @Summary
       Writes a data byte into Flash

     @Description
       This routine writes the given data byte into mentioned Flash address.

       This routine intially reads block of data (from Flash) into RAM, updates
       data values in RAM, and writes back updated values to Flash.

     @Preconditions
       None

     @Param
       flashAddr      - Flash program memory location to which data has to be written
     *flashRdBufPtr - Pointer to RAM buffer of size 'ERASE_FLASH_BLOCKSIZE' at least
       byte           - Data byte to be written in Flash

     @Returns
       None

     @Example
       <code>
       uint8_t    writeData = 0xAA;
       uint32_t    flashAddr = 0x7D00;
       uint8_t    Buf[ERASE_FLASH_BLOCKSIZE];

       FLASH_WriteWord(flashAddr, Buf, writeData);
       </code>
     */
    void FLASH_WriteByte(uint32_t flashAddr, uint8_t *flashRdBufPtr, uint8_t byte);

    /**
      @Summary
        Writes data to complete block of Flash

      @Description
        This routine writes data bytes to complete block in Flash program memory

      @Preconditions
        None

      @Param
        writeAddr      - A valid block starting address in Flash
     *flashWrBufPtr - Pointer to an array of size 'WRITE_FLASH_BLOCKSIZE' at least

      @Returns
        -1, if the given address is not a valid block starting address of Flash
        0, in case of valid block starting address

      @Example
        <code>
        #define FLASH_ROW_ADDRESS     0x7D00

        uint8_t wrBlockData[] =
        {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
            0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
        };

        // write to Flash memory block
        FLASH_WriteBlock((uint32_t)FLASH_ROW_ADDRESS, (uint8_t *)wrBlockData);
        </code>
     */
    int8_t FLASH_WriteBlock(uint32_t writeAddr, uint8_t *flashWrBufPtr);

    /**
      @Summary
        Erases complete Flash program memory block

      @Description
        This routine erases complete Flash program memory block

      @Preconditions
        None

      @Param
        baseAddr - A valid block starting address in Flash program memory

      @Returns
        None

      @Example
        <code>
        uint32_t    flashBlockStartAddr = 0x7D00;

        FLASH_EraseBlock(flashBlockStartAddr);
        </code>
     */
    void FLASH_EraseBlock(uint32_t baseAddr);

    /**
      Section: Data EEPROM Module APIs
     */

    /**
      @Summary
        Writes a data byte to Data EEPROM

      @Description
        This routine writes a data byte to given Data EEPROM location

      @Preconditions
        None

      @Param
        bAdd  - Data EEPROM location to which data to be written
        bData - Data to be written to Data EEPROM location

      @Returns
        None

      @Example
        <code>
        uint8_t dataeeAddr = 0x10;
        uint8_t dataeeData = 0x55;

        DATAEE_WriteByte(dataeeAddr, dataeeData);
        </code>
     */
    void DATAEE_WriteByte(uint8_t bAdd, uint8_t bData);

    /**
      @Summary
        Reads a data byte from Data EEPROM

      @Description
        This routine reads a data byte from given Data EEPROM location

      @Preconditions
        None

      @Param
        bAdd  - Data EEPROM location from which data has to be read

      @Returns
        Data byte read from given Data EEPROM location

      @Example
        <code>
        uint8_t dataeeAddr = 0x10;
        uint8_t readData;

        readData = DATAEE_ReadByte(dataeeAddr);
        </code>
     */
    uint8_t DATAEE_ReadByte(uint8_t bAdd);

#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif // _MEMORY_H
/**
 End of File
 */

