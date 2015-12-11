/**
  High Endurance Flash Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    HEF.c

  Summary:
    This is the source file for the HEF lab

  Description:
    This source file contains the code on how the HEF lab works.
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
#include <assert.h>
#include <string.h>
#include "../../mcc_generated_files/mcc.h"
#include "../../global.h"
#include "../../Flash.h"
#include "../../HEFlash.h"

/*
                             Application    
 */
void _fassert( int line, const char *file, const char *expr){
    TRISC = 0xf0;
    PORTC = PORTC+1;
} // _fassert

void HEF(void)
{
    int i = 0;
    
    // Store your data here
    char buffer[];                                                              

    // Check if this state is running
     if(!labState){
        // Enable the Global Interrupts
        INTERRUPT_GlobalInterruptDisable();    
        
        // Enable the Peripheral Interrupts
        INTERRUPT_PeripheralInterruptDisable();     

        // Change the state to RUNNING
        labState = RUNNING;                        
    }

    if(labState){
        // Write value 0b0101
        // Transfer to buffer register
        buffer[0] = 5;               
        
        // Write contents of buffer to 1 byte of Block 1 of HEF memory
        errorCheck = HEFLASH_writeBlock( 1, (void*)&buffer, 1);         
        assert (errorCheck == 0);   
        
        // Read back value display in LED
        // Read current info in HEF memory
        errorCheck = HEFLASH_readBlock((void*)&buffer, 1, 1);       
        assert ( errorCheck == 0);

        rotateReg = *buffer;

        // Determine which LED will light up
        // ie. which bit in the register the 1 has rotated to.
        D4_LAT = rotateReg & 1;
        D5_LAT = (rotateReg & 2) >> 1;
        D6_LAT = (rotateReg & 4) >> 2;
        D7_LAT = (rotateReg & 8) >> 3;
    }

    // Check if a switch event occurs
    if(switchEvent){                
        // Disable the Global Interrupts
        INTERRUPT_GlobalInterruptDisable();    
        
        // Disable the Peripheral Interrupts
        INTERRUPT_PeripheralInterruptDisable();     
        
        // Change the state to NOT_RUNNING
        labState = NOT_RUNNING;                                        
   }
}
/**
 End of File
 */
