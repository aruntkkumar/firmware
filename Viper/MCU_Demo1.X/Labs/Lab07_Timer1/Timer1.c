/**
  Timer1 Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    Timer1.c

  Summary:
    This is the source file for the Timer1 lab

  Description:
    This source file contains the code on how the Timer1 lab works.
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

#include "../../mcc_generated_files/mcc.h"
#include "../../global.h"

/*
                             Application    
 */
void Timer1(void)
{
    // Check if this state is running
    if(!labState){              
        // Start Timer1
        TMR1_StartTimer();  
        
        // Begin with D4 High
        D4_LAT = 1;           
        
        // Clear other LEDs
        D5_LAT = D6_LAT = D7_LAT = 0;   
        
        // Initialize temporary register to begin at 1
        rotateReg = 1;             
        
        // Change the state to RUNNING
        labState = RUNNING;                                                     
    }

    if(labState){
        // Wait for Timer1 to overflow
        while(!TMR1IF);      
        
         // Reload the initial value of TMR1
        TMR1_Reload();             
        
        // Rotate position of LED
        rotateReg = rotateReg<<1; 
        
        // Return to initial position of LED
        if (rotateReg == 16)
            rotateReg = 1;       
        
        // Determine which LED will light up
        // ie. which bit in the register the 1 has rotated to.
        D4_LAT = rotateReg & 1;                                                 
        D5_LAT = (rotateReg & 2) >> 1;                                          
        D6_LAT = (rotateReg & 4) >> 2;
        D7_LAT = (rotateReg & 8) >> 3;
        
        // Clear the TMR1 interrupt flag
        TMR1IF = 0;                                                             
    }

    // Check if a switch event occurs
    if(switchEvent){      
        // Change the state to NOT_RUNNING
        labState = NOT_RUNNING;     
        
        // Stop Timer1
        TMR1_StopTimer();                                                       
    }
}
/**
 End of File
 */