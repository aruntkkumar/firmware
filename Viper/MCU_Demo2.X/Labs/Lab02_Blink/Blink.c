/**
  Blink Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    Blink.c

  Summary:
    This is the source file for the Blink lab

  Description:
    This source file contains the code on how the Blink lab works.
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
void Blink(void)
{
    // Check if this state is running
    if(!labState){
        // Start Timer1
        TMR1_StartTimer();  
        
        // Clear all LEDs
        D4_LAT = D5_LAT = D6_LAT = D7_LAT = 0; 
        
        // Change the state to RUNNING
        labState = RUNNING;                                                    
    }

    if(labState){
        // Wait for Timer1 to overflow
        while(!TMR1IF);          
        
        // Reload the initial value of TMR1
        TMR1_Reload();    
        
        // Increment the counter variable
        counter++;
        
        // Wait for third overflow for 1.5 secs delay
        if(counter == 3){       
            // Toggle D4
            D4_Toggle();    
            
            // Reset counter to 0
            counter = 0;
        }
        
        // Clear TMR1 Overflow flag
        TMR1IF = 0;                                                             
    }

    // Check if a switch event occurs
    if(switchEvent){
        // Stop Timer1
        TMR1_StopTimer();
        
        // Change the state to NOT_RUNNING
        labState = NOT_RUNNING;                                                    
    }
}
/**
 End of File
 */