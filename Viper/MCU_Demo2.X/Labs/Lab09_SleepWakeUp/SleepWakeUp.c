/**
  Sleep Wakeup Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    SleepWakeUp.c

  Summary:
    This is the source file for the Sleep Wakeup lab

  Description:
    This source file contains the code on how the Sleep Wakeup lab works.
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
void SleepWakeUp(void)
{
    // Check if this state is running
    if(!labState){        
        // Turn ON D4 and D6
        D4_LAT = D6_LAT = 1;   
        
        // Turn OF D5 and D7
        D5_LAT = D7_LAT = 0; 
        
        // Enable Watchdog Timer
        WDTCON0bits.SEN = 1;  
        
        // Enter SLEEP MODE
        SLEEP();   
        
        // Change the state to RUNNING
        labState = RUNNING;                                                     
    }
    
    if(labState){        
        // Wait 16 seconds for the WDT time out and then turn OFF D4 and D6
        // and turn ON D5 and D7
        D4_LAT = D6_LAT = 0;   
        D5_LAT = D7_LAT = 1;     
        
        // Disable Watchdog Timer
        WDTCON0bits.SEN = 0; 
    }

    // Check if a switch event occurs
    if(switchEvent){   
        // Change the state to NOT_RUNNING
        labState = NOT_RUNNING;                                       
    }
}
/**
 End of File
 */