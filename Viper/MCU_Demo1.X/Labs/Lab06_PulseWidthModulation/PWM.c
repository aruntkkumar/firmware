/**
  Pulse Width Modulation Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    PWM.c

  Summary:
    This is the source file for the PWM lab

  Description:
    This source file contains the code on how the PWM lab works.
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
void PWM(void)
{
    // Check if this state is running
    if(!labState){            
        // Start Timer2
        TMR2_Start();            
        
        // Clear LEDs
        D4_LAT = D5_LAT = D6_LAT = D7_LAT = 0; 
        
        // Set D7 as output of CCP1 using PPS
        RC5PPS = 0b00001100;  
        
        // Change the state to RUNNING
        labState = RUNNING;                                                     
    }

    if(labState){
        // Start ADC conversion
        ADC1_StartConversion(POT1);   
        
        // Wait for the conversion to finish
        while(ADC1_IsConversionDone());    
        
        // Store the top 8 MSbs into CCPR1H
        CCPR1H = ADRESH;        
        
        // Store the 2 LSbs into CCPR1L register to complete the 10bit resolution
        CCPR1L = ADRESL;                                                        
    }

    // Check if a switch event occurs
    if(switchEvent){
        // Restore D7 as a normal output
        RC5PPS = 0b00000000;
        
        // Stop Timer2
        TMR2_Stop();                                                         
        
        // Change the state to NOT_RUNNING
        labState = NOT_RUNNING;                                                
    }
}
/**
 End of File
 */