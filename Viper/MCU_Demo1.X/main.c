/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"
#include "global.h"

/*
                         Main application
 */

uint8_t lm8335Input, lm8335Previous, i;

void Fix_Initialize(void){
    
    ODCONA = 0x00;                 // standard push-pull drive on output port pins
    ODCONB = 0x00;                 // standard push-pull drive on output port pins

    WPUC = 0xFF;                   // weak pull-ups enabled on input pins
    OPTION_REGbits.nWPUEN = 0x00;  // change to the generated code   
}

void main(void){
    SYSTEM_Initialize();  
    Fix_Initialize();       // Corrections to auto-generated code
    
    lm8335Input = (IO_RC2_GetValue() << 7) + 
                  (IO_RC3_GetValue() << 6) +
                  (IO_RC4_GetValue() << 5) +
                  (IO_RC5_GetValue() << 4);
    lm8335Previous = lm8335Input+1;        // ensure they are different
    
    while (1){
        // Add your application code
        
        SPI_DATA_Toggle();         // Toggle every time round the loop
        
        if (IO_RC0_GetValue())
            LED1_SetHigh();
        else
            LED1_SetLow();
        
        if (IO_RC1_GetValue())
            LED2_SetHigh();
        else
            LED2_SetLow();

        if (IO_RC6_GetValue())
            SPI_SEL_SetHigh();
        else
            SPI_SEL_SetLow();

        if (IO_RC7_GetValue())
            DAC1_SetOutput(1);
        else
            DAC1_SetOutput(0);


        // read bits in reverse order
        lm8335Input = (IO_RC2_GetValue() << 7) + 
                      (IO_RC3_GetValue() << 6) +
                      (IO_RC4_GetValue() << 5) +
                      (IO_RC5_GetValue() << 4);

        if (lm8335Input != lm8335Previous) {
            lm8335Previous = lm8335Input;
            OE_SetHigh();                    // Disable shift register
            
            // Start with additional clock cycle
            PE_D_LAT = 0;                    // Should already be zero
            CLK_GLOBAL_SetHigh();            // Clock is left low so start with high
            asm("NOP");
            CLK_GLOBAL_SetLow();             // This shifts data into register (due to inversion)
            
            for (i=0; i<8; i++) {
                PE_D_LAT = lm8335Input & 0x01;
                CLK_GLOBAL_SetHigh();
                asm("NOP");
                CLK_GLOBAL_SetLow();        // This shifts data into register (due to inversion)
                lm8335Input = lm8335Input >> 1;
            }
            
            // Additional clock cycle to load into output register
            PE_D_LAT = 0;
            CLK_GLOBAL_SetHigh();
            asm("NOP");
            CLK_GLOBAL_SetLow();             
            asm("NOP");

            OE_SetLow();                     // Enable shift register
        };
    }
}
/**
 End of File
 */