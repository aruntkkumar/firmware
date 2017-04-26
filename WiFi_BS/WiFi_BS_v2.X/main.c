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
        Device            :  PIC18F23K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
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

#include "mcc_generated_files/mcc.h"

uint8_t rssi[9], quality[9], rssimax, qualitymax, rssiindex[9] = {0,0,0,0,0,0,0,0,0}, qualityindex[9] = {0,0,0,0,0,0,0,0,0}, index;

void switchstate(uint8_t a) {
    switch (a) {
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        case 7:
            break;
        case 8:
            break;
        case 9:
            break;
        default:
            break;
    }
    return;
}

void getvalues (uint8_t b) {
        EUSART1_Write(0x52);
        rssi[b] += EUSART1_Read();
        EUSART1_Write(0x4c);
        quality[b] += EUSART1_Read();
        return;
}

void main(void) {
    // Initialize the device
    SYSTEM_Initialize();
    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1) {
    start:
        while (EUSART1_Read() != 0xFF) {
            //DO NOTHING. WAITING FOR THE TRIGGER TO START
        }
        //for (uint8_t i=0; i<9; i++) {
        //    for (uint8_t j=0; j<10; j++) {   //j represents the repetition count
        //        switchstate(i+1);
        //        __delay_us(5);
        //        EUSART1_Write(0x52);
        //        rssi[i] += EUSART1_Read();
        //        EUSART1_Write(0x4c);
        //        quality[i] += EUSART1_Read();
        //    }
        //}
        for (uint8_t i=0; i<5; i++) {
            switch (i) {
                case 0:
                    switchstate(5);
                    break;
                case 1:
                    switchstate(4);
                    break;
                case 2:
                    switchstate(6);
                    break;
                case 3:
                    switchstate(2);
                    break;
                case 4:
                    switchstate(8);
                    break;
                default:
                    break;
            }
            __delay_us(25);
            getvalues(i);            
        }

        rssimax = rssi[0];
        index = 0;
        for (uint8_t i=0; i<5; i++) {
            if (rssi[i] < rssimax) {
                //rssiindex[9] = {0,0,0,0,0,0,0,0,0};
                rssiindex[0] = i+1;
                rssiindex[1] = 0;
                rssiindex[2] = 0;
                rssiindex[3] = 0;
                rssiindex[4] = 0;
                //rssiindex[5] = 0;
                //rssiindex[6] = 0;
                //rssiindex[7] = 0;
                //rssiindex[8] = 0;
                //rssimax = rssi[i];
            } else if (rssi[i] == rssimax) {
                rssiindex[index] = i+1;
                index++;
            }
        }
        
        qualitymax = quality[0];
        index = 0;
        for (uint8_t i=0; i<5; i++) {
            if (quality[i] > qualitymax) {
                //qualityindex[9] = {0,0,0,0,0,0,0,0,0};
                qualityindex[0] = i+1;
                qualityindex[1] = 0;
                qualityindex[2] = 0;
                qualityindex[3] = 0;
                qualityindex[4] = 0;
                //qualityindex[5] = 0;
                //qualityindex[6] = 0;
                //qualityindex[7] = 0;
                //qualityindex[8] = 0;
                //qualitymax = quality[i];
            } else if (quality[i] == qualitymax) {
                qualityindex[index] = i+1;
                index++;
            }
        }
        
        for (uint8_t i=0; i<5; i++) {
            for (uint8_t j=0; j<5; j++) {
                if ((rssiindex[i] == qualityindex[j]) && (rssiindex[i] !=0) && (qualityindex[j] !=0)) {
                    switchstate(i);
                    EUSART1_Write(0x53);
                    __delay_us(25);
                    EUSART1_Write(i);
                    __delay_us(25);
                    EUSART1_Write(0x53);
                    __delay_us(25);
                    goto start;
                }
            }
        }
    }
}
/**
 End of File
 */