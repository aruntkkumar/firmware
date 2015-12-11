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
        Device            :  PIC16LF1618
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
//#include <delays.h>
/*
                         Main application
 */
const uint8_t DacOutput[4] = {42,70,85,99}; // 42=0.3V,70=0.5,85=0.6,99=0.7
//All are in reverse order for SPI mode transfer
const uint8_t USIDLOW[44] =    {0x80, 0x4C, 0x27, 0x40, //PWR Mode: Normal
                                0x80, 0x4C, 0xB0, 0x41, //F6-F7 ON
                                0x80, 0x4C, 0x30, 0x7F, //F1-F6 ON
                                0x80, 0x4C, 0xB0, 0x00, //F7 ON
                                0x80, 0x4C, 0x30, 0x01, //F6 ON
                                0x80, 0x4C, 0x30, 0x02, //F5 ON
                                0x80, 0x4C, 0x30, 0x04, //F4 ON
                                0x80, 0x4C, 0x30, 0x08, //F3 ON
                                0x80, 0x4C, 0x30, 0x10, //F2 ON
                                0x80, 0x4C, 0x30, 0x20, //F1 ON
                                0x80, 0x4C, 0x30, 0x40};//F1-F7 OFF
const uint8_t USIDHIGH[44] =   {0x80, 0x5C, 0x07, 0x40, 
                                0x80, 0x5C, 0x90, 0x41, 
                                0x80, 0x5C, 0x10, 0x7F, 
                                0x80, 0x5C, 0x90, 0x00, 
                                0x80, 0x5C, 0x10, 0x01, 
                                0x80, 0x5C, 0x10, 0x02, 
                                0x80, 0x5C, 0x10, 0x04, 
                                0x80, 0x5C, 0x10, 0x08, 
                                0x80, 0x5C, 0x10, 0x10, 
                                0x80, 0x5C, 0x10, 0x20, 
                                0x80, 0x5C, 0x10, 0x40};
uint8_t Byte1, Byte2, Byte3;                // First 3 bytes from RFFE
uint8_t readDummy;

void MIPIDATA (uint8_t i){
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDHIGH[j]);
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDHIGH[(i+j)]);
}

void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    DAC1_Initialize();
    PIN_MANAGER_Initialize();  
    SPI_Initialize();
//    ODCONA = 0x00;                 // standard push-pull drive on output port pins
//    ODCONB = 0x00;                 // standard push-pull drive on output port pins

//    WPUC = 0xFF;                   // weak pull-ups enabled on input pins
//    OPTION_REGbits.nWPUEN = 0x00;  // change to the generated code  
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
//    TRISC = 0xFF;
    DAC1_SetOutput(0);
    CTLA_SetLow();                                      //RF1 ON
    CTLB_SetLow();
    MAIN_NIC_LDO_EN_SetLow();                           //0: OFF / MCM sleep mode
    AUX_NIC_LDO_EN_SetLow();
    while (1) {
        while ((PORTC & 0xC0) != 0xC0){}
            Byte1 = PORTC;                                //Sequence starts for Bits 7&6=1&1
        while ((PORTC & 0xC0) != 0x40){}
            Byte2 = PORTC & 0x3F;                       //Sequence when 7&6 bits toggles (0 1)
        while ((PORTC & 0xC0) != 0x80){}
            Byte3 = PORTC & 0x3F;                       //Sequence when 7&6 bits toggles (1 0)
        if (((Byte1 & 0x10) == 0x10) && ((Byte1 & 0x0C) != 0x00))   //ANT_SEL=01/11 & TRX_SEL>0 (DAC Stage)
            DAC1_SetOutput(DacOutput[(Byte1 & 0x03)]);    //DAC Output
        else
            DAC1_SetOutput(0);
/*    // Main NIC LDO & Aux NIC LDO Power Enable Sequence
        if ((Byte1 & 0x3C) == 0x20){
                MAIN_NIC_LDO_EN_SetLow();               //Main_NIC_LDO is Low
                AUX_NIC_LDO_EN_SetHigh();               //Aux_NIC_LDO is High        
        }else{      if (((Byte1 & 0x30) != 0x00) && ((Byte1 & 0x0C) != 0x00)){
                        if ((Byte2 == 5) || (Byte2 == 8) || (Byte2 == 12) || (Byte2 == 13) || (Byte2 == 20) || (Byte2 == 26) || (Byte2 == 29) || (Byte3 == 5) || (Byte3 == 8) || (Byte3 == 12) || (Byte3 == 13) || (Byte3 == 20) || (Byte3 == 26) || (Byte3 == 29)){
                            MAIN_NIC_LDO_EN_SetHigh();  //3: Two antennas selected
                            AUX_NIC_LDO_EN_SetHigh();                        
                        }else{
                            MAIN_NIC_LDO_EN_SetLow();   //2: Aux antenna selected
                            AUX_NIC_LDO_EN_SetHigh();                            
                        }                
        }else{
            MAIN_NIC_LDO_EN_SetLow();                   //0: OFF / MCM sleep mode
            AUX_NIC_LDO_EN_SetLow();            
        }
        }    */
    // RF Switch Selection Sequence
    if (Byte2 == 3 && Byte3 == 7){
        CTLA_SetLow();                                  // RF3 is ON
        CTLB_SetHigh();
    }
    else{ if ((Byte2 == 7  && Byte3 == 0) || (Byte2 == 30 && Byte3 == 0) || (Byte2 == 41 && Byte3 == 0) || (Byte2 == 5  && Byte3 == 30) || (Byte2 == 7  && Byte3 == 20) || (Byte2 == 12 && Byte3 == 30) || (Byte2 == 41 && Byte3 == 41)){
        CTLA_SetHigh();                                 // RF2 is ON
        CTLB_SetLow();
    }
    else{
        CTLA_SetLow();                                  // RF1 is ON
        CTLB_SetLow();
    }  
    }
//    for (int i=0; i<8; i++){                            //Working SPI loop
//        readDummy = SPI_Exchange8bit(USIDHIGH[i]);
    //        SDO_PORT = readDummy;
    //        SSP1CON1bits.WCOL = 0;
    //
    //        SSPBUF = ALLINONE[i];
    //
    //        while (SSP1STATbits.BF == 0) {
    //    }
    //        SDO_PORT = SSPBUF;
//        }   
    MIPIDATA (4);
    MIPIDATA (8);
    }
}
   