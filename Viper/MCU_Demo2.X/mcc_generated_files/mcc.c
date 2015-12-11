/**
  @Generated MPLAB® Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1619
        Driver Version    :  1.02
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

// Configuration bits: selected in the GUI

// CONFIG1
#pragma config IESO = ON    // Internal/External Switch Over->Internal External Switch Over mode is enabled
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config PWRTE = OFF    // Power-up Timer Enable->PWRT disabled
#pragma config FOSC = INTOSC    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled
#pragma config MCLRE = OFF    // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config LPBOR = ON    // Low-Power Brown Out Reset->Low-Power BOR is enabled
#pragma config PPS1WAY = ON    // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config LVP = ON    // Low-Voltage Programming Enable->Low-voltage programming enabled
#pragma config ZCD = OFF    // Zero Cross Detect Disable Bit->ZCD disable.  ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config PLLEN = ON    // PLL Enable Bit->4x PLL is always enabled
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.

// CONFIG3
#pragma config WDTCWS = WDTCWS100    // WDT Window Select->100 percent window open time (Legacy WDT) 
#pragma config WDTCCS = MFINTOSC    // WDT Input Clock Selector->31.25 kHz HFINTOSC (MFINTOSC)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable->WDT controlled by the SWDTEN bit in the WDTCON register
#pragma config WDTCPS = WDTCPSE    // WDT Period Select->1:524299 (16 s period)

#include "mcc.h"

void SYSTEM_Initialize(void) {
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    WWDT_Initialize();
    DAC1_Initialize();
}

void OSCILLATOR_Initialize(void) {
    // SPLLEN disabled; SCS FOSC; IRCF 16MHz_HF; 
    OSCCON = 0x78;
    // OSTS intosc; HFIOFR disabled; HFIOFS not0.5percent_acc; PLLR disabled; MFIOFR disabled; HFIOFL not2percent_acc; LFIOFR disabled; 
    OSCSTAT = 0x00;
    // TUN 0x0; 
    OSCTUNE = 0x00;
    // Set the secondary oscillator

}

void WWDT_Initialize(void) {
    // Initializes the WWDT to the default states configured in the MCC GUI
    WDTCON0 = WDTCPS;
    WDTCON1 = WDTCWS | WDTCCS;

}

void WWDT_SoftEnable(void) {
    // WWDT software enable. 
    WDTCON0bits.SEN = 1;
}

void WWDT_SoftDisable(void) {
    // WWDT software disable.
    WDTCON0bits.SEN = 0;
}

bool WWDT_TimeOutStatusGet(void) {
    // Return the status of WWDT time out reset.
    return (PCONbits.nRWDT);
}

bool WWDT_WindowViolationStatusGet(void) {
    // Return the status of WWDT window violation reset.
    return (PCONbits.nWDTWV);
}

void WWDT_TimerClear(void) {
    // Disable the interrupt,read back the WDTCON0 reg for arming, 
    // clearing the WWDT and enable the interrupt.
    uint8_t readBack = 0;

    bool state = GIE;
    GIE = 0;
    readBack = WDTCON0;
    CLRWDT();
    GIE = state;
}

/**
 End of File
 */