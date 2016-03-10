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
        Device            :  PIC16LF1618
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
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config MCLRE = OFF    // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config FOSC = INTOSC    // ->INTOSC oscillator: I/O function on CLKIN pin
#pragma config BOREN = OFF    // Brown-out Reset Enable->Brown-out Reset disabled
#pragma config PWRTE = ON    // Power-up Timer Enable->PWRT enabled
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled

// CONFIG2
#pragma config PLLEN = ON    // PLL Enable Bit->4x PLL is always enabled
#pragma config LPBOR = OFF    // Low-Power Brown Out Reset->Low-Power BOR is disabled
#pragma config LVP = ON    // Low-Voltage Programming Enable->Low-voltage programming enabled
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config ZCD = OFF    // Zero Cross Detect Disable Bit->ZCD disable.  ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config PPS1WAY = ON    // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.

// CONFIG3
#pragma config WDTCWS = WDTCWSSW    // WDT Window Select->Software WDT window size control (WDTWS bits)
#pragma config WDTCPS = WDTCPS1F    // WDT Period Select->Software Control (WDTPS)
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#pragma config WDTCCS = SWC    // WDT Input Clock Selector->Software control, controlled by WDTCS bits

#include "mcc.h"

void SYSTEM_Initialize(void) {
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    DAC1_Initialize();
}

void OSCILLATOR_Initialize(void) {
    // SCS INTOSC; SPLLEN disabled; IRCF 16MHz_HF; 
    OSCCON = 0x78;
    // LFIOFR disabled; HFIOFL not2percent_acc; PLLR disabled; HFIOFS not0.5percent_acc; HFIOFR disabled; MFIOFR disabled; 
    OSCSTAT = 0x00;
    // TUN 0x0; 
    OSCTUNE = 0x00;
    // Set the secondary oscillator

}


/**
 End of File
 */