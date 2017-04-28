/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC18F23K22
        Version           :  1.01
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set L_SP2TR_VC aliases
#define L_SP2TR_VC_TRIS               TRISB0
#define L_SP2TR_VC_LAT                LATB0
#define L_SP2TR_VC_PORT               PORTBbits.RB0
#define L_SP2TR_VC_WPU                WPUB0
#define L_SP2TR_VC_ANS                ANSB0
#define L_SP2TR_VC_SetHigh()    do { LATB0 = 1; } while(0)
#define L_SP2TR_VC_SetLow()   do { LATB0 = 0; } while(0)
#define L_SP2TR_VC_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define L_SP2TR_VC_GetValue()         PORTBbits.RB0
#define L_SP2TR_VC_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define L_SP2TR_VC_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define L_SP2TR_VC_SetPullup()    do { WPUB0 = 1; } while(0)
#define L_SP2TR_VC_ResetPullup()   do { WPUB0 = 0; } while(0)
#define L_SP2TR_VC_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define L_SP2TR_VC_SetDigitalMode()   do { ANSB0 = 0; } while(0)
// get/set L_SP4T_VC2 aliases
#define L_SP4T_VC2_TRIS               TRISB1
#define L_SP4T_VC2_LAT                LATB1
#define L_SP4T_VC2_PORT               PORTBbits.RB1
#define L_SP4T_VC2_WPU                WPUB1
#define L_SP4T_VC2_ANS                ANSB1
#define L_SP4T_VC2_SetHigh()    do { LATB1 = 1; } while(0)
#define L_SP4T_VC2_SetLow()   do { LATB1 = 0; } while(0)
#define L_SP4T_VC2_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define L_SP4T_VC2_GetValue()         PORTBbits.RB1
#define L_SP4T_VC2_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define L_SP4T_VC2_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define L_SP4T_VC2_SetPullup()    do { WPUB1 = 1; } while(0)
#define L_SP4T_VC2_ResetPullup()   do { WPUB1 = 0; } while(0)
#define L_SP4T_VC2_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define L_SP4T_VC2_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set L_SP4T_VC1 aliases
#define L_SP4T_VC1_TRIS               TRISB2
#define L_SP4T_VC1_LAT                LATB2
#define L_SP4T_VC1_PORT               PORTBbits.RB2
#define L_SP4T_VC1_WPU                WPUB2
#define L_SP4T_VC1_ANS                ANSB2
#define L_SP4T_VC1_SetHigh()    do { LATB2 = 1; } while(0)
#define L_SP4T_VC1_SetLow()   do { LATB2 = 0; } while(0)
#define L_SP4T_VC1_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define L_SP4T_VC1_GetValue()         PORTBbits.RB2
#define L_SP4T_VC1_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define L_SP4T_VC1_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define L_SP4T_VC1_SetPullup()    do { WPUB2 = 1; } while(0)
#define L_SP4T_VC1_ResetPullup()   do { WPUB2 = 0; } while(0)
#define L_SP4T_VC1_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define L_SP4T_VC1_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set L_SP2TL_VC aliases
#define L_SP2TL_VC_TRIS               TRISB3
#define L_SP2TL_VC_LAT                LATB3
#define L_SP2TL_VC_PORT               PORTBbits.RB3
#define L_SP2TL_VC_WPU                WPUB3
#define L_SP2TL_VC_ANS                ANSB3
#define L_SP2TL_VC_SetHigh()    do { LATB3 = 1; } while(0)
#define L_SP2TL_VC_SetLow()   do { LATB3 = 0; } while(0)
#define L_SP2TL_VC_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define L_SP2TL_VC_GetValue()         PORTBbits.RB3
#define L_SP2TL_VC_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define L_SP2TL_VC_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define L_SP2TL_VC_SetPullup()    do { WPUB3 = 1; } while(0)
#define L_SP2TL_VC_ResetPullup()   do { WPUB3 = 0; } while(0)
#define L_SP2TL_VC_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define L_SP2TL_VC_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set R_SP2TR_VC aliases
#define R_SP2TR_VC_TRIS               TRISC0
#define R_SP2TR_VC_LAT                LATC0
#define R_SP2TR_VC_PORT               PORTCbits.RC0
#define R_SP2TR_VC_SetHigh()    do { LATC0 = 1; } while(0)
#define R_SP2TR_VC_SetLow()   do { LATC0 = 0; } while(0)
#define R_SP2TR_VC_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define R_SP2TR_VC_GetValue()         PORTCbits.RC0
#define R_SP2TR_VC_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define R_SP2TR_VC_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

// get/set R_SP4T_VC2 aliases
#define R_SP4T_VC2_TRIS               TRISC1
#define R_SP4T_VC2_LAT                LATC1
#define R_SP4T_VC2_PORT               PORTCbits.RC1
#define R_SP4T_VC2_SetHigh()    do { LATC1 = 1; } while(0)
#define R_SP4T_VC2_SetLow()   do { LATC1 = 0; } while(0)
#define R_SP4T_VC2_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define R_SP4T_VC2_GetValue()         PORTCbits.RC1
#define R_SP4T_VC2_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define R_SP4T_VC2_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

// get/set R_SP4T_VC1 aliases
#define R_SP4T_VC1_TRIS               TRISC2
#define R_SP4T_VC1_LAT                LATC2
#define R_SP4T_VC1_PORT               PORTCbits.RC2
#define R_SP4T_VC1_ANS                ANSC2
#define R_SP4T_VC1_SetHigh()    do { LATC2 = 1; } while(0)
#define R_SP4T_VC1_SetLow()   do { LATC2 = 0; } while(0)
#define R_SP4T_VC1_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define R_SP4T_VC1_GetValue()         PORTCbits.RC2
#define R_SP4T_VC1_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define R_SP4T_VC1_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define R_SP4T_VC1_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define R_SP4T_VC1_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set R_SP2TL_VC aliases
#define R_SP2TL_VC_TRIS               TRISC3
#define R_SP2TL_VC_LAT                LATC3
#define R_SP2TL_VC_PORT               PORTCbits.RC3
#define R_SP2TL_VC_ANS                ANSC3
#define R_SP2TL_VC_SetHigh()    do { LATC3 = 1; } while(0)
#define R_SP2TL_VC_SetLow()   do { LATC3 = 0; } while(0)
#define R_SP2TL_VC_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define R_SP2TL_VC_GetValue()         PORTCbits.RC3
#define R_SP2TL_VC_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define R_SP2TL_VC_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define R_SP2TL_VC_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define R_SP2TL_VC_SetDigitalMode()   do { ANSC3 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */