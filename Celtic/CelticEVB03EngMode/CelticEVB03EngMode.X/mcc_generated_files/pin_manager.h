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
        Device            :  PIC18LF23K22
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

// get/set MAIN_NIC_LDO_EN aliases
#define MAIN_NIC_LDO_EN_TRIS               TRISA0
#define MAIN_NIC_LDO_EN_LAT                LATA0
#define MAIN_NIC_LDO_EN_PORT               PORTAbits.RA0
#define MAIN_NIC_LDO_EN_ANS                ANSA0
#define MAIN_NIC_LDO_EN_SetHigh()    do { LATA0 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetLow()   do { LATA0 = 0; } while(0)
#define MAIN_NIC_LDO_EN_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define MAIN_NIC_LDO_EN_GetValue()         PORTAbits.RA0
#define MAIN_NIC_LDO_EN_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define MAIN_NIC_LDO_EN_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set AUX_NIC_LDO_EN aliases
#define AUX_NIC_LDO_EN_TRIS               TRISA1
#define AUX_NIC_LDO_EN_LAT                LATA1
#define AUX_NIC_LDO_EN_PORT               PORTAbits.RA1
#define AUX_NIC_LDO_EN_ANS                ANSA1
#define AUX_NIC_LDO_EN_SetHigh()    do { LATA1 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetLow()   do { LATA1 = 0; } while(0)
#define AUX_NIC_LDO_EN_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define AUX_NIC_LDO_EN_GetValue()         PORTAbits.RA1
#define AUX_NIC_LDO_EN_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define AUX_NIC_LDO_EN_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set DACOUT aliases
#define DACOUT_TRIS               TRISA2
#define DACOUT_LAT                LATA2
#define DACOUT_PORT               PORTAbits.RA2
#define DACOUT_ANS                ANSA2
#define DACOUT_SetHigh()    do { LATA2 = 1; } while(0)
#define DACOUT_SetLow()   do { LATA2 = 0; } while(0)
#define DACOUT_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define DACOUT_GetValue()         PORTAbits.RA2
#define DACOUT_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define DACOUT_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define DACOUT_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define DACOUT_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set SDATA1_MIPI aliases
#define SDATA1_MIPI_TRIS               TRISA3
#define SDATA1_MIPI_LAT                LATA3
#define SDATA1_MIPI_PORT               PORTAbits.RA3
#define SDATA1_MIPI_ANS                ANSA3
#define SDATA1_MIPI_SetHigh()    do { LATA3 = 1; } while(0)
#define SDATA1_MIPI_SetLow()   do { LATA3 = 0; } while(0)
#define SDATA1_MIPI_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define SDATA1_MIPI_GetValue()         PORTAbits.RA3
#define SDATA1_MIPI_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define SDATA1_MIPI_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define SDATA1_MIPI_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define SDATA1_MIPI_SetDigitalMode()   do { ANSA3 = 0; } while(0)
// get/set SDATA2_MIPI aliases
#define SDATA2_MIPI_TRIS               TRISA4
#define SDATA2_MIPI_LAT                LATA4
#define SDATA2_MIPI_PORT               PORTAbits.RA4
#define SDATA2_MIPI_SetHigh()    do { LATA4 = 1; } while(0)
#define SDATA2_MIPI_SetLow()   do { LATA4 = 0; } while(0)
#define SDATA2_MIPI_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define SDATA2_MIPI_GetValue()         PORTAbits.RA4
#define SDATA2_MIPI_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define SDATA2_MIPI_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

// get/set SCLK_MIPI aliases
#define SCLK_MIPI_TRIS               TRISA5
#define SCLK_MIPI_LAT                LATA5
#define SCLK_MIPI_PORT               PORTAbits.RA5
#define SCLK_MIPI_ANS                ANSA5
#define SCLK_MIPI_SetHigh()    do { LATA5 = 1; } while(0)
#define SCLK_MIPI_SetLow()   do { LATA5 = 0; } while(0)
#define SCLK_MIPI_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define SCLK_MIPI_GetValue()         PORTAbits.RA5
#define SCLK_MIPI_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define SCLK_MIPI_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define SCLK_MIPI_SetAnalogMode()   do { ANSA5 = 1; } while(0)
#define SCLK_MIPI_SetDigitalMode()   do { ANSA5 = 0; } while(0)
// get/set CTLA_SW1 aliases
#define CTLA_SW1_TRIS               TRISA6
#define CTLA_SW1_LAT                LATA6
#define CTLA_SW1_PORT               PORTAbits.RA6
#define CTLA_SW1_SetHigh()    do { LATA6 = 1; } while(0)
#define CTLA_SW1_SetLow()   do { LATA6 = 0; } while(0)
#define CTLA_SW1_Toggle()   do { LATA6 = ~LATA6; } while(0)
#define CTLA_SW1_GetValue()         PORTAbits.RA6
#define CTLA_SW1_SetDigitalInput()    do { TRISA6 = 1; } while(0)
#define CTLA_SW1_SetDigitalOutput()   do { TRISA6 = 0; } while(0)

// get/set CTLB_SW1 aliases
#define CTLB_SW1_TRIS               TRISA7
#define CTLB_SW1_LAT                LATA7
#define CTLB_SW1_PORT               PORTAbits.RA7
#define CTLB_SW1_SetHigh()    do { LATA7 = 1; } while(0)
#define CTLB_SW1_SetLow()   do { LATA7 = 0; } while(0)
#define CTLB_SW1_Toggle()   do { LATA7 = ~LATA7; } while(0)
#define CTLB_SW1_GetValue()         PORTAbits.RA7
#define CTLB_SW1_SetDigitalInput()    do { TRISA7 = 1; } while(0)
#define CTLB_SW1_SetDigitalOutput()   do { TRISA7 = 0; } while(0)

// get/set GPO0 aliases
#define GPO0_TRIS               TRISB0
#define GPO0_LAT                LATB0
#define GPO0_PORT               PORTBbits.RB0
#define GPO0_WPU                WPUB0
#define GPO0_ANS                ANSB0
#define GPO0_SetHigh()    do { LATB0 = 1; } while(0)
#define GPO0_SetLow()   do { LATB0 = 0; } while(0)
#define GPO0_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define GPO0_GetValue()         PORTBbits.RB0
#define GPO0_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define GPO0_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define GPO0_SetPullup()    do { WPUB0 = 1; } while(0)
#define GPO0_ResetPullup()   do { WPUB0 = 0; } while(0)
#define GPO0_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define GPO0_SetDigitalMode()   do { ANSB0 = 0; } while(0)
// get/set GPO1 aliases
#define GPO1_TRIS               TRISB1
#define GPO1_LAT                LATB1
#define GPO1_PORT               PORTBbits.RB1
#define GPO1_WPU                WPUB1
#define GPO1_ANS                ANSB1
#define GPO1_SetHigh()    do { LATB1 = 1; } while(0)
#define GPO1_SetLow()   do { LATB1 = 0; } while(0)
#define GPO1_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define GPO1_GetValue()         PORTBbits.RB1
#define GPO1_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define GPO1_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define GPO1_SetPullup()    do { WPUB1 = 1; } while(0)
#define GPO1_ResetPullup()   do { WPUB1 = 0; } while(0)
#define GPO1_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define GPO1_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set GPO2 aliases
#define GPO2_TRIS               TRISB2
#define GPO2_LAT                LATB2
#define GPO2_PORT               PORTBbits.RB2
#define GPO2_WPU                WPUB2
#define GPO2_ANS                ANSB2
#define GPO2_SetHigh()    do { LATB2 = 1; } while(0)
#define GPO2_SetLow()   do { LATB2 = 0; } while(0)
#define GPO2_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define GPO2_GetValue()         PORTBbits.RB2
#define GPO2_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define GPO2_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define GPO2_SetPullup()    do { WPUB2 = 1; } while(0)
#define GPO2_ResetPullup()   do { WPUB2 = 0; } while(0)
#define GPO2_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define GPO2_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set GPO3 aliases
#define GPO3_TRIS               TRISB3
#define GPO3_LAT                LATB3
#define GPO3_PORT               PORTBbits.RB3
#define GPO3_WPU                WPUB3
#define GPO3_ANS                ANSB3
#define GPO3_SetHigh()    do { LATB3 = 1; } while(0)
#define GPO3_SetLow()   do { LATB3 = 0; } while(0)
#define GPO3_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define GPO3_GetValue()         PORTBbits.RB3
#define GPO3_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define GPO3_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define GPO3_SetPullup()    do { WPUB3 = 1; } while(0)
#define GPO3_ResetPullup()   do { WPUB3 = 0; } while(0)
#define GPO3_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define GPO3_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set GPO4 aliases
#define GPO4_TRIS               TRISB4
#define GPO4_LAT                LATB4
#define GPO4_PORT               PORTBbits.RB4
#define GPO4_WPU                WPUB4
#define GPO4_ANS                ANSB4
#define GPO4_SetHigh()    do { LATB4 = 1; } while(0)
#define GPO4_SetLow()   do { LATB4 = 0; } while(0)
#define GPO4_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define GPO4_GetValue()         PORTBbits.RB4
#define GPO4_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define GPO4_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define GPO4_SetPullup()    do { WPUB4 = 1; } while(0)
#define GPO4_ResetPullup()   do { WPUB4 = 0; } while(0)
#define GPO4_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define GPO4_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set GPO5 aliases
#define GPO5_TRIS               TRISB5
#define GPO5_LAT                LATB5
#define GPO5_PORT               PORTBbits.RB5
#define GPO5_WPU                WPUB5
#define GPO5_ANS                ANSB5
#define GPO5_SetHigh()    do { LATB5 = 1; } while(0)
#define GPO5_SetLow()   do { LATB5 = 0; } while(0)
#define GPO5_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define GPO5_GetValue()         PORTBbits.RB5
#define GPO5_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define GPO5_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define GPO5_SetPullup()    do { WPUB5 = 1; } while(0)
#define GPO5_ResetPullup()   do { WPUB5 = 0; } while(0)
#define GPO5_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define GPO5_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set GPO6 aliases
#define GPO6_TRIS               TRISB6
#define GPO6_LAT                LATB6
#define GPO6_PORT               PORTBbits.RB6
#define GPO6_WPU                WPUB6
#define GPO6_SetHigh()    do { LATB6 = 1; } while(0)
#define GPO6_SetLow()   do { LATB6 = 0; } while(0)
#define GPO6_Toggle()   do { LATB6 = ~LATB6; } while(0)
#define GPO6_GetValue()         PORTBbits.RB6
#define GPO6_SetDigitalInput()    do { TRISB6 = 1; } while(0)
#define GPO6_SetDigitalOutput()   do { TRISB6 = 0; } while(0)

#define GPO6_SetPullup()    do { WPUB6 = 1; } while(0)
#define GPO6_ResetPullup()   do { WPUB6 = 0; } while(0)
// get/set GPO7 aliases
#define GPO7_TRIS               TRISB7
#define GPO7_LAT                LATB7
#define GPO7_PORT               PORTBbits.RB7
#define GPO7_WPU                WPUB7
#define GPO7_SetHigh()    do { LATB7 = 1; } while(0)
#define GPO7_SetLow()   do { LATB7 = 0; } while(0)
#define GPO7_Toggle()   do { LATB7 = ~LATB7; } while(0)
#define GPO7_GetValue()         PORTBbits.RB7
#define GPO7_SetDigitalInput()    do { TRISB7 = 1; } while(0)
#define GPO7_SetDigitalOutput()   do { TRISB7 = 0; } while(0)

#define GPO7_SetPullup()    do { WPUB7 = 1; } while(0)
#define GPO7_ResetPullup()   do { WPUB7 = 0; } while(0)
// get/set CTLA_SW2 aliases
#define CTLA_SW2_TRIS               TRISC0
#define CTLA_SW2_LAT                LATC0
#define CTLA_SW2_PORT               PORTCbits.RC0
#define CTLA_SW2_SetHigh()    do { LATC0 = 1; } while(0)
#define CTLA_SW2_SetLow()   do { LATC0 = 0; } while(0)
#define CTLA_SW2_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define CTLA_SW2_GetValue()         PORTCbits.RC0
#define CTLA_SW2_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define CTLA_SW2_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

// get/set CTLB_SW2 aliases
#define CTLB_SW2_TRIS               TRISC1
#define CTLB_SW2_LAT                LATC1
#define CTLB_SW2_PORT               PORTCbits.RC1
#define CTLB_SW2_SetHigh()    do { LATC1 = 1; } while(0)
#define CTLB_SW2_SetLow()   do { LATC1 = 0; } while(0)
#define CTLB_SW2_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define CTLB_SW2_GetValue()         PORTCbits.RC1
#define CTLB_SW2_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define CTLB_SW2_SetDigitalOutput()   do { TRISC1 = 0; } while(0)


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