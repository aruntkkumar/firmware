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
        Device            :  PIC16LF1618
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

// get/set DAC1OUT aliases
#define DAC1OUT_TRIS               TRISA0
#define DAC1OUT_LAT                LATA0
#define DAC1OUT_PORT               RA0
#define DAC1OUT_ANS                ANSA0
#define DAC1OUT_SetHigh()    do { LATA0 = 1; } while(0)
#define DAC1OUT_SetLow()   do { LATA0 = 0; } while(0)
#define DAC1OUT_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define DAC1OUT_GetValue()         RA0
#define DAC1OUT_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define DAC1OUT_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define DAC1OUT_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define DAC1OUT_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set PE_OE aliases
#define PE_OE_TRIS               TRISA1
#define PE_OE_LAT                LATA1
#define PE_OE_PORT               RA1
#define PE_OE_ANS                ANSA1
#define PE_OE_SetHigh()    do { LATA1 = 1; } while(0)
#define PE_OE_SetLow()   do { LATA1 = 0; } while(0)
#define PE_OE_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define PE_OE_GetValue()         RA1
#define PE_OE_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define PE_OE_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define PE_OE_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define PE_OE_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set MAIN_NIC_LDO_EN aliases
#define MAIN_NIC_LDO_EN_TRIS               TRISA2
#define MAIN_NIC_LDO_EN_LAT                LATA2
#define MAIN_NIC_LDO_EN_PORT               RA2
#define MAIN_NIC_LDO_EN_ANS                ANSA2
#define MAIN_NIC_LDO_EN_SetHigh()    do { LATA2 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetLow()   do { LATA2 = 0; } while(0)
#define MAIN_NIC_LDO_EN_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define MAIN_NIC_LDO_EN_GetValue()         RA2
#define MAIN_NIC_LDO_EN_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define MAIN_NIC_LDO_EN_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define MAIN_NIC_LDO_EN_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set PE_D aliases
#define PE_D_TRIS               TRISA4
#define PE_D_LAT                LATA4
#define PE_D_PORT               RA4
#define PE_D_ANS                ANSA4
#define PE_D_SetHigh()    do { LATA4 = 1; } while(0)
#define PE_D_SetLow()   do { LATA4 = 0; } while(0)
#define PE_D_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define PE_D_GetValue()         RA4
#define PE_D_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define PE_D_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define PE_D_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define PE_D_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set PE_CLK aliases
#define PE_CLK_TRIS               TRISA5
#define PE_CLK_LAT                LATA5
#define PE_CLK_PORT               RA5
#define PE_CLK_SetHigh()    do { LATA5 = 1; } while(0)
#define PE_CLK_SetLow()   do { LATA5 = 0; } while(0)
#define PE_CLK_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define PE_CLK_GetValue()         RA5
#define PE_CLK_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define PE_CLK_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

// get/set AUX_NIC_LDO_EN aliases
#define AUX_NIC_LDO_EN_TRIS               TRISB4
#define AUX_NIC_LDO_EN_LAT                LATB4
#define AUX_NIC_LDO_EN_PORT               RB4
#define AUX_NIC_LDO_EN_ANS                ANSB4
#define AUX_NIC_LDO_EN_SetHigh()    do { LATB4 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetLow()   do { LATB4 = 0; } while(0)
#define AUX_NIC_LDO_EN_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define AUX_NIC_LDO_EN_GetValue()         RB4
#define AUX_NIC_LDO_EN_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define AUX_NIC_LDO_EN_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define AUX_NIC_LDO_EN_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set SDATA1_MIPI aliases
#define SDATA1_MIPI_TRIS               TRISB5
#define SDATA1_MIPI_LAT                LATB5
#define SDATA1_MIPI_PORT               RB5
#define SDATA1_MIPI_ANS                ANSB5
#define SDATA1_MIPI_SetHigh()    do { LATB5 = 1; } while(0)
#define SDATA1_MIPI_SetLow()   do { LATB5 = 0; } while(0)
#define SDATA1_MIPI_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define SDATA1_MIPI_GetValue()         RB5
#define SDATA1_MIPI_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define SDATA1_MIPI_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define SDATA1_MIPI_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define SDATA1_MIPI_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set SDATA2_MIPI aliases
#define SDATA2_MIPI_TRIS               TRISB6
#define SDATA2_MIPI_LAT                LATB6
#define SDATA2_MIPI_PORT               RB6
#define SDATA2_MIPI_WPU                WPUB6
#define SDATA2_MIPI_SetHigh()    do { LATB6 = 1; } while(0)
#define SDATA2_MIPI_SetLow()   do { LATB6 = 0; } while(0)
#define SDATA2_MIPI_Toggle()   do { LATB6 = ~LATB6; } while(0)
#define SDATA2_MIPI_GetValue()         RB6
#define SDATA2_MIPI_SetDigitalInput()    do { TRISB6 = 1; } while(0)
#define SDATA2_MIPI_SetDigitalOutput()   do { TRISB6 = 0; } while(0)

#define SDATA2_MIPI_SetPullup()    do { WPUB6 = 1; } while(0)
#define SDATA2_MIPI_ResetPullup()   do { WPUB6 = 0; } while(0)
// get/set SCLK_MIPI aliases
#define SCLK_MIPI_TRIS               TRISB7
#define SCLK_MIPI_LAT                LATB7
#define SCLK_MIPI_PORT               RB7
#define SCLK_MIPI_WPU                WPUB7
#define SCLK_MIPI_SetHigh()    do { LATB7 = 1; } while(0)
#define SCLK_MIPI_SetLow()   do { LATB7 = 0; } while(0)
#define SCLK_MIPI_Toggle()   do { LATB7 = ~LATB7; } while(0)
#define SCLK_MIPI_GetValue()         RB7
#define SCLK_MIPI_SetDigitalInput()    do { TRISB7 = 1; } while(0)
#define SCLK_MIPI_SetDigitalOutput()   do { TRISB7 = 0; } while(0)

#define SCLK_MIPI_SetPullup()    do { WPUB7 = 1; } while(0)
#define SCLK_MIPI_ResetPullup()   do { WPUB7 = 0; } while(0)
// get/set IO_RC0 aliases
#define IO_RC0_TRIS               TRISC0
#define IO_RC0_LAT                LATC0
#define IO_RC0_PORT               RC0
#define IO_RC0_ANS                ANSC0
#define IO_RC0_SetHigh()    do { LATC0 = 1; } while(0)
#define IO_RC0_SetLow()   do { LATC0 = 0; } while(0)
#define IO_RC0_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define IO_RC0_GetValue()         RC0
#define IO_RC0_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define IO_RC0_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

#define IO_RC0_SetAnalogMode()   do { ANSC0 = 1; } while(0)
#define IO_RC0_SetDigitalMode()   do { ANSC0 = 0; } while(0)
// get/set IO_RC1 aliases
#define IO_RC1_TRIS               TRISC1
#define IO_RC1_LAT                LATC1
#define IO_RC1_PORT               RC1
#define IO_RC1_ANS                ANSC1
#define IO_RC1_SetHigh()    do { LATC1 = 1; } while(0)
#define IO_RC1_SetLow()   do { LATC1 = 0; } while(0)
#define IO_RC1_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define IO_RC1_GetValue()         RC1
#define IO_RC1_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define IO_RC1_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

#define IO_RC1_SetAnalogMode()   do { ANSC1 = 1; } while(0)
#define IO_RC1_SetDigitalMode()   do { ANSC1 = 0; } while(0)
// get/set IO_RC2 aliases
#define IO_RC2_TRIS               TRISC2
#define IO_RC2_LAT                LATC2
#define IO_RC2_PORT               RC2
#define IO_RC2_ANS                ANSC2
#define IO_RC2_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_RC2_SetLow()   do { LATC2 = 0; } while(0)
#define IO_RC2_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_RC2_GetValue()         RC2
#define IO_RC2_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IO_RC2_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define IO_RC2_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set IO_RC3 aliases
#define IO_RC3_TRIS               TRISC3
#define IO_RC3_LAT                LATC3
#define IO_RC3_PORT               RC3
#define IO_RC3_ANS                ANSC3
#define IO_RC3_SetHigh()    do { LATC3 = 1; } while(0)
#define IO_RC3_SetLow()   do { LATC3 = 0; } while(0)
#define IO_RC3_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define IO_RC3_GetValue()         RC3
#define IO_RC3_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define IO_RC3_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define IO_RC3_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define IO_RC3_SetDigitalMode()   do { ANSC3 = 0; } while(0)
// get/set IO_RC4 aliases
#define IO_RC4_TRIS               TRISC4
#define IO_RC4_LAT                LATC4
#define IO_RC4_PORT               RC4
#define IO_RC4_WPU                WPUC4
#define IO_RC4_SetHigh()    do { LATC4 = 1; } while(0)
#define IO_RC4_SetLow()   do { LATC4 = 0; } while(0)
#define IO_RC4_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define IO_RC4_GetValue()         RC4
#define IO_RC4_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define IO_RC4_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define IO_RC4_SetPullup()    do { WPUC4 = 1; } while(0)
#define IO_RC4_ResetPullup()   do { WPUC4 = 0; } while(0)
// get/set IO_RC5 aliases
#define IO_RC5_TRIS               TRISC5
#define IO_RC5_LAT                LATC5
#define IO_RC5_PORT               RC5
#define IO_RC5_WPU                WPUC5
#define IO_RC5_SetHigh()    do { LATC5 = 1; } while(0)
#define IO_RC5_SetLow()   do { LATC5 = 0; } while(0)
#define IO_RC5_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define IO_RC5_GetValue()         RC5
#define IO_RC5_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

#define IO_RC5_SetPullup()    do { WPUC5 = 1; } while(0)
#define IO_RC5_ResetPullup()   do { WPUC5 = 0; } while(0)
// get/set IO_RC6 aliases
#define IO_RC6_TRIS               TRISC6
#define IO_RC6_LAT                LATC6
#define IO_RC6_PORT               RC6
#define IO_RC6_ANS                ANSC6
#define IO_RC6_SetHigh()    do { LATC6 = 1; } while(0)
#define IO_RC6_SetLow()   do { LATC6 = 0; } while(0)
#define IO_RC6_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define IO_RC6_GetValue()         RC6
#define IO_RC6_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define IO_RC6_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define IO_RC6_SetAnalogMode()   do { ANSC6 = 1; } while(0)
#define IO_RC6_SetDigitalMode()   do { ANSC6 = 0; } while(0)
// get/set IO_RC7 aliases
#define IO_RC7_TRIS               TRISC7
#define IO_RC7_LAT                LATC7
#define IO_RC7_PORT               RC7
#define IO_RC7_ANS                ANSC7
#define IO_RC7_SetHigh()    do { LATC7 = 1; } while(0)
#define IO_RC7_SetLow()   do { LATC7 = 0; } while(0)
#define IO_RC7_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define IO_RC7_GetValue()         RC7
#define IO_RC7_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define IO_RC7_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define IO_RC7_SetAnalogMode()   do { ANSC7 = 1; } while(0)
#define IO_RC7_SetDigitalMode()   do { ANSC7 = 0; } while(0)

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