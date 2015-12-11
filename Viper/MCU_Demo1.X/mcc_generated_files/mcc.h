/**
  @Generated MPLAB® Code Configurator Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.h

  @Summary:
    This is the mcc.h file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1619
        Version           :  1.02
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

#ifndef MCC_H
#define	MCC_H
#include <xc.h>
#include "pin_manager.h"
#include <stdint.h>
#include <stdbool.h>
#include "dac1.h"
#include "fvr.h"

#define _XTAL_FREQ  4000000
#define WDTCWS  6
#define WDTCPS  28 
#define WDTCCS  16 

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Initializes the device to the default states configured in the
 *                  MCC GUI
 * @Example
    SYSTEM_Initialize(void);
 */
void SYSTEM_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Initializes the oscillator to the default states configured in the
 *                  MCC GUI
 * @Example
    OSCILLATOR_Initialize(void);
 */
void OSCILLATOR_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Initializes the WWDT to the default states configured in the
 *                  MCC GUI
 * @Example
    WWDT_Initialize();
 */
void WWDT_Initialize(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
   Enable the WWDT by setting the SEN bit.
 * @Example
    WWDT_SoftEnable();
 */
void WWDT_SoftEnable(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
   Disable the WWDT by clearing the SEN bit.
 * @Example
    WWDT_SoftDisable();
 */
void WWDT_SoftDisable(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
   Disable the interrupt, arm the WWDT by reading back the WDTCON0 register
 * clear the WWDT and enable the interrupt.
 * @Example
    WWDT_TimerClear();
 */
void WWDT_TimerClear(void);

/**
 * @Param
    none
 * @Returns
   High --> WWDT reset has not occurred. 
 * Low  --> WWDT reset has  occurred.
 * @Description
    Returns the status of whether the WWDT reset has occurred or not.
 * @Example
    if(WWDT_TimeOutStatusGet())
 */
bool WWDT_TimeOutStatusGet(void);

/**
 * @Param
    none
 * @Returns
   High --> WWDT window violation reset has not occurred. 
 * Low  --> WWDT window violation reset has  occurred.
 * @Description
    Returns the status of, whether the WWDT window violation 
 *  reset has occurred or not.
 * @Example
    if(WWDT_WindowViolationStatusGet())
 */
bool WWDT_WindowViolationStatusGet(void);


#endif	/* MCC_H */
/**
 End of File
 */