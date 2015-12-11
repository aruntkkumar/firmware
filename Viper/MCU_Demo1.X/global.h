/**
  Global Header File

  @Company
    Microchip Technology Inc.

  @File Name
    global.h

  @Summary
    This is the header file for the variables declarations and function prototypes

  @Description
    This header file provides all the variables and function prototypes used in this project
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

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

/**
  Section: Macro Declarations
 */
#define UP                  1
#define DOWN                0
#define PRESSED             1
#define NOT_PRESSED         0
#define RUNNING             1
#define NOT_RUNNING         0

/**
  Section: Data Types Definitions
 */
// Switch
    unsigned char labNumber = 0;
    unsigned char switchEvent = 0;
    unsigned char labState = NOT_RUNNING;
    unsigned char btnState = NOT_PRESSED;

// Lab02 - Blink
    unsigned char counter = 0;

// Lab03 - Rotate
    unsigned char rotateReg;

// Lab04 - ADC
    unsigned int adcResult = 0;

// Lab05 - VSRotate
    unsigned int delay;

// Lab10 - High Endurance Flash
    unsigned char errorCheck;
    unsigned char counterHEF = 0;

/**
  Section: Function Prototypes
 */
    
/**
  @Summary
    Performs the Switch Function.

  @Description
    This checks the state of the switch button S1 whether it is previously
    pressed or not. If it is previously HIGH, the function will then check if
    S1 is currently being pressed. If pressed, the state is changed to LOW,
    otherwise it remains HIGH.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void Switch(void);

/**
  @Summary
    Performs the HelloWorld Lab.

  @Description
    This turns on D4 LED on the Curiosity Board.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void HelloWorld(void);

/**
  @Summary
    Performs the Blink Lab.

  @Description
    D4 blinks at a rate of approximately 1.5 seconds

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void Blink();

/**
  @Summary
    Performs the Rotate Lab.

  @Description
    LEDs rotate from right to left at a rate of 0.5s

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void Rotate(void);

/**
  @Summary
    Performs the Analog to Digital Conversion Lab.

  @Description
    The top four MSbs of the ADC are shown on the LEDs.
    Rotate the potentiometer to change the display.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void ADC(void);

/**
  @Summary
    Performs the Variable Speed Rotate Lab.

  @Description
    LEDs rotate from right to left similar to Rotate Lab with varying speed.
    Rotate the POT counterclockwise to see the LEDs shift faster.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void VSR(void);

/**
  @Summary
    Performs the Pulse Width Modulation Lab.

  @Description
    Rotating POT1 will adjust the brightness of a single LED, D7.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void PWM(void);

/**
  @Summary
    Performs the Timer1 Lab.

  @Description
    LEDs rotate from right to left, similar to Rotate Lab, at a rate of ~0.5 seconds.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void Timer1(void);

/**
  @Summary
    Performs the Interrupt Lab.

  @Description
    LEDs rotate at a constant speed every time Timer0 overflows.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void Interrupt(void);

/**
  @Summary
    Performs the Sleep-Wakeup Lab.

  @Description
    D4 and D6 is ON while the MCU is in sleep mode. After 16 seconds, the Watchdog 
    Timer will wake up the device from sleep and D5 and D7 will turn ON, while 
    D4 and D6 will turn OFF.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void SleepWakeUp(void);

/**
  @Summary
    Performs the High Endurance Flash Lab.

  @Description
    This function writes a character (0x05) in the Flash memory.Then it will read the
    stored character and then display in the LEDs.

  @Preconditions
    SYSTEM_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    none
*/
void HEF(void);

/**
  @Summary
    Performs the Delay Function.

  @Description
    This function counts instruction cycles to produce a delay of 0.5 seconds.

  @Preconditions
    None.

  @Returns
    None

  @Param
    none
*/
void Delay(void);


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */
/**
 End of File
 */