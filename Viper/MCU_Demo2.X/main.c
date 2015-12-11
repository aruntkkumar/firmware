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

#define SSC_AC                           0
#define SSC_BD                           1

// Defined bands 5, 8, 12, 13, 20, 26, 29
const uint8_t ACTIVEBAND[64] =
{0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0};

// Define DAC levels 0.3v, 0.5v, 0.6v, 0.7v assuming VDD as reference
const uint8_t DACLEVEL[4] = {43, 71, 85, 100};

// The two supported 4-bit slave addresses (in reverse bit order)
const uint8_t SLAVEADDRESS[2] = {0x0E, 0x06};

// The value sent for normal operation (in reverse bit order)
#define NORMALOPERATION     0x3A
// The value sent for write register 1 (in reverse bit order)
#define WRITEREGISTER1      0x82

// The address/command parity for the two supported slave addresses
// NB parity is the same for normal operation and write register 1
const uint8_t COMMANDPARITY[2] = {0x00, 0x01};

// The possible values for write register 1 commands (in reverse bit order)
const uint8_t VALUE[9] = {0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02};

// The corresponding parity values
const uint8_t PARITY[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



uint8_t Byte1, Byte2, Byte3;
uint8_t AntSel, TrxSel, PwrLvl, BandSel, CABandSel;

void Fix_Initialize(void){

    ODCONA = 0x00;                 // standard push-pull drive on output port pins
    ODCONB = 0x00;                 // standard push-pull drive on output port pins

    WPUC = 0xFF;                   // weak pull-ups enabled on input pins
    OPTION_REGbits.nWPUEN = 0x00;  // change to the generated code   
}

void EnableLDOs(void){
    if (TrxSel == 0 && AntSel == 2){
        MAIN_NIC_LDO_EN_SetLow();
        AUX_NIC_LDO_EN_SetHigh();
    }
    else if (TrxSel > 0 && AntSel > 0) {
        AUX_NIC_LDO_EN_SetHigh();
        if (ACTIVEBAND[BandSel] || ACTIVEBAND[CABandSel]){
            MAIN_NIC_LDO_EN_SetHigh();
        }
        else {
            MAIN_NIC_LDO_EN_SetLow();
        }
    }
    else {
        MAIN_NIC_LDO_EN_SetLow();
        AUX_NIC_LDO_EN_SetLow();
    }
}

void GenerateDACOut(void){
    if (AntSel == 1 && TrxSel == 1){
        DAC1_SetOutput(DACLEVEL[PwrLvl]);
    }
    else {
        DAC1_SetOutput(0);
    }
}

void ConfigureShiftRegister(void){
    uint8_t ReverseReg, i;
    
    // Pick out RF3 case
    if (BandSel == 3 && CABandSel == 7){
        ReverseReg = 0x40; // CTL1 = L, CTLB = H
    }
    // Pick out RF2 cases
    else if ((BandSel == 7  && CABandSel == 0) ||
             (BandSel == 30 && CABandSel == 0) ||
             (BandSel == 41 && CABandSel == 0) ||
             (BandSel == 5  && CABandSel == 30) ||
             (BandSel == 7  && CABandSel == 20) ||
             (BandSel == 12 && CABandSel == 30) ||
             (BandSel == 41 && CABandSel == 41)
            ){
        ReverseReg = 0x80; // CTL1 = H, CTLB = L
    }
    // Must be RF1
    else{
        ReverseReg = 0x00; // CTL1 = L, CTLB = L
    }
    
    // Start with additional clock cycle
    PE_D_LAT = 0;                    // Should already be zero
    CLK_GLOBAL_SetHigh();            // Clock is left low so start with high
    asm("NOP");
    CLK_GLOBAL_SetLow();             // This shifts data into register (due to inversion)
    
    for (i=0; i<8; i++) {
        PE_D_LAT = ReverseReg & 0x01;
        CLK_GLOBAL_SetHigh();
        asm("NOP");
        CLK_GLOBAL_SetLow();        // This shifts data into register (due to inversion)
        ReverseReg = ReverseReg >> 1;
    }
    
    // Additional clock cycle to load into output register
    PE_D_LAT = 0;
    CLK_GLOBAL_SetHigh();
    asm("NOP");
    CLK_GLOBAL_SetLow();             

}

void SendSlaveAddress(uint8_t Id){
    uint8_t ReverseReg, i;
    
    ReverseReg = SLAVEADDRESS[Id];
    
    // Send 4 bit slave address
    for (i=0; i<4; i++) {
        SPI_MUX_DATA_LAT = ReverseReg & 0x01;
        CLK_GLOBAL_SetHigh();
        asm("NOP");
        CLK_GLOBAL_SetLow();        // This clocks data the data out
        ReverseReg = ReverseReg >> 1;
    }
}

void SendByte(uint8_t ReverseValue){
    uint8_t i;
    
    // Send 8 bit value
    for (i=0; i<8; i++) {
        SPI_MUX_DATA_LAT = ReverseValue & 0x01;
        CLK_GLOBAL_SetHigh();
        asm("NOP");
        CLK_GLOBAL_SetLow();        // This clocks data the data out
        ReverseValue = ReverseValue >> 1;
    }
}

void SendBit(uint8_t BitValue){
    
    // Send bit value
    SPI_MUX_DATA_LAT = BitValue & 0x01;
    CLK_GLOBAL_SetHigh();
    asm("NOP");
    CLK_GLOBAL_SetLow();        // This clocks data the data out
}

void ConfigureSSC(uint8_t Id, uint8_t ValueId){
    
    // Send Sequence StartCondition
    SPI_MUX_DATA_SetHigh();
    asm("NOP");
    SPI_MUX_DATA_SetLow();
    
    SendSlaveAddress(Id);
    
    // Send Normal Operation Command
    SendByte(NORMALOPERATION);
    SendBit(COMMANDPARITY[Id]);
    
    // Send Value 0
    SendByte(VALUE[0]);
    SendBit(PARITY[0]);
    
    // Park Bus
    SendBit(0);
    
    asm("NOP");
    asm("NOP");
    
    // Now send write register 1
    
    // Send Sequence StartCondition
    SPI_MUX_DATA_SetHigh();
    asm("NOP");
    SPI_MUX_DATA_SetLow();
    
    SendSlaveAddress(Id);
    
    // Send Write Register 1 Command
    SendByte(WRITEREGISTER1);
    SendBit(COMMANDPARITY[Id]);
    
    // Send Value 0
    SendByte(VALUE[ValueId]);
    SendBit(PARITY[ValueId]);
    
    // Park Bus
    SendBit(0);
}

void main(void){
    uint8_t ValueId;
    
    SYSTEM_Initialize();  
    Fix_Initialize();       // Corrections to auto-generated code
    
    while (1){
        // Add your application code
        
        // Read the parallel input until we get the first byte
        Byte1 = 0;
        while ((Byte1 & 0xC0) != 0xC0) {
            Byte1 = PORTC;  // Read the parallel input from the LM8335
        }
        // Read the parallel input until we get the second byte
        Byte2 = 0;
        while ((Byte2 & 0xC0) != 0x40) {
            Byte2 = PORTC;  // Read the parallel input from the LM8335
        }
        // Read the parallel input until we get the third byte
        Byte3 = 0;
        while ((Byte3 & 0xC0) != 0x80) {
            Byte3 = PORTC;  // Read the parallel input from the LM8335
        }
        
        // Extract the register values
        AntSel = (Byte1 & 0x30) >> 4;
        TrxSel = (Byte1 & 0x0C) >> 2;
        PwrLvl = Byte1 & 0x03;
        BandSel = Byte2 & 0x3F;
        CABandSel = Byte3 & 0x3F;
        
        // Enable the LDOs
        EnableLDOs();
        
        // Generate DAC_OUT voltage
        GenerateDACOut();
        
        PE_OE_SetHigh();                    // Disable shift register
        
        // Configure SSCA and SSCB
        
        if (AntSel == 0 || AntSel == 2 ||
            TrxSel == 0 || TrxSel == 3){

            ValueId = 0;  // ValueId 0 corresponds to F1-F7 OFF
        }
        // The following logic is made up and will need replacing 
        // with real logic when defined
        
        else if (BandSel == 5 || CABandSel == 5){
            ValueId = 1;
        }
        else if (BandSel == 8 || CABandSel == 8){
            ValueId = 2;
        }
        else if (BandSel == 12 || CABandSel == 12){
            ValueId = 3;
        }
        else if (BandSel == 13 || CABandSel == 13){
            ValueId = 4;
        }
        else if (BandSel == 20 || CABandSel == 20){
            ValueId = 5;
        }
        else if (BandSel == 26 || CABandSel == 26){
            ValueId = 6;
        }
        else if (BandSel == 29 || CABandSel == 29){
            ValueId = 7;
        }
        else {
            ValueId = 0;
        }
        
        SPI_SEL_SetHigh();
        ConfigureSSC(SSC_AC, ValueId);
        ConfigureSSC(SSC_BD, ValueId);
        
        // Configure SSCC and SSCD
        
        SPI_SEL_Toggle();
        ConfigureSSC(SSC_AC, ValueId);
        ConfigureSSC(SSC_BD, ValueId);
        
        // Configure the shift register outputs
        ConfigureShiftRegister();
        
        PE_OE_SetLow();                     // Enable shift register
    }
}
/**
 End of File
 */