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

#include "mcc_generated_files/mcc.h"

const uint8_t DacOutput[4] = {43,72,86,100}; // 43=0.3V,72=0.5,86=0.6,100=0.7
//All are in reverse order for SPI mode transfer
const uint8_t USIDLOWREV[36] =    {0x4C, 0x27, 0x40, //PWR Mode: Normal
                                   0x4C, 0xB0, 0x41, //F6-F7 ON
                                   0x4C, 0x30, 0x7F, //F1-F6 ON
                                   0x4C, 0xB0, 0x00, //F7 ON
                                   0x4C, 0x30, 0x01, //F6 ON
                                   0x4C, 0x30, 0x02, //F5 ON
                                   0x4C, 0x30, 0x04, //F4 ON
                                   0x4C, 0x30, 0x08, //F3 ON
                                   0x4C, 0x30, 0x10, //F2 ON
                                   0x4C, 0x30, 0x20, //F1 ON
                                   0x4C, 0x30, 0x40, //F1-F7 OFF
                                   0x4C, 0x30, 0x54};//State 10 (0101000) [F1 F2 F3 F4 F5 F6 F7]
const uint8_t USIDHIGHREV[33] =   {0x5C, 0x07, 0x40, 
                                   0x5C, 0x90, 0x41, 
                                   0x5C, 0x10, 0x7F, 
                                   0x5C, 0x90, 0x00, 
                                   0x5C, 0x10, 0x01, 
                                   0x5C, 0x10, 0x02, 
                                   0x5C, 0x10, 0x04, 
                                   0x5C, 0x10, 0x08, 
                                   0x5C, 0x10, 0x10, 
                                   0x5C, 0x10, 0x20, 
                                   0x5C, 0x10, 0x40};
/*const uint8_t USIDLOW1[44] =    {0x01, 0x32, 0xE4, 0x02, //PWR Mode: Normal
                                0x01, 0x32, 0x0D, 0x82, //F6-F7 ON
                                0x01, 0x32, 0x0C, 0xFE, //F1-F6 ON
                                0x01, 0x32, 0x0D, 0x00, //F7 ON
                                0x01, 0x32, 0x0C, 0x80, //F6 ON
                                0x01, 0x32, 0x0C, 0x40, //F5 ON
                                0x01, 0x32, 0x0C, 0x20, //F4 ON
                                0x01, 0x32, 0x0C, 0x10, //F3 ON
                                0x01, 0x32, 0x0C, 0x08, //F2 ON
                                0x01, 0x32, 0x0C, 0x04, //F1 ON
                                0x01, 0x32, 0x0C, 0x02};//F1-F7 OFF
const uint8_t USIDHIGH1[44] =   {0x01, 0x3A, 0xE0, 0x02, 
                                0x01, 0x3A, 0x09, 0x82, 
                                0x01, 0x3A, 0x08, 0xFE, 
                                0x01, 0x3A, 0x09, 0x00, 
                                0x01, 0x3A, 0x08, 0x80, 
                                0x01, 0x3A, 0x08, 0x40, 
                                0x01, 0x3A, 0x08, 0x20, 
                                0x01, 0x3A, 0x08, 0x10, 
                                0x01, 0x3A, 0x08, 0x08, 
                                0x01, 0x3A, 0x08, 0x04, 
                                0x01, 0x3A, 0x08, 0x02};*/
uint8_t Byte1, Byte2, Byte3;                // First 3 bytes from RFFE
uint8_t readDummy;

/*void MIPIDATA (uint8_t i){                // Need to enable SPI module to use the same
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDLOW[j]);
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDLOW[(i+j)]);
}*/

void MIPITRANSFER1 (uint8_t k){
        SDO1_SetHigh();
        asm("NOP");
        SDO1_SetLow();
        readDummy = USIDLOWREV[0];
        readDummy = readDummy >> 1;
        for (uint8_t j=0; j<7; j++){
            SCK_SetHigh();
            SDO1_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        for (uint8_t i=1; i<3; i++){
            readDummy = USIDLOWREV[i];
            for (uint8_t j=0; j<8; j++){
            SCK_SetHigh();
            SDO1_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        }
        asm("NOP");
        asm("NOP");
        SDO1_SetHigh();
        asm("NOP");
        SDO1_SetLow();
        readDummy = USIDLOWREV[k];
        readDummy = readDummy >> 1;
        for (uint8_t j=0; j<7; j++){
            SCK_SetHigh();
            SDO1_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        for (uint8_t i=1; i<3; i++){
            readDummy = USIDLOWREV[(k+i)];
            for (uint8_t j=0; j<8; j++){
            SCK_SetHigh();
            SDO1_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        }
}

void MIPITRANSFER2 (uint8_t k){
        SDO2_SetHigh();
        asm("NOP");
        SDO2_SetLow();
        readDummy = USIDHIGHREV[0];
        readDummy = readDummy >> 1;
        for (uint8_t j=0; j<7; j++){
            SCK_SetHigh();
            SDO2_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        for (uint8_t i=1; i<3; i++){
            readDummy = USIDHIGHREV[i];
            for (uint8_t j=0; j<8; j++){
            SCK_SetHigh();
            SDO2_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        }
        asm("NOP");
        asm("NOP");
        SDO2_SetHigh();
        asm("NOP");
        SDO2_SetLow();
        readDummy = USIDHIGHREV[k];
        readDummy = readDummy >> 1;
        for (uint8_t j=0; j<7; j++){
            SCK_SetHigh();
            SDO2_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        for (uint8_t i=1; i<3; i++){
            readDummy = USIDHIGHREV[(k+i)];
            for (uint8_t j=0; j<8; j++){
            SCK_SetHigh();
            SDO2_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
            }
        }
}

void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    DAC1_Initialize();
    PIN_MANAGER_Initialize();  
//    SPI_Initialize();
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
    // Main NIC LDO & Aux NIC LDO Power Enable Sequence
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
        }    
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
//    MIPIDATA (4);
//    MIPIDATA (8);
    // MIPI Clock and Data Generation
    if ((Byte2 == 5) && (Byte3 == 0)){
        MIPITRANSFER1 (33);
        MIPITRANSFER2 (3);
    }else{      if ((Byte2 == 5) || (Byte3 == 5)){
        MIPITRANSFER1 (3); 
    }else{      if ((Byte2 == 8) || (Byte3 == 8)){  
        MIPITRANSFER1 (6);
    }else{      if ((Byte2 == 12) || (Byte3 == 12)){   
        MIPITRANSFER1 (9);
    }else{      if ((Byte2 == 13) || (Byte3 == 13)){ 
        MIPITRANSFER1 (12);
    }else{      if ((Byte2 == 20) || (Byte3 == 20)){
        MIPITRANSFER1 (15);
    }else{      if ((Byte2 == 26) || (Byte3 == 26)){
        MIPITRANSFER1 (18);
    }else{      if ((Byte2 == 29) || (Byte3 == 29)){ 
        MIPITRANSFER1 (21);
    }else{
        MIPITRANSFER1 (30);
    }        
    }
    }
    }
    }
    }
    }
    }
    }
}
   