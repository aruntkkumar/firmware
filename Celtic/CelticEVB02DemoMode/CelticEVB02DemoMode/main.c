/**
 Celtic v 1.0 for EVB02. Code to control 2 modules of SSC (4 SSCs in total)

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

const uint8_t DacOutput[4] = {43, 72, 86, 100}; // 43=0.3V,72=0.5,86=0.6,100=0.7
//All are in reverse order for SPI mode transfer
/*const uint8_t USIDLOWREV[36] = {0x4C, 0x27, 0x40, //PWR Mode: Normal
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
                                0x4C, 0x30, 0x54}; //State 10 (0101000) [F1 F2 F3 F4 F5 F6 F7]
const uint8_t USIDHIGHREV[33]= {0x5C, 0x07, 0x40,
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
const uint8_t USIDLOW1[44] =    {0x01, 0x32, 0xE4, 0x02, //PWR Mode: Normal
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
const   uint8_t SLAVEADD[2] =  {0x06, 0x0E};   //USID LOW,USID HIGH
const   uint8_t REGADD[2]   =  {0x07, 0x10};   //NORMAL POWER-UP ADDRESS,REGISTER ADDRESS
const   uint8_t DATA[65]    =  {0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,        //REVERSED DATA 
                                0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
                                0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
                                0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
                                0x06}; //STATE64 (F6&F7 ON)
const   uint8_t PARITY[65]  =  {0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 
                                0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 
                                0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 
                                0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 
                                0x01}; //STATE64 (F6&F7 ON)
const uint8_t ACTIVEBAND[64]=  {0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
                                0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
                                1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0};    // Defined bands 5, 8, 12, 13, 20, 26, 29

uint8_t Byte1, Byte2, Byte3; // First 3 bytes from RFFE
uint8_t parity1, parity2, parity3, parity4;
//uint8_t readDummy;

/*void MIPIDATA (uint8_t i){                // Need to enable SPI module to use the same
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDLOW[j]);
    for (uint8_t j=0; j<4; j++)
                readDummy = SPI_Exchange8bit(USIDLOW[(i+j)]);
}*/

/*void MIPITRANSFER(uint8_t k) {
    SDO1_SetHigh();
    asm("NOP");
    SDO1_SetLow();
    readDummy = USIDLOWREV[0];
    readDummy = readDummy >> 1;
    for (uint8_t j = 0; j < 7; j++) {
        SCK_SetHigh();
        SDO1_LAT = readDummy & 0x01;
        asm("NOP");
        SCK_SetLow();
        readDummy = readDummy >> 1;
    }
    for (uint8_t i = 1; i < 3; i++) {
        readDummy = USIDLOWREV[i];
        for (uint8_t j = 0; j < 8; j++) {
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
    for (uint8_t j = 0; j < 7; j++) {
        SCK_SetHigh();
        SDO1_LAT = readDummy & 0x01;
        asm("NOP");
        SCK_SetLow();
        readDummy = readDummy >> 1;
    }
    for (uint8_t i = 1; i < 3; i++) {
        readDummy = USIDLOWREV[(k + i)];
        for (uint8_t j = 0; j < 8; j++) {
            SCK_SetHigh();
            SDO1_LAT = readDummy & 0x01;
            asm("NOP");
            SCK_SetLow();
            readDummy = readDummy >> 1;
        }
    }
}*/

void shiftRegister (uint8_t Dummy1, uint8_t Dummy2, uint8_t y){
    for (uint8_t m=0; m<y; m++){
        SCK_SetHigh();
        SDO1_LAT = Dummy1 & 0x01;
        SDO2_LAT = Dummy2 & 0x01;
        asm("NOP");
        SCK_SetLow();
        Dummy1 = Dummy1 >> 1;
        Dummy2 = Dummy2 >> 1;        
    }
}

uint8_t paritycheck (uint8_t Dummy){
    bool parity = 0;
    while (Dummy)           //while (Dummy!=0)
    {
        parity = !parity;
        Dummy = Dummy & (Dummy - 1);
    }        
    if (parity) return 0;
    else        return 1;
}

void MIPI (uint8_t a, uint8_t b, uint8_t c, uint8_t d){
    //parity1 = paritycheck(DATA[a]);
    //parity2 = paritycheck(DATA[b]);
    //parity3 = paritycheck(DATA[c]);
    //parity4 = paritycheck(DATA[d]);
    for (uint8_t i=0; i<2; i++){          
        SDO1_SetHigh();
        SDO2_SetHigh();
        asm("NOP");
        SDO1_SetLow();
        SDO2_SetLow();
        shiftRegister (SLAVEADD[i],SLAVEADD[i],4);
        shiftRegister (0x02,0x02,3);
        shiftRegister (REGADD[0],REGADD[0],5);
        if (i==0)   
            shiftRegister (0x01,0x01,1);     //For USID = 0110, Parity = 1
        else        
            shiftRegister (0x00,0x00,1);     //For USID = 0111, Parity = 0 
        shiftRegister (0x00,0x00,8);    //F1-F7 OFF
        shiftRegister (0x01,0x01,2);    //Parity + Bus Park
        asm("NOP");
        asm("NOP");
                 
        SDO1_SetHigh();
        SDO2_SetHigh();
        asm("NOP");
        SDO1_SetLow();
        SDO2_SetLow();
        shiftRegister (SLAVEADD[i],SLAVEADD[i],4);
        shiftRegister (0x02,0x02,3);
        shiftRegister (REGADD[1],REGADD[1],5);
        if (i==0)   
            shiftRegister (0x01,0x01,1);     //For USID = 0110, Parity = 1
        else        
            shiftRegister (0x00,0x00,1);     //For USID = 0111, Parity = 0       
        if (i==0){
                shiftRegister (DATA[a],DATA[c],8);          //SSC1 & SSC3
                //shiftRegister (parity1,parity3,2);          //Parity + Bus Park
                shiftRegister (PARITY[a],PARITY[c],2);          //Parity + Bus Park
        }else{
                shiftRegister (DATA[b],DATA[d],8);          //SSC2 & SSC4
                //shiftRegister (parity2,parity4,2);          //Parity + Bus Park
                shiftRegister (PARITY[b],PARITY[d],2);          //Parity + Bus Park
        }
        asm("NOP");
        asm("NOP");
    }
}

void ConfigureShiftRegister(void){
    uint8_t ReverseReg, i;
    /*if ((Byte1 & 0x03) == 0x00)             // SSC1+SSC2 (RF1,RF4)
        ReverseReg = 0x30;                  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF1,RF4
    else if ((Byte1 & 0x03) == 0x01)        // SSC1+Passive (RF1,RF3)
        ReverseReg = 0x10;                  // CTLA1 = L, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF1,RF3
    else if ((Byte1 & 0x03) == 0x02)        // Passive+SSC2 (RF2,RF4)
        ReverseReg = 0xB0;                  // CTLA1 = H, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF2,RF4
    else                                    // Passive+Passive (RF2,RF3)
        ReverseReg = 0x90;                  // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF2,RF3   
     */
    if (Byte1 == 0xDC){
        if ((Byte2 == 2 && Byte3 == 0) || (Byte2 == 12 && Byte3 == 0) || (Byte2 == 13 && Byte3 == 0)|| (Byte2 == 20 && Byte3 == 0))
            ReverseReg = 0x30;                  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF1,RF4
        //else if ((Byte2 == 12 && Byte3 == 0) || (Byte2 == 13 & Byte3 == 0))
        //    ReverseReg = 0x10;                 // CTLA1 = L, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF1,RF3
        else
            ReverseReg = 0x90;                  // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF2,RF3
    }
    else if (Byte1 ==0xD8){
        if ((Byte2 == 29 && Byte3 == 0) || (Byte2 == 2 && Byte3 == 12) || (Byte2 == 4 && Byte3 == 12) || (Byte2 == 4 && Byte3 == 13))
            ReverseReg = 0x30;                  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF1,RF4
        //else if (Byte2 == 29 && Byte3 == 0)
        //    ReverseReg = 0x10;                 // CTLA1 = L, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF1,RF3
        else
            ReverseReg = 0x90;                  // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF2,RF3
    } else{
            ReverseReg = 0x90;                 // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF2,RF3 
    }      
    /*// Pick out RF3 case
    if (Byte2 == 3 && Byte3 == 7){
        ReverseReg = 0x50; // CTL1 = L, CTLB = H
    }
    // Pick out RF2 cases
    else if ((Byte2 == 7  && Byte3 == 0) ||
             (Byte2 == 30 && Byte3 == 0) ||
             (Byte2 == 41 && Byte3 == 0) ||
             (Byte2 == 5  && Byte3 == 30) ||
             (Byte2 == 7  && Byte3 == 20) ||
             (Byte2 == 12 && Byte3 == 30) ||
             (Byte2 == 41 && Byte3 == 41)
            ){
        ReverseReg = 0x90; // CTL1 = H, CTLB = L
    }
    // Must be RF1
    else{
        ReverseReg = 0x10; // CTL1 = L, CTLB = L
    }*/
    /*
    if ((Byte2 == 1 && Byte3 == 0) || (Byte2 == 2 && Byte3 == 0) || (Byte2 == 12 && Byte3 == 0) || (Byte2 == 2 && Byte3 == 12) || (Byte2 == 2 && Byte3 == 13) || (Byte2 == 2 && Byte3 == 29))
        ReverseReg = 0x00;  // CTLA1 = L, CTLB1 = L, CTLA2 = L, CTLB2 = L. i.e. RF1,RF1
    else if ((Byte2 == 5 && Byte3 == 0) || (Byte2 == 8 && Byte3 == 0) || (Byte2 == 29 && Byte3 == 0))
        ReverseReg = 0x20;  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = L. i.e. RF1,RF2
    else if ((Byte2 == 3 && Byte3 == 0) || (Byte2 == 13 && Byte3 == 0) || (Byte2 == 20 && Byte3 == 0) || (Byte2 == 25 && Byte3 == 0))
        ReverseReg = 0x10;  // CTLA1 = L, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF1,RF3
    //else if ((Byte2 == 4 && Byte3 == 0) || (Byte2 == 26 && Byte3 == 0) || (Byte2 == 1 && Byte3 == 8) || (Byte2 == 2 && Byte3 == 5))
    //    ReverseReg = 0x30;  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF1,RF4
    
    else if ((Byte2 == 30 && Byte3 == 0) || (Byte2 == 41 && Byte3 == 41))
        ReverseReg = 0x80;  // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = L. i.e. RF2,RF1
    else if ((Byte2 == 7 && Byte3 == 0))
        ReverseReg = 0xA0;  // CTLA1 = H, CTLB1 = L, CTLA2 = H, CTLB2 = L. i.e. RF2,RF2
    else if ((Byte2 == 41 && Byte3 == 0))
        ReverseReg = 0x90;  // CTLA1 = H, CTLB1 = L, CTLA2 = L, CTLB2 = H. i.e. RF2,RF3
    else if ((Byte2 == 12 && Byte3 == 30))
        ReverseReg = 0xB0;  // CTLA1 = H, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF2,RF4
    
    else if ((Byte2 == 3 && Byte3 == 7))
        ReverseReg = 0x40;  // CTLA1 = L, CTLB1 = H, CTLA2 = L, CTLB2 = L. i.e. RF3,RF1
    else if ((Byte2 == 3 && Byte3 == 20))
        ReverseReg = 0x60;  // CTLA1 = L, CTLB1 = H, CTLA2 = H, CTLB2 = L. i.e. RF3,RF2
    else if ((Byte2 == 4 && Byte3 == 5))
        ReverseReg = 0x50;  // CTLA1 = L, CTLB1 = H, CTLA2 = L, CTLB2 = H. i.e. RF3,RF3
    else if ((Byte2 == 4 && Byte3 == 12))
        ReverseReg = 0x70;  // CTLA1 = L, CTLB1 = H, CTLA2 = H, CTLB2 = H. i.e. RF3,RF4

    else if ((Byte2 == 4 && Byte3 == 13))
        ReverseReg = 0xC0;  // CTLA1 = H, CTLB1 = H, CTLA2 = L, CTLB2 = L. i.e. RF4,RF1
    else if ((Byte2 == 4 && Byte3 == 29))
        ReverseReg = 0xE0;  // CTLA1 = H, CTLB1 = H, CTLA2 = H, CTLB2 = L. i.e. RF4,RF2
    else if ((Byte2 == 5 && Byte3 == 30))
        ReverseReg = 0xD0;  // CTLA1 = H, CTLB1 = H, CTLA2 = L, CTLB2 = H. i.e. RF4,RF3
    else if ((Byte2 == 7 && Byte3 == 20))
        ReverseReg = 0xF0;  // CTLA1 = H, CTLB1 = H, CTLA2 = H, CTLB2 = H. i.e. RF4,RF4
    
    else    
        ReverseReg = 0x30;  // CTLA1 = L, CTLB1 = L, CTLA2 = H, CTLB2 = H. i.e. RF1,RF4 (Default including commented values)
    */
    // Start with additional clock cycle
    PE_D_LAT = 0;           // Should already be zero
    PE_CLK_SetHigh();          // Clock is left low so start with high
    asm("NOP");
    PE_CLK_SetLow();           // This shifts data into register (due to inversion)
    
    for (i=0; i<8; i++) {
        PE_D_LAT = ReverseReg & 0x01;
        PE_CLK_SetHigh();
        asm("NOP");
        PE_CLK_SetLow();       // This shifts data into register (due to inversion)
        ReverseReg = ReverseReg >> 1;
    }
    
    // Additional clock cycle to load into output register
    PE_D_LAT = 0;
    PE_CLK_SetHigh();
    asm("NOP");
    PE_CLK_SetLow();             

}

void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    DAC1_Initialize();
    PIN_MANAGER_Initialize();
    //    SPI_Initialize();
    //DAC1_SetOutput(0);
    //CTLA_SetLow(); //RF1 ON
    //CTLB_SetLow();
    PE_OE_SetLow();
    PE_D_SetLow();
    PE_CLK_SetLow();
    MAIN_NIC_LDO_EN_SetLow(); //0: OFF / MCM sleep mode
    AUX_NIC_LDO_EN_SetLow();
    while (1) {
        while ((PORTC & 0xC0) != 0xC0) {
        }
        Byte1 = PORTC; //Sequence starts for Bits 7&6=1&1
        while ((PORTC & 0xC0) != 0x40) {
        }
        Byte2 = PORTC & 0x3F; //Sequence when 7&6 bits toggles (0 1)
        while ((PORTC & 0xC0) != 0x80) {
        }
        Byte3 = PORTC & 0x3F; //Sequence when 7&6 bits toggles (1 0)
        /*if (((Byte1 & 0x10) == 0x10) && ((Byte1 & 0x0C) != 0x00)) //ANT_SEL=01/11 & TRX_SEL>0 (DAC Stage)
            DAC1_SetOutput(DacOutput[(Byte1 & 0x03)]); //DAC Output is disabled
        else
            DAC1_SetOutput(0);*/
        // Main NIC LDO & Aux NIC LDO Power Enable Sequence
        if ((Byte1 & 0x3C) == 0x20) {
            MAIN_NIC_LDO_EN_SetLow(); //Main_NIC_LDO is Low
            AUX_NIC_LDO_EN_SetHigh(); //Aux_NIC_LDO is High        
        } else {
            if (((Byte1 & 0x30) != 0x00) && ((Byte1 & 0x0C) != 0x00)) {
                AUX_NIC_LDO_EN_SetHigh();
                if (ACTIVEBAND[Byte2] || ACTIVEBAND[Byte3])
                    MAIN_NIC_LDO_EN_SetHigh();
                else
                    MAIN_NIC_LDO_EN_SetLow();
                /*if ((Byte2 == 5) || (Byte2 == 8) || (Byte2 == 12) || (Byte2 == 13) || (Byte2 == 20) || (Byte2 == 26) || (Byte2 == 29) || (Byte3 == 5) || (Byte3 == 8) || (Byte3 == 12) || (Byte3 == 13) || (Byte3 == 20) || (Byte3 == 26) || (Byte3 == 29)) {
                    MAIN_NIC_LDO_EN_SetHigh(); //3: Two antennas selected
                    AUX_NIC_LDO_EN_SetHigh();
                } else {
                    MAIN_NIC_LDO_EN_SetLow(); //2: Aux antenna selected
                    AUX_NIC_LDO_EN_SetHigh();
                }*/
            } else {
                MAIN_NIC_LDO_EN_SetLow(); //0: OFF / MCM sleep mode
                AUX_NIC_LDO_EN_SetLow();
            }
        }
        /*if (Byte2 == 3 && Byte3 == 7) {
            CTLA_SetLow(); // RF3 is ON
            CTLB_SetHigh();
        } else {
            if ((Byte2 == 7 && Byte3 == 0) || (Byte2 == 30 && Byte3 == 0) || (Byte2 == 41 && Byte3 == 0) || (Byte2 == 5 && Byte3 == 30) || (Byte2 == 7 && Byte3 == 20) || (Byte2 == 12 && Byte3 == 30) || (Byte2 == 41 && Byte3 == 41)) {
                CTLA_SetHigh(); // RF2 is ON
                CTLB_SetLow();
            } else {
                CTLA_SetLow(); // RF1 is ON
                CTLB_SetLow();
            }
        }*/
        //    MIPIDATA (4);
        //    MIPIDATA (8);
        // MIPI Clock and Data Generation (SSC1,SSC2,SSC3,SSC4)        
        /*if ((Byte1 & 0x0C) == 0x00)
            MIPI(Byte2,Byte2,Byte3,Byte3);
        else if ((Byte1 & 0x0C) == 0x04)
            MIPI(64,64,Byte3,Byte3);
        else if ((Byte1 & 0x0C) == 0x08)
            MIPI(Byte2,Byte2,64,64);
        else                                //if ((Byte1 & 0x0C) == 0x0C)
            MIPI(64,64,64,64);*/
        if (Byte2 == 2 && Byte3 == 0)       //Band 2
            MIPI(10,10,32,32);
        else if (Byte2 == 29 && Byte3 == 0) //Band 29
            MIPI(48,48,63,63);
        else if (Byte2 == 12 && Byte3 == 0) //Band 12
            MIPI(37,37,61,61);
        else if (Byte2 == 13 && Byte3 == 0) //Band 13
            MIPI(21,21,32,32);
        else if (Byte2 == 20 && Byte3 == 0) //Band 20
            MIPI(9,9,1,1);
        else if (Byte2 == 2 && Byte3 == 12) //Band 2+12
            MIPI(24,24,5,5);
        else if ((Byte2 == 4 && Byte3 == 12) || (Byte2 == 4 && Byte3 == 13)) //Band 13
            MIPI(17,17,1,1);
        else    MIPI(0,0,0,0);        
        /*
        if (Byte2 == 5 && Byte3 == 0)       //RF1,RF2
            MIPI(63,63,63,63);
        else if (Byte2 == 8 && Byte3 == 0)  //RF1,RF2
            MIPI(10,64,0,0);
        else if (Byte2 == 12 && Byte3 == 0) //RF1,RF1
            MIPI(0,0,10,64);
        else if (Byte2 == 13 && Byte3 == 0) //RF1,RF3
            MIPI(12,30,45,55);
        else if (Byte2 == 20 && Byte3 == 0) //RF1,RF3
            MIPI(21,21,21,21);
        else if (Byte2 == 26 && Byte3 == 0) //RF1,RF4
            MIPI(1,1,1,1);
        else if (Byte2 == 29 && Byte3 == 0) //RF1,RF2
            MIPI(5,10,15,20);
        else if (Byte2 == 1 && Byte3 == 8)  //RF1,RF4
            MIPI(15,18,21,36);
        else if (Byte2 == 2 && Byte3 == 5)  //RF1,RF4
            MIPI(20,20,20,20);
        else if (Byte2 == 2 && Byte3 == 12) //RF1,RF1
            MIPI(46,1,15,64);
        else if (Byte2 == 2 && Byte3 == 13) //RF1,RF1
            MIPI(5,20,55,0);
        else if (Byte2 == 2 && Byte3 == 29) //RF1,RF1
            MIPI(7,9,35,60);
        else if (Byte2 == 3 && Byte3 == 20) //RF3,RF2
            MIPI(44,55,33,11);
        else if (Byte2 == 4 && Byte3 == 5)  //RF3,RF3
            MIPI(23,34,45,56);
        else if (Byte2 == 4 && Byte3 == 12) //RF3,RF4
            MIPI(10,20,30,40);
        else if (Byte2 == 4 && Byte3 == 13) //RF4,RF1
            MIPI(20,30,40,50);
        else if (Byte2 == 4 && Byte3 == 29) //RF4,RF2
            MIPI(32,21,10,55);
        else if (Byte2 == 5 && Byte3 == 30) //RF4,RF3
            MIPI(12,24,36,48);
        else if (Byte2 == 7 && Byte3 == 20) //RF4,RF4
            MIPI(9,18,27,36);
        else if (Byte2 == 12 && Byte3 == 30)//RF2,RF4
            MIPI(44,44,44,44);
        else
            MIPI(0,0,0,0);
        */
        /*if ((Byte2 == 5) && (Byte3 == 0)) {
            MIPI(10,64,0,0);
            //    MIPITRANSFER(33);
            //    MIPITRANSFER(3);
        } else {
            if ((Byte2 == 5) || (Byte3 == 5)) {
                //    MIPITRANSFER(3);
            } else {
                if ((Byte2 == 8) || (Byte3 == 8)) {
                    //    MIPITRANSFER(6);
                } else {
                    if ((Byte2 == 12) || (Byte3 == 12)) {
                        //    MIPITRANSFER(9);
                    } else {
                        if ((Byte2 == 13) || (Byte3 == 13)) {
                            //    MIPITRANSFER(12);
                        } else {
                            if ((Byte2 == 20) || (Byte3 == 20)) {
                                //    MIPITRANSFER(15);
                            } else {
                                if ((Byte2 == 26) || (Byte3 == 26)) {
                                    //    MIPITRANSFER(18);
                                } else {
                                    if ((Byte2 == 29) || (Byte3 == 29)) {
                                        //    MIPITRANSFER(21);
                                    } else {
                                        //    MIPITRANSFER(30);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }*/
        // RF Switch Selection Sequence
        PE_OE_SetHigh();             //To test with EVB01 with has Shift Register
        ConfigureShiftRegister();
        PE_OE_SetLow();
    }
}
