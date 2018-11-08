/**
    Lenovo test unit using EVB02 for Type A: SSC CXM3664XR; VDD, GND, SCLK, SDATA; Main and Aux SSC UID all connect to low.
    Engineering Mode for EVB02
    2 switches: SW1 & SW2
    2 SSCs: SSC1 & SSC2 (currently SSC2 is ignored)
    MAIN_NIC_LDO_EN & AUX_NIC_LDO_EN to control the LDOs separately
    PE to control the switches
    1 separate SDATA_MIPI line for each SSCs. SCLK is shared by each SSCs.
    Byte 2 is currently ignored; it is conserved for SW3 and SW4 (future iterations).
    
    The byte structure is as follows:
        Byte1: 1 1 SW1 SW1  SW1 SW2 SW2 SW2
        Byte2: 0 1 SW3 SW3  SW3 SW4 SW4 SW4 (currently disabled)
        Byte3: 1 0 DAC(Last 6 bits)
        Byte4: 0 1 MIPI_Device(2 bits)  DAC(First 2 bits) MNIC ANIC
        Byte5: 1 SSC1(7 bits; due to state 64 - 1000000)
        Byte6: 0 SSC2 and so on.

        MIPI_Device(2 bits):
        0 0 - None selected
        0 1 - CXM3664XR
        1 0 - CXA4484GC / CXA4484XR
        1 1 - 4-bit SSC (in development)
 
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
const   uint8_t SLAVEADD[2] =  {0x06, 0x0E};   //USID LOW,USID HIGH
const   uint8_t REGADD[2]   =  {0x07, 0x10};   //NORMAL POWER-UP ADDRESS,REGISTER ADDRESS
const   uint8_t DATA[65]    =  {0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,        //REVERSED DATA 
                                0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
                                0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
                                0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
                                0x02}; //STATE64 (F6&F7 ON)
const   uint8_t PARITY[65]  =  {0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 
                                0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 
                                0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 
                                0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 
                                0x00}; //STATE64 (F6&F7 ON)
const uint8_t ACTIVEBAND[64]=  {0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
                                0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
                                1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0};    // Defined bands 5, 8, 12, 13, 20, 26, 29
const uint8_t SLAVEWRITEADD[2]={0x64, 0x74};    // USID LOW/HIGH and C2 C1 C0 A4 (Refer SSC Datasheet)
const uint8_t ADDRESSDATA1[65]={0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, //Only for USID LOW
                                0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
                                0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
                                0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
                                0x1A};          // A3=0 A2=0 A1=0 A0=1 P=1 D7=0 D6 D5
const uint8_t ADDRESSDATA2[65]={0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, //Only for USID HIGH
                                0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
                                0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                                0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                                0x12};          // A3=0 A2=0 A1=0 A0=1 P=0 D7=0 D6 D5
const uint8_t DATABUSPARK[65]= {0x04, 0x08, 0x10, 0x1C, 0x20, 0x2C, 0x34, 0x38, 0x40, 0x4C, 0x54, 0x58, 0x64, 0x68, 0x70, 0x7C,
                                0x80, 0x8C, 0x94, 0x98, 0xA4, 0xA8, 0xB0, 0xBC, 0xC4, 0xC8, 0xD0, 0xDC, 0xE0, 0xEC, 0xF4, 0xF8,
                                0x00, 0x0C, 0x14, 0x18, 0x24, 0x28, 0x30, 0x3C, 0x44, 0x48, 0x50, 0x5C, 0x60, 0x6C, 0x74, 0x78,
                                0x84, 0x88, 0x90, 0x9C, 0xA0, 0xAC, 0xB4, 0xB8, 0xC0, 0xCC, 0xD4, 0xD8, 0xE4, 0xE8, 0xF0, 0xFC,
                                0x00};          // D4 D3 D2 D1 D0 P B 0


uint8_t Byte1, Byte2, Byte3, Byte4, Byte5, Byte6; // First 6 bytes from RFFE
uint8_t DAC_Step, Dummy, start = 0;
/*
void shiftRegister (uint8_t Dummy1, uint8_t Dummy2, uint8_t y){
    for (uint8_t m=0; m<y; m++){
        SCLK_MIPI_SetHigh();
        SDATA1_MIPI_LAT = Dummy1 & 0x01;
        SDATA2_MIPI_LAT = Dummy2 & 0x01;
        //SDO3_LAT = Dummy3 & 0x01;                                     //Enable when the 3rd SSC pin is named as SDATA3_MIPI
        //SDO4_LAT = Dummy4 & 0x01;                                     //Enable when the 4th SSC pin is named as SDATA4_MIPI
        SCLK_MIPI_SetLow();
        Dummy1 = Dummy1 >> 1;
        Dummy2 = Dummy2 >> 1;   
        //Dummy3 = Dummy3 >> 1;                                         //Enable when the 3rd SSC pin is named as SDATA3_MIPI
        //Dummy4 = Dummy4 >> 1;                                         //Enable when the 4th SSC pin is named as SDATA4_MIPI 
    }
}

void MIPI (uint8_t a, uint8_t b){          
    SDATA1_MIPI_SetHigh();
    SDATA2_MIPI_SetHigh();
    //SDO3_SetHigh();                                                   //Enable when the 3rd SSC pin is named as SDATA3_MIPI
    //SDO4_SetHigh();                                                   //Enable when the 4th SSC pin is named as SDATA4_MIPI
    SDATA1_MIPI_SetLow();
    SDATA2_MIPI_SetLow();
    //SDO3_SetLow();                                                    //Enable when the 3rd SSC pin is named as SDATA3_MIPI
    //SDO4_SetLow();                                                    //Enable when the 4th SSC pin is named as SDATA4_MIPI
    shiftRegister (SLAVEADD[0],SLAVEADD[0],4);                          //Change the SLAVEADD according to the SSC USID
    shiftRegister (0x82,0x82,8);                                        //Write Command+Register Address
    shiftRegister (0x01,0x01,1);                                        //For USID = 0110, Parity = 1 and For USID = 0111, Parity = 0; change accordingly
    shiftRegister (DATA[a],DATA[b],8);                                  //SSC1 & SSC2
    shiftRegister (PARITY[a],PARITY[b],2);                              //Parity + Bus Park
}
*/

void MIPISPI (uint8_t a, uint8_t b){                                    //WONT WORK DUE TO SPI MISSING MSB BIT
    RB6PPS = 0x00; // RB6->LATB6                                        //First SCC should have UID as Low            
    SDO_SetHigh();
    SDO_SetLow();
    RB6PPS = 0x11; // RB6->MSSP:SDO
    SPI_Exchange8bit(SLAVEWRITEADD[0]);
    SPI_Exchange8bit(ADDRESSDATA1[a]);
    SPI_Exchange8bit(DATABUSPARK[a]);
    RB6PPS = 0x00; // RB6->LATB6                                        //Second SCC should have UID as High              
    SDO_SetHigh();
    SDO_SetLow();
    RB6PPS = 0x11; // RB6->MSSP:SDO
    SPI_Exchange8bit(SLAVEWRITEADD[1]);
    SPI_Exchange8bit(ADDRESSDATA2[b]);
    SPI_Exchange8bit(DATABUSPARK[b]);    
}

void MIPISPI1 (uint8_t a){
    RB6PPS = 0x00; // RB6->LATB6                                        //Single SCC with UID as Low
    SDO_SetLow();
    SDO_SetHigh();
    SDO_SetLow();
    RB6PPS = 0x11; // RB6->MSSP:SDO
    SPI_Exchange8bit(SLAVEWRITEADD[0]);                   
    if (a == 0x60){
        SPI_Exchange8bit(0x1B);                                         //For USID = High, it is 0x13                                                 
        SPI_Exchange8bit(0x04);                                         //Same for USID = High                         
    }
    else{
        SPI_Exchange8bit(ADDRESSDATA1[a]);                                                
        SPI_Exchange8bit(DATABUSPARK[a]);                            
    }
//    //Read the register address
//    RB6PPS = 0x00; // RB6->LATB6                                        //Single SCC with UID as Low
//    SDO_SetLow();
//    SDO_SetHigh();
//    SDO_SetLow();
//    RB6PPS = 0x11; // RB6->MSSP:SDO
//    SPI_Exchange8bit(0x66);
//    Dummy = SPI_Exchange8bit(0x10);
//    SPI_Exchange8bit(Dummy);
}

/*
void ConfigureShiftRegister(void){
    uint8_t ReverseReg = 0x00, i;   
    
    // RF SW1
    Dummy = Byte1 & 0x38;
    if (Dummy == 0x00)
        ReverseReg |= 0x00;
    else if (Dummy == 0x20)
        ReverseReg |= 0x80;
    else if (Dummy == 0x10)
        ReverseReg |= 0x40;
    else if (Dummy == 0x30)
        ReverseReg |= 0xC0;
    // RF SW2
    Dummy = Byte1 & 0x07;
    if (Dummy == 0x00)
        ReverseReg |= 0x00;
    else if (Dummy == 0x04)
        ReverseReg |= 0x20;
    else if (Dummy == 0x02)
        ReverseReg |= 0x10; 
    else if (Dummy == 0x06)
        ReverseReg |= 0x30;    
    
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
*/
void main(void) {
    //if (INTCONbits.IOCIF)
    //goto reset;
    if (start == 1) goto reset;
    // initialize the device
    SYSTEM_Initialize();
    //DAC1_SetOutput(0);      //DAC already initialised to ZERO
    //PE_OE_SetLow();
    //PE_D_SetLow();
    //PE_CLK_SetLow();
    MAIN_NIC_LDO_EN_SetLow(); //0: OFF / MCM sleep mode
    SW1_CTL0_SetLow();
    SW1_CTL1_SetLow();
    SW1_CTL2_SetLow();
    CTLA_SW2_SetLow();
    CTLB_SW2_SetLow();
    reset:
    start = 1;
    if (IOCCFbits.IOCCF7 == 0) {     //To enable SLEEP mode after powering up
        SLEEP();}
    else    {    
        IOCCFbits.IOCCF7 = 0;}
    while (1) {
        //First Byte
        while ((PORTC & 0xC0) != 0xC0) {
        }
        //__delay_ms(5);
        Byte1 = PORTC;                                              //Sequence starts for Bits 7&6=1&1       
        //Second Byte (Ignored the second byte, which is for SW3 & SW4)
        while ((PORTC & 0xC0) != 0x40) {
        }
        //__delay_ms(5);
        Byte2 = PORTC & 0x3F;                                       //Sequence starts for Bits 7&6=1&1             
        //Third Byte 
        while ((PORTC & 0xC0) != 0x80) {
        }
        //__delay_ms(5);
        Byte3 = PORTC & 0x3F;                                       //Sequence when 7&6 bits toggles (1 0). DAC(Last 6 bits)
        //Fourth Byte
        while ((PORTC & 0xC0) != 0x40) {
        }
        //__delay_ms(5);
        Byte4 = PORTC & 0x3F;                                       //Sequence when 7&6 bits toggles (0 1). X X DAC(First 2 bits) MAIN_NIC_LDO_EN AUX_NIC_LDO_EN
        //Fifth Byte
        while ((PORTC & 0x80) != 0x80) {
        }
        //__delay_ms(5);
        Byte5 = PORTC & 0x7F;                                       //Sequence when Bit 7 toggles (1). SSC1 (7 bits; in case of state 64: 1000000)
        //Sixth Byte (Currently disabled)
        //while ((PORTC & 0x80) != 0x00) {
        //}
        //Byte6 = PORTC & 0x7F;                                       //Sequence when Bit 7 toggles (0). SSC2
        
//        // DAC Output Selection Sequence
//        DAC_Step = ((Byte4 << 4) & 0xC0);
//        DAC_Step = (DAC_Step | Byte3);
//        DAC1_SetOutput(DAC_Step);
//        
//        // Main NIC LDO & Aux NIC LDO Power Enable Sequence
//        if ((Byte4 & 0x02) == 0x02)
//            MAIN_NIC_LDO_EN_SetHigh();
//        else
//            MAIN_NIC_LDO_EN_SetLow();
//        //if ((Byte4 & 0x01) == 0x01)   //Aux NIC disabled
//        //    AUX_NIC_LDO_EN_SetHigh();
//        //else
//        //    AUX_NIC_LDO_EN_SetLow();
//        
//        // MIPI Clock and Data Generation (SSC1,SSC2), Add more SSC values to function in case of more SSCs        
//        //MIPI(Byte5, Byte6);
//        //MIPISPI(Byte5, Byte6);
        if ((Byte4 & 0x30) == 0x10)
            MIPISPI1(Byte5);

        // RF Switch Selection Sequence
        /*PE_OE_SetHigh();
        ConfigureShiftRegister();
        PE_OE_SetLow();*/
        // RF SW1
        switch ((Byte1 & 0x38)){
            case 0x00:
                SW1_CTL0_SetHigh();
                SW1_CTL1_SetHigh();
                SW1_CTL2_SetHigh();
                break;
            case 0x08:
                SW1_CTL0_SetLow();
                SW1_CTL1_SetLow();
                SW1_CTL2_SetLow();
                break;
            case 0x10:
                SW1_CTL0_SetHigh();
                SW1_CTL1_SetLow();
                SW1_CTL2_SetLow();
                break;
            case 0x18:
                SW1_CTL0_SetLow();
                SW1_CTL1_SetHigh();
                SW1_CTL2_SetLow();
                break;
            case 0x20:
                SW1_CTL0_SetHigh();
                SW1_CTL1_SetHigh();
                SW1_CTL2_SetLow();
                break;
            case 0x28:
                SW1_CTL0_SetLow();
                SW1_CTL1_SetLow();
                SW1_CTL2_SetHigh();
                break;
            case 0x30:
                SW1_CTL0_SetHigh();
                SW1_CTL1_SetLow();
                SW1_CTL2_SetHigh();
                break;
            case 0x38:
                SW1_CTL0_SetLow();
                SW1_CTL1_SetHigh();
                SW1_CTL2_SetHigh();
                break;
            default:
                SW1_CTL0_SetLow();
                SW1_CTL1_SetLow();
                SW1_CTL2_SetLow();
                break;
        }         
//        // RF SW2
//        switch ((Byte1 & 0x07)){
//            case 0x01:
//                CTLA_SW2_SetLow();
//                CTLB_SW2_SetLow();
//                break;
//            case 0x02:
//                CTLA_SW2_SetHigh();
//                CTLB_SW2_SetLow(); 
//                break;
//            case 0x03:
//                CTLA_SW2_SetLow();
//                CTLB_SW2_SetHigh();
//                break;
//            case 0x04:
//                CTLA_SW2_SetHigh();
//                CTLB_SW2_SetHigh();
//                break;
//            default:
//                break;
//        }
//        
        SLEEP();
    }
}
