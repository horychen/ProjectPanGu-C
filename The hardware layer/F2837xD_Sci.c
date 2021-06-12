//###########################################################################
//
// FILE:   F2837xD_Sci.c
//
// TITLE:  F2837xD SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File


//
// Defines
//
#if CPU_FRQ_200MHZ
    //    #define CPU_FREQ        (200E6)
    //    #define LSPCLK_FREQ     (CPU_FREQ/4)
    //    #define SCI_Asynchronous_Baud 100E3 // ???
    //    #define SCI_BRR         (LSPCLK_FREQ / (SCI_Asynchronous_Baud*8)) - 1
    #define CPU_FREQ     150E6
    #define LSPCLK_FREQ  (CPU_FREQ/4.0)
    #define SCI_FREQ     115200
    #define SCI_BRR     ((LSPCLK_FREQ/(SCI_FREQ*8))-1)  // Actually it's 117,187.5Hz.
#endif


void InitSciGpio(){
   InitScicGpio();
}

void InitScicGpio(void){
    GPIO_SetupPinOptions(38, GPIO_OUTPUT, GPIO_ASYNC);//SCI_TX
    GPIO_SetupPinMux(38,1,5);//
    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_ASYNC);//SCI_RX
    GPIO_SetupPinMux(39,1,5);//

//    GPIO_SetupPinOptions(38, GPIO_OUTPUT, GPIO_ASYNC);
//    #if NUMBER_OF_DSP_CORES == 1
//        GPIO_SetupPinMux(38,GPIO_MUX_CPU1,5);
//    #else
//        GPIO_SetupPinMux(38,GPIO_MUX_CPU2,5);
//    #endif

//    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_ASYNC);
//    #if NUMBER_OF_DSP_CORES == 1
//        GPIO_SetupPinMux(39,GPIO_MUX_CPU1,5);
//    #else
//        GPIO_SetupPinMux(39,GPIO_MUX_CPU2,5);
//    #endif


}
void InitSci(void){
    CpuSysRegs.PCLKCR7.bit.SCI_C = 1; // SCI_C Clock Enable bit (Table 2-179) Peripheral Clock Gating Registers

    ScicRegs.SCICCR.all =0x0007; // 1 stop bit,  No loopback
                                 // No parity, 8 char bits,
                                 // async mode, idle-line protocol
    ScicRegs.SCICTL1.all =0x0003;// enable TX, RX, internal SCICLK,
                                 // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.bit.TXINTENA = 1;
    ScicRegs.SCICTL2.bit.RXBKINTENA =1;

    // SCI at 9600 baud @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    /* CJH: SCI_BRR = 200e6/4 / (9600*8) - 1 = 650 = 0x28B
     * 注意是波特率（SCI_FREQ）越高， BRR越小，比如 115200 Hz 对应 SCI_BRR = 53.25 = 0x35，此时SCIHBAUD.all=0x00 */
    ScicRegs.SCIHBAUD.all=0x02;
    ScicRegs.SCILBAUD.all=0x8B;
    //  ScicRegs.SCICCR.bit.LOOPBKENA =1;

    ScicRegs.SCIFFTX.all=0xC028;
    ScicRegs.SCIFFRX.bit.RXFFIL=8;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=0;
    ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;
    //    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

//    SciaRegs.SCIFFTX.all=0xE040;
//    SciaRegs.SCIFFRX.all=0x2044;

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScicRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

    ScicRegs.SCIFFRX.bit.RXFFIENA=1;






//    ScicRegs.SCICTL2.bit.TXINTENA = 1;
//    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
//    ScicRegs.SCIHBAUD.all = 0x0000;
    ScicRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
//    ScicRegs.SCIFFTX.all = 0xC022;
//    ScicRegs.SCIFFRX.all = 0x0022;
//    ScicRegs.SCIFFCT.all = 0x00;
//
//    ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
//    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;


    // scic_fifo_init();
    if(1){
//        ScicRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back
//        ScicRegs.SCIFFTX.all = 0xC022;
//        ScicRegs.SCIFFRX.all = 0x0022;
//        ScicRegs.SCIFFCT.all = 0x00;
//        ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//        ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
//        ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
    }
}
