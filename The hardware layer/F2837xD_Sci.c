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

void InitSciGpio(){
   InitScicGpio();
}

void InitScicGpio(void){
    GPIO_SetupPinOptions(38, GPIO_OUTPUT, GPIO_SYNC);//SCI_TX
    GPIO_SetupPinMux(38,1,5);//
    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_SYNC);//SCI_RX
    GPIO_SetupPinMux(39,1,5);//
}
void InitSci(void){
    ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                 // No parity,8 char bits,
                                 // async mode, idle-line protocol
    ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                 // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.bit.TXINTENA = 1;  //SCTTXBUF ENA
    ScicRegs.SCICTL2.bit.RXBKINTENA =1;
    ScicRegs.SCIHBAUD.all=0x0002;//0x0000;  // 9600 baud @LSPCLK = 37.5MHz.  //8-15reserved, 0x01E7
    ScicRegs.SCILBAUD.all=0x008B;//0x0028;
    //  ScicRegs.SCICCR.bit.LOOPBKENA =1;

    ScicRegs.SCIFFTX.all=0xC028;
    ScicRegs.SCIFFRX.bit.RXFFIL=8;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=0;
    ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;
    //    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    ScicRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
    ScicRegs.SCIFFRX.bit.RXFFIENA=1;
}

//
// End of file
//
