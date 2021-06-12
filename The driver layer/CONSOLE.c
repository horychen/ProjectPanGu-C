/*
 * SCI_232.c
 *
 *  Created on: 2021Äê5ÔÂ15ÈÕ
 *      Author: yuanx
 */
#include <All_Definition.h>

//
// Globals
//
Uint16 sdataA[2];    // Send data for SCI-A
Uint16 rdataA[2];    // Received data for SCI-A
Uint16 rdata_pointA; // Used for checking the received data

__interrupt void scicTxFifoIsr(void){
    Uint16 i;

    for(i=0; i< 2; i++)
    {
       ScicRegs.SCITXBUF.all=sdataA[i];  // Send data
    }

    sdataA[0] = 66;
    sdataA[1] = 67;

    //    for(i=0; i< 2; i++)                  // Increment send data for next cycle
    //    {
    //       sdataA[i] = (sdataA[i]+1) & 0x00F;
    //    }

    ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8; // Issue PIE ACK for Group 8 (for SCI-C and SCI-D)
}

__interrupt void scicRxFifoIsr(void){
    Uint16 i;

    for(i=0;i<2;i++)
    {
       rdataA[i]=ScicRegs.SCIRXBUF.all;  // Read data
    }

    for(i=0;i<2;i++)                     // Check received data
    {
       if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) )
       {
           //error();
       }
    }

    rdata_pointA = (rdata_pointA+1) & 0x00FF;

    ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8; // Issue PIE ACK for Group 8 (for SCI-C and SCI-D)
}
