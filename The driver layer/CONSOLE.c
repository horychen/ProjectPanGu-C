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
Uint16 sdataA[2] = {0x22, 0x23};    // Send data for SCI-A
Uint16 rdataA[2];    // Received data for SCI-A
Uint16 rdata_pointA; // Used for checking the received data

__interrupt void scicTxFifoIsr(void){
    Uint16 i;

    for(i=0; i< 2; i++)
    {
       ScicRegs.SCITXBUF.all=sdataA[i];  // Send data
    }

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


void sci_poll(int16 ch){
    static int set_current_flag = FALSE;

    static int dac_offset_flag  = FALSE;
    static int dac_channel_flag = FALSE;

    static int reverse_speed_flag = FALSE;
    static int reverse_current_flag = FALSE;

    if( set_current_flag || dac_offset_flag || dac_channel_flag ){
        if(set_current_flag){
            switch(ch){
                case ' ': set_current_flag = FALSE; break;
                case '+': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit += 0.1; break;
                case '-': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit -= 0.1; break;
                case '1': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  = 0.1; break;
                case '2': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =   2; break;
                case '3': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =   3; break;
                case '4': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =   4; break;
                case '5': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =   5; break;
                case '6': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =  -5; break;
                case '7': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =  -4; break;
                case '8': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =  -3; break;
                case '9': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =  -2; break;
                case '0': G.sendCurrentCommandFlag = TRUE; pid1_spd.OutLimit  =  -1; break;
              default : break;
            }
        }
    }
    else // No Flag is set
    {
        switch(ch)
        {
            /* Default mode is to change speed. */
            case '1':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = -500;
                break;
            case '2':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = -300;
                break;
            case '3':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 50;
                break;
            case '4':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 100;
                break;
            case '5':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 300;
                break;
            case '6':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 500;
                break;
            case '7':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 800;
                break;
            case '8':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 1500;
                break;
            case '9':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 2000;
                break;
            case '0':
                G.sendSpeedCommandFlag = TRUE; G.Set_manual_rpm = 0;
                break;
            case '-':
                reverse_speed_flag = TRUE; // not used
                reverse_current_flag = TRUE; // not used
                break;

            /* Change torque current */
            case 'c':
                set_current_flag = TRUE;
                G.sendCurrentCommandFlag = TRUE;
                break;

            /* Stop */
            case 's':
            case 'S':
                break;

            /* Run */
            case 'r':
            case 'R':
                G.sendSpeedCommandFlag = TRUE;
                break;

            default :
                /*Do Nothing Or Test Respond*/;
                //send_flag = 999;
                break;
        }
    }


}




/*
 * else if(dac_channel_flag){
    switch(ch)
    {
      case '+': dac_channel += 1; send_flag=5; break;
      case '-': dac_channel -= 1; send_flag=5; break;
      case ' ': dac_channel_flag=False; break;
      default : break;
    }
}else if(dac_offset_flag){
    if(dac_channel>8){
          dac_channel=8;
    }else if(dac_channel<1){
          dac_channel=1;
    }
    switch(ch)
    {
      case '+': dac_offset[dac_channel-1] += 1; send_flag=55; break;
      case '-': dac_offset[dac_channel-1] -= 1; send_flag=55; break;
      case ' ': dac_offset_flag=False; break;
      default : break;
    }
}
*/
