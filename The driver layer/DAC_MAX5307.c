#include "All_Definition.h"

void DAC_MAX5307(int channel_number, REAL dac_value)
{
    // effective range for dac_value is in [-1, 1]//=-3V-3V for scope
    int32 b = (dac_value + 1.0)*2048;

    // Offset
    //b += dac_offset[a-1];

    // 限幅
    if(b>4095){
        b = 4095;
    }
    else if(b<0){
        b = 0;
    }

    // debug
    //b = 2048;

    GpioDataRegs.GPBSET.bit.GPIO57 = 1;//SPICS
    NOP;
    NOP;
    GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;

    switch(channel_number)
    {
        case 1 :
            SpiaRegs.SPITXBUF=b|0x2000;                 //DAC_A 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 2 :
            SpiaRegs.SPITXBUF=b|0x3000;                 //DAC_B 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 3 :
            SpiaRegs.SPITXBUF=b|0x4000;                 //DAC_C 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 4 :
            SpiaRegs.SPITXBUF=b|0x5000;                 //DAC_D 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 5 :
            SpiaRegs.SPITXBUF=b|0x6000;                 //DAC_E 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 6 :
            SpiaRegs.SPITXBUF=b|0x7000;                 //DAC_F 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 7 :
            SpiaRegs.SPITXBUF=b|0x8000;                 //DAC_G 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        case 8 :
            SpiaRegs.SPITXBUF=b|0x9000;                 //DAC_H 通道
            while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){}    //!=1 ==0
            break;
        default :
            NOP;
            break;
    }

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    NOP;
    NOP;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    GpioDataRegs.GPBSET.bit.GPIO57 = 1;
    NOP;
    NOP;
    GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;

    SpiaRegs.SPITXBUF=0xEFF0;

    while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){NOP;}

    SpiaRegs.SPICCR.bit.SPISWRESET=0;
    NOP;
    NOP;
    SpiaRegs.SPICCR.bit.SPISWRESET=1;

}

