//###########################################################################
//
// FILE:   yx2837xD_EPwm.c
//
// TITLE:  F2837xD EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//

#include <All_Definition.h>


volatile struct EPWM_REGS *ePWM[] = {
        &EPwm1Regs,         //intentional: (ePWM[0] not used)
        &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
        &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 carrier_period, int16 db_cnt) {
    EALLOW;
    // Time Base SubModule Registers
    (*ePWM[n]).TBCTL.bit.PRDLD = TB_SHADOW ; // set Immediate load
    (*ePWM[n]).TBPRD = carrier_period / 2; // divide by 2 because of TB_COUNT_UPDOWN.
    (*ePWM[n]).TBPHS.bit.TBPHS = 0;
    (*ePWM[n]).TBCTR = 0;
    (*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    // (*ePWM[n]).TBCTL.bit.HSPCLKDIV = 0b111;
    // (*ePWM[n]).TBCTL.bit.CLKDIV    = 0b111;
    (*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
    (*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;


    // TBCLK = EPWMCLK / (HSPCLKDIV * CLKDIV) = 100 MHz / (1*1)
    //    TBCLK Time-base clock.
    //    This is a prescaled version of the EPWM clock (EPWMCLK) and is used by all submodules within the ePWM.
    //    This clock determines the rate at which time-base counter increments or decrements.

    (*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
    (*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_CMPB; // sync "down-stream"

    // Counter Compare Submodule Registers
    (*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially
    (*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    // Action Qualifier SubModule Registers
    (*ePWM[n]).AQCTLA.bit.CAU =AQ_CLEAR; //SYSTEM_PWM_AQ_CLEAR;// ;
    (*ePWM[n]).AQCTLA.bit.CAD =AQ_SET; //SYSTEM_PWM_AQ_SET;// ;
        //    #define SYSTEM_PWM_AQ_SET                 0x2     //AQ_SET;    OTHERWISE, REVERSE!
        //    #define SYSTEM_PWM_AQ_CLEAR               0x1     //AQ_CLEAR;

    // Active high complementary PWMs - Set up the deadband
    (*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
    (*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    (*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
        //    #define SYSTEM_PWM_DB_ACTV                0x2     //DB_ACTV_LOC  0x1   DB_ACTV_HIC  0x2 // LOW EFFECT /HIGH EFFECT
    (*ePWM[n]).DBRED.all = db_cnt;
    (*ePWM[n]).DBFED.all = db_cnt; // 这么多个TBCLK的时间，作为死区时间
    EDIS;
}

//
// InitEPwmGpio - Initialize all EPWM modules' GPIOs
//
void InitEPwmGpio(void)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm9Gpio();
    InitEPwm10Gpio();
    InitEPwm11Gpio();
    InitEPwm12Gpio();
}

//
// InitEPwm1Gpio - Initialize EPWM1 GPIOs
//
void InitEPwm1Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPEPUD.bit.GPIO145 = 1;    // Disable pull-up on GPIO145 (EPWM1A)
    GpioCtrlRegs.GPEPUD.bit.GPIO146 = 1;    // Disable pull-up on GPIO146 (EPWM1B)

    //
    // Configure EPWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM1 functional
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPEMUX2.bit.GPIO145 = 1;   // Configure GPIO145 as EPWM1A
    GpioCtrlRegs.GPEMUX2.bit.GPIO146 = 1;   // Configure GPIO146 as EPWM1B

    EDIS;
}

//
// InitEPwm2Gpio - Initialize EPWM2 GPIOs
//
void InitEPwm2Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPEPUD.bit.GPIO147 = 1;    // Disable pull-up on GPIO147 (EPWM2A)
    GpioCtrlRegs.GPEPUD.bit.GPIO148 = 1;    // Disable pull-up on GPIO148 (EPWM2B)

    //
    // Configure EPwm-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM2 functional pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    GpioCtrlRegs.GPEMUX2.bit.GPIO147 = 1;   // Configure GPIO147 as EPWM2A
    GpioCtrlRegs.GPEMUX2.bit.GPIO148 = 1;   // Configure GPIO148 as EPWM2B

    EDIS;
}

//
// InitEPwm3Gpio - Initialize EPWM3 GPIOs
//
void InitEPwm3Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    //   for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    // GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPEPUD.bit.GPIO149 = 1;    // Disable pull-up on GPIO149 (EPWM3A)
    GpioCtrlRegs.GPEPUD.bit.GPIO150 = 1;    // Disable pull-up on GPIO150 (EPWM3B)

    //
    // Configure EPwm-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM3 functional pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    // GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
    GpioCtrlRegs.GPEMUX2.bit.GPIO149 = 1;   // Configure GPIO149 as EPWM3A
    GpioCtrlRegs.GPEMUX2.bit.GPIO150 = 1;   // Configure GPIO150 as EPWM3B

    EDIS;
}

//
// InitEPwm4Gpio - Initialize EPWM4 GPIOs
//
void InitEPwm4Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    //   for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    // GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;      // Disable pull-up on GPIO6 (EPWM4A)
    // GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;      // Disable pull-up on GPIO7 (EPWM4B)
    GpioCtrlRegs.GPEPUD.bit.GPIO151 = 1;       // Disable pull-up on GPIO151 (EPWM4A)
    GpioCtrlRegs.GPEPUD.bit.GPIO152 = 1;       // Disable pull-up on GPIO152 (EPWM4B)

     //
     // Configure EPWM-4 pins using GPIO regs
     // This specifies which of the possible GPIO pins will be EPWM4 functional
     // pins.
     // Comment out other unwanted lines.
     //
     //  GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
     //  GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
    GpioCtrlRegs.GPEMUX2.bit.GPIO151 = 1;      // Configure GPIO151 as EPWM4A
    GpioCtrlRegs.GPEMUX2.bit.GPIO152 = 1;      // Configure GPIO152 as EPWM4B


    EDIS;
}

//
// InitEPwm5Gpio - Initialize EPWM5 GPIOs
//
void InitEPwm5Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    //   for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //  GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;     // Disable pull-up on GPIO8 (EPWM5A)
    //  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;     // Disable pull-up on GPIO9 (EPWM5B)
    GpioCtrlRegs.GPEPUD.bit.GPIO153 = 1;       // Disable pull-up on GPIO153 (EPWM5A)
    GpioCtrlRegs.GPEPUD.bit.GPIO154 = 1;       // Disable pull-up on GPIO154 (EPWM5B)


    //
    // Configure EPWM-5 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM5 functional
    // pins.
    // Comment out other unwanted lines.
    //
    //  GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;    // Configure GPIO8 as EPWM5A
    //  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;    // Configure GPIO9 as EPWM5B
    GpioCtrlRegs.GPEMUX2.bit.GPIO153 = 1;      // Configure GPIO153 as EPWM5A
    GpioCtrlRegs.GPEMUX2.bit.GPIO154 = 1;      // Configure GPIO154 as EPWM5B


    EDIS;
}

//
// InitEPwm6Gpio - Initialize EPWM6 GPIOs
//
void InitEPwm6Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    // GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;     // Disable pull-up on GPIO10 (EPWM6A)
    // GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;     // Disable pull-up on GPIO11 (EPWM6B)
     GpioCtrlRegs.GPEPUD.bit.GPIO155 = 1;    // Disable pull-up on GPIO155 (EPWM6A)
     GpioCtrlRegs.GPEPUD.bit.GPIO156 = 1;    // Disable pull-up on GPIO156 (EPWM6B)


    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    //  GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    //  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
     GpioCtrlRegs.GPEMUX2.bit.GPIO155 = 1;   // Configure GPIO155 as EPWM6A
     GpioCtrlRegs.GPEMUX2.bit.GPIO156 = 1;   // Configure GPIO156 as EPWM6B

    EDIS;
}

//
// InitEPwm7Gpio - Initialize EPWM7 GPIOs
//
void InitEPwm7Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;    // Disable pull-up on GPIO12 (EPWM7A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;    // Disable pull-up on GPIO13 (EPWM7B)
     GpioCtrlRegs.GPEPUD.bit.GPIO157 = 1;    // Disable pull-up on GPIO157 (EPWM7A)
     GpioCtrlRegs.GPEPUD.bit.GPIO158 = 1;    // Disable pull-up on GPIO158 (EPWM7B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;   // Configure GPIO13 as EPWM7B
     GpioCtrlRegs.GPEMUX2.bit.GPIO157 = 1;   // Configure GPIO157 as EPWM7A
     GpioCtrlRegs.GPEMUX2.bit.GPIO158 = 1;   // Configure GPIO158 as EPWM7B

    EDIS;
}

//
// InitEPwm8Gpio - Initialize EPWM8 GPIOs
//
void InitEPwm8Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up on GPIO14 (EPWM8A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    // Disable pull-up on GPIO15 (EPWM8B)
      GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;    // Disable pull-up on GPIO159 (EPWM8A)
      GpioCtrlRegs.GPFPUD.bit.GPIO160 = 1;    // Disable pull-up on GPIO160 (EPWM8B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
     GpioCtrlRegs.GPEMUX2.bit.GPIO159 = 1;   // Configure GPIO159 as EPWM8A
     GpioCtrlRegs.GPFMUX1.bit.GPIO160 = 1;   // Configure GPIO160 as EPWM8B

    EDIS;
}

//
// InitEPwm9Gpio - Initialize EPWM9 GPIOs
//
void InitEPwm9Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFPUD.bit.GPIO161 = 1;    // Disable pull-up on GPIO161 (EPWM9A)
    GpioCtrlRegs.GPFPUD.bit.GPIO162 = 1;    // Disable pull-up on GPIO162 (EPWM9B)


    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFMUX1.bit.GPIO161 = 1;   // Configure GPIO161 as EPWM9A
    GpioCtrlRegs.GPFMUX1.bit.GPIO162 = 1;   // Configure GPIO162 as EPWM9B

    EDIS;
}

//
// InitEPwm10Gpio - Initialize EPWM10 GPIOs
//
void InitEPwm10Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFPUD.bit.GPIO163 = 1;    // Disable pull-up on GPIO163 (EPWM10A)
    GpioCtrlRegs.GPFPUD.bit.GPIO164 = 1;    // Disable pull-up on GPIO164 (EPWM10B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFMUX1.bit.GPIO163 = 1;   // Configure GPIO163 as EPWM10A
    GpioCtrlRegs.GPFMUX1.bit.GPIO164 = 1;   // Configure GPIO164 as EPWM10B

    EDIS;
}

//
// InitEPwm11Gpio - Initialize EPWM11 GPIOs
//
void InitEPwm11Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFPUD.bit.GPIO165 = 1;    // Disable pull-up on GPIO165 (EPWM11A)
    GpioCtrlRegs.GPFPUD.bit.GPIO166 = 1;    // Disable pull-up on GPIO166 (EPWM11B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFMUX1.bit.GPIO165 = 1;   // Configure GPIO165 as EPWM11A
    GpioCtrlRegs.GPFMUX1.bit.GPIO166 = 1;   // Configure GPIO166 as EPWM11B

    EDIS;
}

//
// InitEPwm12Gpio - Initialize EPWM12 GPIOs
//
void InitEPwm12Gpio(void)
{
    EALLOW;
    //
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFPUD.bit.GPIO167 = 1;    // Disable pull-up on GPIO167 (EPWM12A)
    GpioCtrlRegs.GPFPUD.bit.GPIO168 = 1;    // Disable pull-up on GPIO168 (EPWM12B)

    //
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM6 functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPFMUX1.bit.GPIO167 = 1;   // Configure GPIO167 as EPWM12A
    GpioCtrlRegs.GPFMUX1.bit.GPIO168 = 1;   // Configure GPIO168 as EPWM12B

    EDIS;
}
void ePWM_initialize(void)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm9Gpio();
    InitEPwm10Gpio();
    InitEPwm11Gpio();
    InitEPwm12Gpio();

    EALLOW;
    /* PLLSYSCLK / EPWMCLKDIV = EPWMCLK
        * 200 MHz / 2 = 100 MHz
        * Default: EPWMCLK = SYSCLKOUT / 2.
        */
        ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;
        PWM_1ch_UpDwnCnt_CNF(1,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);   //10k, 5us   10000,500
        PWM_1ch_UpDwnCnt_CNF(2,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(3,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(4,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(5,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(6,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);

        PWM_1ch_UpDwnCnt_CNF(7,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);   //10k, 5us   10000,500
        PWM_1ch_UpDwnCnt_CNF(8,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(9,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(10,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(11,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);
        PWM_1ch_UpDwnCnt_CNF(12,SYSTEM_CARRIER_PERIOD,SYSTEM_PWM_DEADTIME_CNT);

    /*-- Setup Event-Trigger (ET) Submodule --*/
        // Event-Trigger Selection and Event-Trigger Pre-Scale Register
        EPwm1Regs.ETSEL.all = (EPwm1Regs.ETSEL.all & ~0xFF0F) | 0x9909;
        EPwm1Regs.ETPS.all = (EPwm1Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm2Regs.ETSEL.all = (EPwm2Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm2Regs.ETPS.all = (EPwm2Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm3Regs.ETSEL.all = (EPwm3Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm3Regs.ETPS.all = (EPwm3Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm4Regs.ETSEL.all = (EPwm4Regs.ETSEL.all & ~0xFF0F) | 0x9909;
        EPwm4Regs.ETPS.all = (EPwm4Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm5Regs.ETSEL.all = (EPwm5Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm5Regs.ETPS.all = (EPwm5Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm6Regs.ETSEL.all = (EPwm6Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm6Regs.ETPS.all = (EPwm6Regs.ETPS.all & ~0x3303) | 0x1101;

        EPwm7Regs.ETSEL.all = (EPwm7Regs.ETSEL.all & ~0xFF0F) | 0x9909;
        EPwm7Regs.ETPS.all = (EPwm7Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm8Regs.ETSEL.all = (EPwm8Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm8Regs.ETPS.all = (EPwm8Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm9Regs.ETSEL.all = (EPwm9Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm9Regs.ETPS.all = (EPwm9Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm10Regs.ETSEL.all = (EPwm10Regs.ETSEL.all & ~0xFF0F) | 0x9909;
        EPwm10Regs.ETPS.all = (EPwm10Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm11Regs.ETSEL.all = (EPwm11Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm11Regs.ETPS.all = (EPwm11Regs.ETPS.all & ~0x3303) | 0x1101;
        EPwm12Regs.ETSEL.all = (EPwm12Regs.ETSEL.all & ~0xFF0F) | 0x9901;
        EPwm12Regs.ETPS.all = (EPwm12Regs.ETPS.all & ~0x3303) | 0x1101;
    EDIS;
}
