//###########################################################################
//
// FILE:   yx2837xD_Adc.c
//
// TITLE:  F2837xD Adc Support Functions.
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
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"


// AdcSetMode - Set the resolution and signalmode for a given ADC. This will
//              ensure that the correct trim is loaded.
//

//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
void ADC_initialize(void)
{
    {
        //Write ADC configurations and power up the ADC for both ADC A
            Uint16 i;

            EALLOW;

            //write configurations for ADC-A
            // External REFERENCE must be provided
            AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;

            //Set pulse positions to late
            AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

            //write configurations for ADC-B
            // External REFERENCE must be provided
            AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;
            AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;

            //Set pulse positions to late
            AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

        //  //write configurations for ADC-C
            // External REFERENCE must be provided
            AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdccRegs.ADCCTL2.bit.RESOLUTION = 0;
            AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;

            //Set pulse positions to late
            AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        //
        //  //write configurations for ADC-D
            // External REFERENCE must be provided
            AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcdRegs.ADCCTL2.bit.RESOLUTION = 0;
            AdcdRegs.ADCCTL2.bit.SIGNALMODE = 0;

            //Set pulse positions to late
            AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

            //delay for > 1ms to allow ADC time to power up
            for(i = 0; i < 1000; i++){
                asm("   RPT#255 || NOP");
            }


	EDIS;

	EALLOW;
	// VDC @ A0
		// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 0;    // SOC0 will convert pin A0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

		// CURRENT W @ A1
		// ********************************
		AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 1;    // SOC1 will convert pin A1
		AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
		AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C


		//  CURRENT U @ A2
		// ********************************
		AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 2;    // SOC2 will convert pin A2
		AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
		AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C


		//  CURRENT V @ A3
		// ********************************
		AdcaRegs.ADCSOC3CTL.bit.CHSEL     = 3;    // SOC3 will convert pin A3
		AdcaRegs.ADCSOC3CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
		AdcaRegs.ADCSOC3CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

		//  CURRENT DC BUS @ A4
		// ***************************
		AdcaRegs.ADCSOC4CTL.bit.CHSEL     = 4;    // SOC4 will convert pin A4,
		AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
		AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  IGBT_TEMP @ A5
        // ***************************
        AdcaRegs.ADCSOC5CTL.bit.CHSEL     = 5;    // SOC5 will convert pin A5,
        AdcaRegs.ADCSOC5CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcaRegs.ADCSOC5CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP3 @ B0
        // ***************************
        AdcbRegs.ADCSOC6CTL.bit.CHSEL     = 0;    // SOC5 will convert pin B0,
        AdcbRegs.ADCSOC6CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC6CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP2 @ B1
        // ***************************
        AdcbRegs.ADCSOC7CTL.bit.CHSEL     = 1;    // SOC5 will convert pin B1,
        AdcbRegs.ADCSOC7CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC7CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP5 @ B2
        // ***************************
        AdcbRegs.ADCSOC8CTL.bit.CHSEL     = 2;    // SOC5 will convert pin B2,
        AdcbRegs.ADCSOC8CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC8CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP4 @ B3
        // ***************************
        AdcbRegs.ADCSOC9CTL.bit.CHSEL     = 3;    // SOC5 will convert pin B3,
        AdcbRegs.ADCSOC9CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC9CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP7 @ B4
        // ***************************
        AdcbRegs.ADCSOC10CTL.bit.CHSEL     = 4;    // SOC5 will convert pin B4,
        AdcbRegs.ADCSOC10CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC10CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C

        //  TEMP6 @ B5
        // ***************************
        AdcbRegs.ADCSOC11CTL.bit.CHSEL     = 5;    // SOC5 will convert pin B5,
        AdcbRegs.ADCSOC11CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
        AdcbRegs.ADCSOC11CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
		EDIS;
}


  /*  void AdcSetMode(Uint16 adc, Uint16 resolution, Uint16 signalmode)
    {
        Uint16 adcOffsetTrimOTPIndex; //index into OTP table of ADC offset trims
        Uint16 adcOffsetTrim;         //temporary ADC offset trim

        //
        //re-populate INL trim
        //
        CalAdcINL(adc);

        if(0xFFFF != *((Uint16*)GetAdcOffsetTrimOTP))
        {
            //
            //offset trim function is programmed into OTP, so call it
            //

            //
            //calculate the index into OTP table of offset trims and call
            //function to return the correct offset trim
            //
            adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
            adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
        }
        else
        {
            //
            //offset trim function is not populated, so set offset trim to 0
            //
            adcOffsetTrim = 0;
        }

        //
        //Apply the resolution and signalmode to the specified ADC.
        //Also apply the offset trim and, if needed, linearity trim correction.
        //
        switch(adc)
        {
            case ADC_ADCA:
                AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
                AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
                AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
                if(ADC_RESOLUTION_12BIT == resolution)
                {
                    //
                    //12-bit linearity trim workaround
                    //
                    AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
                    AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
                    AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
                    AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
                }
            break;
            case ADC_ADCB:
                AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
                AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
                AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
                if(ADC_RESOLUTION_12BIT == resolution)
                {
                    //
                    //12-bit linearity trim workaround
                    //
                    AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
                    AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
                    AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
                    AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
                }
            break;
            case ADC_ADCC:
                AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
                AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
                AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
                if(ADC_RESOLUTION_12BIT == resolution)
                {
                    //
                    //12-bit linearity trim workaround
                    //
                    AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
                    AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
                    AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
                    AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
                }
            break;
            case ADC_ADCD:
                AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
                AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
                AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
                if(ADC_RESOLUTION_12BIT == resolution)
                {
                    //
                    //12-bit linearity trim workaround
                    //
                    AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
                    AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
                    AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
                    AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
                }
            break;
        }
    }

//
// CalAdcINL - Loads INL trim values from OTP into the trim registers of the
//             specified ADC. Use only as part of AdcSetMode function, since
//             linearity trim correction is needed for some modes.
//
void CalAdcINL(Uint16 adc)
{
    switch(adc)
    {
        case ADC_ADCA:
            if(0xFFFF != *((Uint16*)CalAdcaINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcaINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCB:
            if(0xFFFF != *((Uint16*)CalAdcbINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcbINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCC:
            if(0xFFFF != *((Uint16*)CalAdccINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdccINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCD:
            if(0xFFFF != *((Uint16*)CalAdcdINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcdINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
    }*/
}

//
// End of file
//
