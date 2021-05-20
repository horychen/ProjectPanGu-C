/*
 * ECaptureVoltage.c
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */
#include <All_Definition.h>

struct TestECapture ecapU, ecapV, ecapW;
int flag_nonlinear_filtering = FALSE;
int flag_bad_U_capture = FALSE;
int flag_bad_V_capture = FALSE;
int flag_bad_W_capture = FALSE;
extern Uint32 good_capture_U[4];
extern Uint32 good_capture_V[4];
extern Uint32 good_capture_W[4];
#define USE_ECAP_CEVT2_INTERRUPT 1
void do_enhanced_capture(){    // Section 16.6.4 // The 32 bit counter is stored in ECap1Regs.TSCTR
    /*UVW*/
    #if USE_ECAP_CEVT2_INTERRUPT == 1
        ecapU.DutyOnTime1  = good_capture_U[1];
        ecapU.DutyOffTime1 = good_capture_U[2];
        ecapU.DutyOnTime2  = good_capture_U[3];
        ecapU.DutyOffTime2 = good_capture_U[0];
        ecapV.DutyOnTime1  = good_capture_V[1];
        ecapV.DutyOffTime1 = good_capture_V[2];
        ecapV.DutyOnTime2  = good_capture_V[3];
        ecapV.DutyOffTime2 = good_capture_V[0];
        ecapW.DutyOnTime1  = good_capture_W[1];
        ecapW.DutyOffTime1 = good_capture_W[2];
        ecapW.DutyOnTime2  = good_capture_W[3];
        ecapW.DutyOffTime2 = good_capture_W[0];
    #else
        ecapU.DutyOnTime1  = ECap1Regs.CAP2;
        ecapU.DutyOffTime1 = ECap1Regs.CAP3;
        ecapU.DutyOnTime2  = ECap1Regs.CAP4;
        ecapU.DutyOffTime2 = ECap1Regs.CAP1;
        ecapV.DutyOnTime1  = ECap2Regs.CAP2;
        ecapV.DutyOffTime1 = ECap2Regs.CAP3;
        ecapV.DutyOnTime2  = ECap2Regs.CAP4;
        ecapV.DutyOffTime2 = ECap2Regs.CAP1;
        ecapW.DutyOnTime1  = ECap3Regs.CAP2;
        ecapW.DutyOffTime1 = ECap3Regs.CAP3;
        ecapW.DutyOnTime2  = ECap3Regs.CAP4;
        ecapW.DutyOffTime2 = ECap3Regs.CAP1;
    #endif
    ecapU.Period1 = ecapU.DutyOnTime1 + ecapU.DutyOffTime1;
    ecapU.Period2 = ecapU.DutyOnTime2 + ecapU.DutyOffTime2;
    ecapV.Period1 = ecapV.DutyOnTime1 + ecapV.DutyOffTime1;
    ecapV.Period2 = ecapV.DutyOnTime2 + ecapV.DutyOffTime2;
    ecapW.Period1 = ecapW.DutyOnTime1 + ecapW.DutyOffTime1;
    ecapW.Period2 = ecapW.DutyOnTime2 + ecapW.DutyOffTime2;

    /* Nonlinear Filtering of the disturbance */
    flag_bad_U_capture = FALSE;
    flag_bad_V_capture = FALSE;
    flag_bad_W_capture = FALSE;
    if(flag_nonlinear_filtering){
        /*U 注意Uiint32不能做减法（非常容易出错）！*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapU.DutyOnTime1 - (double)ecapU.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapU.DutyOnTime2 - (double)ecapU.DutyOffTime2) > 4444.0
          ){
            flag_bad_U_capture = TRUE;
        }

        /*V*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapV.DutyOnTime1 - (double)ecapV.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapV.DutyOnTime2 - (double)ecapV.DutyOffTime2) > 4444.0
          ){
            flag_bad_V_capture = TRUE;
        }

        /*W*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapW.DutyOnTime1 - (double)ecapW.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)ecapW.DutyOnTime2 - (double)ecapW.DutyOffTime2) > 4444.0
          ){
            flag_bad_W_capture = TRUE;
        }

        if(!flag_bad_U_capture){
            CTRL.I->ecap_terminal_DutyOnRatio[0] = 0.5*(ecapU.DutyOnTime1 + ecapU.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
            CTRL.I->ecap_pwm_time = 0.5*(ecapU.Period1 + ecapU.Period2);
        }
        if(!flag_bad_V_capture){
            CTRL.I->ecap_terminal_DutyOnRatio[1] = 0.5*(ecapV.DutyOnTime1 + ecapV.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        }
        if(!flag_bad_W_capture){
            CTRL.I->ecap_terminal_DutyOnRatio[2] = 0.5*(ecapW.DutyOnTime1 + ecapW.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        }

    }else{

        //    TODO: accuracy can be further improve by replacing SYSTEM_PWM_MAX_COUNT_INVERSE with actual period measured? No, the period is not accurate as the duty ratio is chaning.
        /*API to global variable CTRL with two point moving average*/
        CTRL.I->ecap_terminal_DutyOnRatio[0] = 0.5*(ecapU.DutyOnTime1 + ecapU.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        CTRL.I->ecap_terminal_DutyOnRatio[1] = 0.5*(ecapV.DutyOnTime1 + ecapV.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        CTRL.I->ecap_terminal_DutyOnRatio[2] = 0.5*(ecapW.DutyOnTime1 + ecapW.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE; // / (0.5*(ecapW.Period1+ecapW.Period2))
        CTRL.I->ecap_pwm_time = 0.5*(ecapU.Period1 + ecapU.Period2);
    }
}

// From: D:\ti\controlSUITE\device_support\F2837xS\v210\F2837xS_examples_Cpu1\ecap_capture_pwm\cpu01\ECap_Capture_Pwm_cpu01.c
// InitECapture - Initialize ECAP1 configurations
void InitECapture()
{

    /*ECAP 1*/
    ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    //ECap1Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt
    ECap1Regs.ECEINT.bit.CEVT2 = 1;         // 2 events = __interrupt


    /*ECAP 2*/
    ECap2Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
    ECap2Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ECap2Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
    ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap2Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    //ECap2Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt
    ECap2Regs.ECEINT.bit.CEVT2 = 1;         // 2 events = __interrupt

    /*ECAP 3*/
    ECap3Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
    ECap3Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ECap3Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    ECap3Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
    ECap3Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap3Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    //ECap3Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt
    ECap3Regs.ECEINT.bit.CEVT2 = 1;         // 2 events = __interrupt
}

// ECCTL1 (ECAP Control Reg 1)
//==========================
// CAPxPOL bits
#define EC_RISING 0x0
#define EC_FALLING 0x1
// CTRRSTx bits
#define EC_ABS_MODE 0x0
#define EC_DELTA_MODE 0x1
// PRESCALE bits
#define EC_BYPASS 0x0
#define EC_DIV1 0x0
#define EC_DIV2 0x1
#define EC_DIV4 0x2
#define EC_DIV6 0x3
#define EC_DIV8 0x4
#define EC_DIV10 0x5
// ECCTL2 ( ECAP Control Reg 2)
//==========================
// CONT/ONESHOT bit
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT 0x1
// STOPVALUE bit
#define EC_EVENT1 0x0
#define EC_EVENT2 0x1
#define EC_EVENT3 0x2
#define EC_EVENT4 0x3
// RE-ARM bit
#define EC_ARM 0x1
// TSCTRSTOP bit
#define EC_FREEZE 0x0
#define EC_RUN 0x1
// SYNCO_SEL bit
#define EC_SYNCIN 0x0
#define EC_CTR_PRD 0x1
#define EC_SYNCO_DIS 0x2
// CAP/APWM mode bit
#define EC_CAP_MODE 0x0
#define EC_APWM_MODE 0x1
// APWMPOL bit
#define EC_ACTV_HI 0x0
#define EC_ACTV_LO 0x1
// Generic
#define EC_DISABLE 0x0
#define EC_ENABLE 0x1
#define EC_FORCE 0x1
void InitECaptureContinuousMode(){
    InitECapture();

    //        16.6.1
    // ECAP module 1 config
    //        ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CAP2POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CAP4POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    //        ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    //        ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    //        ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    //        ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    //        ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    //        ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

    //        16.6.2
    // ECAP module 1 config
    //        ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    //        ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
    //        ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    //        ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
    //        ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    //        ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    //        ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    //        ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    //        ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    //        ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    //        ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

    //        16.6.4
    // Code snippet for CAP mode Delta Time, Rising and Falling
    // edge triggers
    // Initialization Time
    //=======================
    // ECAP module 1 config
    ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
    ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
    ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

    // ECAP module 2 config
    ECap2Regs.ECCTL1.bit.CAP1POL = EC_RISING;
    ECap2Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    ECap2Regs.ECCTL1.bit.CAP3POL = EC_RISING;
    ECap2Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    ECap2Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
    ECap2Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
    ECap2Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
    ECap2Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
    ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap2Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

    // ECAP module 3 config
    ECap3Regs.ECCTL1.bit.CAP1POL = EC_RISING;
    ECap3Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    ECap3Regs.ECCTL1.bit.CAP3POL = EC_RISING;
    ECap3Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    ECap3Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
    ECap3Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
    ECap3Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
    ECap3Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
    ECap3Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap3Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap3Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap3Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;
}


#define ECAP_MA_ARRAY_LENGTH 10
struct st_ecap_ma{
    REAL sum;
    REAL Array[ECAP_MA_ARRAY_LENGTH];
    int cursor;
    int length;
} eCapMA1, eCapMA2;

void ecap_moving_average(){
    REAL ud, uq;
    /*Use moving-average ecap measured voltage*/
    /*D*/
    eCapMA1.sum -= eCapMA1.Array[eCapMA1.cursor];
    eCapMA1.sum +=                  CTRL.I->ecap_dq[0];
    eCapMA1.Array[eCapMA1.cursor] = CTRL.I->ecap_dq[0];
    eCapMA1.cursor+=1;
    if(eCapMA1.length==0 || eCapMA1.length>ECAP_MA_ARRAY_LENGTH){
        if(eCapMA1.cursor>=5){
            eCapMA1.cursor=0;
        }
        ud = eCapMA1.sum*0.2;
    }else{
        if(eCapMA1.cursor>=eCapMA1.length){
            eCapMA1.cursor=0;
        }
        ud = eCapMA1.sum/eCapMA1.length;
    }
    /*Q*/
    eCapMA2.sum -= eCapMA2.Array[eCapMA2.cursor];
    eCapMA2.sum +=                  CTRL.I->ecap_dq[1];
    eCapMA2.Array[eCapMA2.cursor] = CTRL.I->ecap_dq[1];
    eCapMA2.cursor+=1;
    if(eCapMA2.length==0 || eCapMA2.length>ECAP_MA_ARRAY_LENGTH){
        if(eCapMA2.cursor>=5){
            eCapMA2.cursor=0;
        }
        uq = eCapMA2.sum*0.2;
    }else{
        if(eCapMA2.cursor>=eCapMA2.length){
            eCapMA2.cursor=0;
        }
        uq = eCapMA2.sum/eCapMA2.length;
    }
    US_P(0) = CTRL.S->cosT*ud - CTRL.S->sinT*uq;
    US_P(1) = CTRL.S->sinT*ud + CTRL.S->cosT*uq;
}

