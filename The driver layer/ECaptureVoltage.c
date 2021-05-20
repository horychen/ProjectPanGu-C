/*
 * ECaptureVoltage.c
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */
#include <All_Definition.h>


#define USE_ECAP_CEVT2_INTERRUPT 1
void do_enhanced_capture(){    // Section 16.6.4 // The 32 bit counter is stored in ECap1Regs.TSCTR
    /*UVW*/
    #if USE_ECAP_CEVT2_INTERRUPT == 1
        CAP.ecapU.DutyOnTime1  = CAP.good_capture_U[1];
        CAP.ecapU.DutyOffTime1 = CAP.good_capture_U[2];
        CAP.ecapU.DutyOnTime2  = CAP.good_capture_U[3];
        CAP.ecapU.DutyOffTime2 = CAP.good_capture_U[0];
        CAP.ecapV.DutyOnTime1  = CAP.good_capture_V[1];
        CAP.ecapV.DutyOffTime1 = CAP.good_capture_V[2];
        CAP.ecapV.DutyOnTime2  = CAP.good_capture_V[3];
        CAP.ecapV.DutyOffTime2 = CAP.good_capture_V[0];
        CAP.ecapW.DutyOnTime1  = CAP.good_capture_W[1];
        CAP.ecapW.DutyOffTime1 = CAP.good_capture_W[2];
        CAP.ecapW.DutyOnTime2  = CAP.good_capture_W[3];
        CAP.ecapW.DutyOffTime2 = CAP.good_capture_W[0];
    #else
        CAP.ecapU.DutyOnTime1  = ECap1Regs.CAP2;
        CAP.ecapU.DutyOffTime1 = ECap1Regs.CAP3;
        CAP.ecapU.DutyOnTime2  = ECap1Regs.CAP4;
        CAP.ecapU.DutyOffTime2 = ECap1Regs.CAP1;
        CAP.ecapV.DutyOnTime1  = ECap2Regs.CAP2;
        CAP.ecapV.DutyOffTime1 = ECap2Regs.CAP3;
        CAP.ecapV.DutyOnTime2  = ECap2Regs.CAP4;
        CAP.ecapV.DutyOffTime2 = ECap2Regs.CAP1;
        CAP.ecapW.DutyOnTime1  = ECap3Regs.CAP2;
        CAP.ecapW.DutyOffTime1 = ECap3Regs.CAP3;
        CAP.ecapW.DutyOnTime2  = ECap3Regs.CAP4;
        CAP.ecapW.DutyOffTime2 = ECap3Regs.CAP1;
    #endif
    CAP.ecapU.Period1 = CAP.ecapU.DutyOnTime1 + CAP.ecapU.DutyOffTime1;
    CAP.ecapU.Period2 = CAP.ecapU.DutyOnTime2 + CAP.ecapU.DutyOffTime2;
    CAP.ecapV.Period1 = CAP.ecapV.DutyOnTime1 + CAP.ecapV.DutyOffTime1;
    CAP.ecapV.Period2 = CAP.ecapV.DutyOnTime2 + CAP.ecapV.DutyOffTime2;
    CAP.ecapW.Period1 = CAP.ecapW.DutyOnTime1 + CAP.ecapW.DutyOffTime1;
    CAP.ecapW.Period2 = CAP.ecapW.DutyOnTime2 + CAP.ecapW.DutyOffTime2;

    /* Nonlinear Filtering of the disturbance */
    CAP.flag_bad_U_capture = FALSE;
    CAP.flag_bad_V_capture = FALSE;
    CAP.flag_bad_W_capture = FALSE;
    if(CAP.flag_nonlinear_filtering){
        /*U ע��Uiint32�������������ǳ����׳�����*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapU.DutyOnTime1 - (double)CAP.ecapU.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapU.DutyOnTime2 - (double)CAP.ecapU.DutyOffTime2) > 4444.0
          ){
            CAP.flag_bad_U_capture = TRUE;
        }

        /*V*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapV.DutyOnTime1 - (double)CAP.ecapV.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapV.DutyOnTime2 - (double)CAP.ecapV.DutyOffTime2) > 4444.0
          ){
            CAP.flag_bad_V_capture = TRUE;
        }

        /*W*/
        if( fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapW.DutyOnTime1 - (double)CAP.ecapW.DutyOffTime1) > 4444.0 ||
            fabs((double)SYSTEM_PWM_MAX_COUNT - (double)CAP.ecapW.DutyOnTime2 - (double)CAP.ecapW.DutyOffTime2) > 4444.0
          ){
            CAP.flag_bad_W_capture = TRUE;
        }

        if(!CAP.flag_bad_U_capture){
            CAP.terminal_DutyOnRatio[0] = 0.5*(CAP.ecapU.DutyOnTime1 + CAP.ecapU.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
            CAP.pwm_time = 0.5*(CAP.ecapU.Period1 + CAP.ecapU.Period2);
        }
        if(!CAP.flag_bad_V_capture){
            CAP.terminal_DutyOnRatio[1] = 0.5*(CAP.ecapV.DutyOnTime1 + CAP.ecapV.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        }
        if(!CAP.flag_bad_W_capture){
            CAP.terminal_DutyOnRatio[2] = 0.5*(CAP.ecapW.DutyOnTime1 + CAP.ecapW.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        }

    }else{

        //    TODO: accuracy can be further improve by replacing SYSTEM_PWM_MAX_COUNT_INVERSE with actual period measured? No, the period is not accurate as the duty ratio is chaning.
        /*API to global variable CTRL with two point moving average*/
        CAP.terminal_DutyOnRatio[0] = 0.5*(CAP.ecapU.DutyOnTime1 + CAP.ecapU.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        CAP.terminal_DutyOnRatio[1] = 0.5*(CAP.ecapV.DutyOnTime1 + CAP.ecapV.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
        CAP.terminal_DutyOnRatio[2] = 0.5*(CAP.ecapW.DutyOnTime1 + CAP.ecapW.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE; // / (0.5*(CAP.ecapW.Period1+CAP.ecapW.Period2))
        CAP.pwm_time = 0.5*(CAP.ecapU.Period1 + CAP.ecapU.Period2);
    }
}

// From: D:\ti\controlSUITE\device_support\F2837xS\v210\F2837xS_examples_Cpu1\eCAPture_pwm\cpu01\ECAPture_Pwm_cpu01.c
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
    ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE; // CEVT2
    ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE; // CEVT2
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
    ECap2Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE; // CEVT2
    ECap2Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE; // CEVT2
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
    ECap3Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE; // CEVT2
    ECap3Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE; // CEVT2
    ECap3Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap3Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap3Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap3Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;
}


#if 0
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
    eCapMA1.sum +=                  CAP.dq[0];
    eCapMA1.Array[eCapMA1.cursor] = CAP.dq[0];
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
    eCapMA2.sum +=                  CAP.dq[1];
    eCapMA2.Array[eCapMA2.cursor] = CAP.dq[1];
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
#endif



/*U*/
__interrupt void ecap1_isr(void){

    // isr nesting test
    CAP.decipher_password[0] = CAP.password_isr_nesting + 21;

    CAP.ECapIntCount[0]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = CAP.good_capture_U[2];
    CAP2 = CAP.good_capture_U[3];
    CAP3 = ECap1Regs.CAP1;
    CAP4 = ECap1Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3; // if OnTime2 is 0, then OffTime2 becomes 20000.
    Uint32 DutyOnTime2  = CAP4; // if OnTime2 is 0, then OffTime2 becomes 20000.
    Uint32 DutyOffTime2 = CAP1;

        //Uint32 Period1 = DutyOnTime1 + DutyOffTime1;
        //Uint32 Period2 = DutyOnTime2 + DutyOffTime2;
        /* This is not the actual period, if we use symmetric PWM,
         * the center location of the square wave high
         * to the next center location of the square wave high
         * is the period. */
        //if(Period1 < SYSTEM_HALF_PWM_MAX_COUNT){ Wrong! Period1 can be larger than 10000
        //if(Period2 < SYSTEM_HALF_PWM_MAX_COUNT){

    // On1 Off1 һ�����(CAP2 and CAP3) 
    // ��4444��ô������֣�������444������Ϊ�ҷ�����ʱ��V������õ���ON1��OFF1�ֱ�Ϊ9000+��9000+��������С��19000���ж�Ϊbad��������Ӧ�������������Ĳ���
    // ���ֵ��ǣ�V���CAP1��ʱ����Ϊ20000+����������Ϊʲô��
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        CAP.u_ecap_bad1.on1  = DutyOnTime1;
        CAP.u_ecap_bad1.off1 = DutyOffTime1;
        CAP.u_ecap_bad1.on2  = DutyOnTime2;
        CAP.u_ecap_bad1.off2 = DutyOffTime2;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        CAP.u_ecap_bad2.on1  = DutyOnTime1;
        CAP.u_ecap_bad2.off1 = DutyOffTime1;
        CAP.u_ecap_bad2.on2  = DutyOnTime2;
        CAP.u_ecap_bad2.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        CAP.ECapPassCount[0] += 1;
        CAP.good_capture_U[0] = CAP1;
        CAP.good_capture_U[1] = CAP2;
        CAP.good_capture_U[2] = CAP3;
        CAP.good_capture_U[3] = CAP4;
    }

        /* For ECap1Regs.ECEINT.bit.CEVT4 = 1; */
        // �����������������룬���ܱ�֤EPWM1�ж�����ʱ���Ѿ��ɼ����˵�ǰ�ĵ�ѹ������Ƶ��Ҳ�����죡
        // ����Ӧ�ð�ECAP�жϸĳ�һ�������ж�һ�Σ�����������ȼ���

        //    // ��һ��������½��غ͵ڶ��������������������߼������ر�С����˵���ǵ�����
        //    if(ECap1Regs.CAP2 + ECap1Regs.CAP3 < SYSTEM_HALF_PWM_MAX_COUNT){
        //        // �ߵ�ƽʱ���ֵ����壬������CAP2/3���ϣ�
        //        // ���Ǵ�ʱ��CAP1��4ʵ������������CAP1��CAP2����������CAP3��CAP4��û�б�ץ����
        //        good_capture_counter_U[0] = ECap1Regs.CAP1;
        //        good_capture_counter_U[1] = ECap1Regs.CAP4;
        //    }else{
        //        // ��Ч
        //        good_capture_counter_U[1] = ECap1Regs.CAP2;
        //        good_capture_counter_U[2] = ECap1Regs.CAP3;
        //    }
        //    // �ڶ���������½��غ͵�һ���������������������߼������ر�С����˵����ͻ�����
        //    if(ECap1Regs.CAP4 + ECap1Regs.CAP1 < SYSTEM_HALF_PWM_MAX_COUNT){
        //        // �͸ߵ�ƽʱ����ͻ����壬������CAP4/1����
        //    }else{
        //        // ��Ч
        //        good_capture_counter_U[3] = ECap1Regs.CAP4;
        //        good_capture_counter_U[0] = ECap1Regs.CAP1;
        //    }

        //    do_enhanced_capture();

        //ECap1Regs.ECCLR.bit.CEVT4 = 1; // 4 events == __interrupt

    ECap1Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap1Regs.ECCLR.bit.INT = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}



/*V*/
__interrupt void ecap2_isr(void){

    // isr nesting test
    CAP.decipher_password[1] = CAP.password_isr_nesting + 22;

    CAP.ECapIntCount[1]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = CAP.good_capture_V[2];
    CAP2 = CAP.good_capture_V[3];
    CAP3 = ECap2Regs.CAP1;
    CAP4 = ECap2Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3;
    Uint32 DutyOnTime2  = CAP4;
    Uint32 DutyOffTime2 = CAP1;

    // On1 Off1 һ�����(CAP2 and CAP3)
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        CAP.v_ecap_bad1.on1  = DutyOnTime1;
        CAP.v_ecap_bad1.off1 = DutyOffTime1;
        CAP.v_ecap_bad1.on2  = DutyOnTime2;
        CAP.v_ecap_bad1.off2 = DutyOffTime2;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        CAP.v_ecap_bad2.on1  = DutyOnTime1;
        CAP.v_ecap_bad2.off1 = DutyOffTime1;
        CAP.v_ecap_bad2.on2  = DutyOnTime2;
        CAP.v_ecap_bad2.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        CAP.ECapPassCount[1] += 1;
        CAP.good_capture_V[0] = CAP1;
        CAP.good_capture_V[1] = CAP2;
        CAP.good_capture_V[2] = CAP3;
        CAP.good_capture_V[3] = CAP4;
    }

    ECap2Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap2Regs.ECCLR.bit.INT = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


/*W*/
__interrupt void ecap3_isr(void){

    // isr nesting test
    CAP.decipher_password[2] = CAP.password_isr_nesting + 23;

    CAP.ECapIntCount[2]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = CAP.good_capture_W[2];
    CAP2 = CAP.good_capture_W[3];
    CAP3 = ECap3Regs.CAP1;
    CAP4 = ECap3Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3;
    Uint32 DutyOnTime2  = CAP4;
    Uint32 DutyOffTime2 = CAP1;

    // On1 Off1 һ�����(CAP2 and CAP3)
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        CAP.w_ecap_bad1.on1  = DutyOnTime1;
        CAP.w_ecap_bad1.off1 = DutyOffTime1;
        CAP.w_ecap_bad1.on2  = DutyOnTime2;
        CAP.w_ecap_bad1.off2 = DutyOffTime2;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) ) ){
        // || // and or or? 
        // ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        CAP.w_ecap_bad2.on1  = DutyOnTime1;
        CAP.w_ecap_bad2.off1 = DutyOffTime1;
        CAP.w_ecap_bad2.on2  = DutyOnTime2;
        CAP.w_ecap_bad2.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        CAP.ECapPassCount[2] += 1;
        CAP.good_capture_W[0] = CAP1;
        CAP.good_capture_W[1] = CAP2;
        CAP.good_capture_W[2] = CAP3;
        CAP.good_capture_W[3] = CAP4;
    }

    ECap3Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap3Regs.ECCLR.bit.INT = 1;
    ECap3Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
