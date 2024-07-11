#include "ACMSim.h"

#define INCREMENTAL_PID TRUE
#if INCREMENTAL_PID
void PID_calc(st_pid_regulator *r){

    r->Err = r->Ref - r->Fbk;
    r->Out = r->OutPrev \
             + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki * r->Err;

    if(r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;

    r->ErrPrev = r->Err; 
    r->OutPrev = r->Out;
}
#else
void PID_calc(st_pid_regulator *r){
    #define DYNAMIC_CLAPMING TRUE

    // 璇樊
    r->Err = r->Ref - r->Fbk;

    // 姣斾緥
    r->P_Term = r->Err * r->Kp;

    // 绉垎
    r->I_Term += r->Err * r->Ki;
    r->OutNonSat = r->I_Term;

    // 娣诲姞绉垎楗卞拰鐗规��
    #if DYNAMIC_CLAPMING
        // dynamic clamping
        if( r->I_Term > r->OutLimit - r->Out)
            r->I_Term = r->OutLimit - r->Out;
        else if( r->I_Term < -r->OutLimit + r->Out)
            r->I_Term =      -r->OutLimit + r->Out; // OutLimit is a positive constant
    #else
        // static clamping
        if( r->I_Term > r->OutLimit)
            r->I_Term = r->OutLimit; 
        else if( r->I_Term < -r->OutLimit)
            r->I_Term = -r->OutLimit;
    #endif

    // 寰垎
    // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

    // 杈撳嚭
    r->Out = r->I_Term + r->P_Term; // + r->D_Term
    r->OutNonSat += r->P_Term; // + r->D_Term
    // 杈撳嚭闄愬箙
    if(r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;

    // 褰撳墠姝ヨ宸祴鍊间负涓婁竴姝ヨ宸�
    r->ErrPrev = r->Err;
    // 璁板綍楗卞拰杈撳嚭鍜屾湭楗卞拰杈撳嚭鐨勫樊
    r->SatDiff = r->Out - r->OutNonSat;
}
#endif


void PIDController_Init(st_PIDController *pid) {

    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(st_PIDController *pid) {

    // * Error signal
    float error = pid->setpoint - pid->measurement;

    // * Proportional   
    float proportional = pid->Kp * error;

    // * Integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->Ts * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->intLimit) {
        pid->integrator = pid->intLimit;
    } else if (pid->integrator < -pid->intLimit) {
        pid->integrator = -pid->intLimit;
    }

    // * Derivative (band-limited differentiator)       
    pid->differentiator = -(2.0f * pid->Kd * (pid->measurement - pid->prevMeasurement)   /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->Ts) * pid->differentiator)
                        / (2.0f * pid->tau + pid->Ts);

    // * Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->outLimit) {
        pid->out = pid->outLimit;
    } else if (pid->out < -pid->outLimit) {
        pid->out = -pid->outLimit;
    }

    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = pid->measurement;

    /* Return controller output */
    return pid->out;
}


// 鍒濆鍖栧嚱鏁�
void ACMSIMC_PIDTuner(){

    PID_iD->Kp  = CURRENT_KP;
    PID_iQ->Kp = CURRENT_KP;     // 0.7;//CURRENT_KP;
    PID_iD->Ki  = CURRENT_KI_CODE;
    PID_iQ->Ki  = CURRENT_KI_CODE; // 0.09;//CURRENT_KI_CODE;

    PID_spd->Kp = SPEED_KP; // 0.0008;//SPEED_KP;
    PID_spd->Ki = SPEED_KI_CODE; // 4e-6;//SPEED_KI_CODE;
    
    PID_pos->Kp = POS_KP;
    PID_pos->Ki = POS_KI;

    PID_iD->OutLimit  = CURRENT_LOOP_LIMIT_VOLTS;
    PID_iQ->OutLimit  = CURRENT_LOOP_LIMIT_VOLTS;
    PID_spd->OutLimit = SPEED_LOOP_LIMIT_AMPERE; // 40;//SPEED_LOOP_LIMIT_AMPERE;
    PID_pos->OutLimit = POS_LOOP_LIMIT_SPEED;

    #if PC_SIMULATION
    printf("%f\n", pid1_iM.Kp);
    printf("%f\n", pid1_iM.Ki);
    printf("%f\n", pid1_iT.Kp);
    printf("%f\n", pid1_iT.Ki);
    printf("%f\n", pid1_spd.Kp);
    printf("%f\n", pid1_spd.Ki);
    #endif

    PID_iX->Kp = CURRENT_KP;
    PID_iY->Kp = CURRENT_KP;
    PID_iX->Ki = CURRENT_KI_CODE;
    PID_iY->Ki = CURRENT_KI_CODE;

    PID_iX->outLimit = CURRENT_LOOP_LIMIT_VOLTS;
    PID_iY->outLimit = CURRENT_LOOP_LIMIT_VOLTS;

    PIDController_Init(PID_iX); // pid1_dispX
    PIDController_Init(PID_iY);

    // #define DISPLACEMENT_KP 0
    // #define DISPLACEMENT_KI_CODE 0
    // #define DISPLACEMENT_LOOP_LIMIT_AMPERE 0

    // pid1_disX.Kp       = DISPLACEMENT_KP;
    // pid1_disY.Kp       = DISPLACEMENT_KP;
    // pid1_disX.Ki       = DISPLACEMENT_KI_CODE;
    // pid1_disY.Ki       = DISPLACEMENT_KI_CODE;
    // pid1_disX.OutLimit = DISPLACEMENT_LOOP_LIMIT_AMPERE;
    // pid1_disY.OutLimit = DISPLACEMENT_LOOP_LIMIT_AMPERE;

    // pid1_ia.Kp = CURRENT_KP;
    // pid1_ib.Kp = CURRENT_KP;
    // pid1_ic.Kp = CURRENT_KP;
    // pid1_ia.Ki = CURRENT_KI_CODE;
    // pid1_ib.Ki = CURRENT_KI_CODE;
    // pid1_ic.Ki = CURRENT_KI_CODE;
    // pid1_ia.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    // pid1_ib.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    // pid1_ic.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
}

struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};
