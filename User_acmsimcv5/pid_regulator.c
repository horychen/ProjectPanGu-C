#include "ACMSim.h"

#define INCREMENTAL_PID TRUE
#if INCREMENTAL_PID
    // void PID_calc(st_pid_regulator *r){

    //     r->Err = r->Ref - r->Fbk;
    //     r->Out = r->OutPrev \
    //             + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki * r->Err;

    //     if(r->Out > r->OutLimit)
    //         r->Out = r->OutLimit;
    //     else if(r->Out < -r->OutLimit)
    //         r->Out = -r->OutLimit;

    //     r->ErrPrev = r->Err; 
    //     r->OutPrev = r->Out;
    // }
    void PID_calc(st_pid_regulator *r){

        // 打印 r->Kp 的值
        // printf("Value of r->Kp: %f\n", r->Kp);
        // printf("Value of r->Ki: %f\n", r->Ki);
        // printf("Value of r->Ref: %f\n", r->Ref);

        r->Err = r->Ref - r->Fbk;

        r->Out = r->OutPrev + \
                r->Kp * ( r->Err - r->ErrPrev ) + \
                r->Ki_CODE * r->Err;

        r->ErrPrev = r->Err; 
        r->OutPrev = r->Out;

        r->KFB_Term = r->KFB * r->Fbk;
        r->Out -= r->KFB_Term;

        // 控制器的限幅需要写在OutPrev幅值之后，否则电流环idq暂态跟踪误差会达到50%左右
        // 20240812：理论分析，限幅放在之前之后貌似没有区别，但仿真结果确实不同
        if(r->Out > r->OutLimit)
            r->Out = r->OutLimit;
        else if(r->Out < -r->OutLimit)
            r->Out = -r->OutLimit;
        // printf("KFB is %f\n", r->KFB);
    }
#else
    void PID_calc(st_pid_regulator *r){
        #define DYNAMIC_CLAPMING TRUE
        #define DYNAMIC_CLAPMING_WUBO FALSE

        // 误差
        r->Err = r->Ref - r->Fbk;

        // 比例
        r->P_Term = r->Err * r->Kp;

        // 积分
        r->I_Term += r->Err * r->Ki;
        r->OutNonSat = r->I_Term;

        // Inner Loop
        r->KFB_Term = r->KFB * r->Fbk;

        // 添加积分饱和特性
        #if DYNAMIC_CLAPMING
            #if DYNAMIC_CLAPMING_WUBO
                // dynamic clamping wubo
                if( r->I_Term > r->OutLimit - r->P_Term)
                    r->I_Term = r->OutLimit - r->P_Term;
                else if( r->I_Term < -r->OutLimit + r->P_Term)
                    r->I_Term =      -r->OutLimit + r->P_Term; // OutLimit is a positive constant
           #else
                // dynamic clamping
                if( r->I_Term > r->OutLimit - r->Out)
                    r->I_Term = r->OutLimit - r->Out;
                else if( r->I_Term < -r->OutLimit + r->Out)
                    r->I_Term =      -r->OutLimit + r->Out; // OutLimit is a positive constant
            #endif
        #else
            // static clamping
            if( r->I_Term > r->OutLimit)
                r->I_Term = r->OutLimit; 
            else if( r->I_Term < -r->OutLimit)
                r->I_Term = -r->OutLimit;
        #endif

        // 微分
        // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

        // 输出
        r->Out = r->I_Term + r->P_Term - r->KFB_Term; // + r->D_Term
        r->OutNonSat += r->P_Term; // + r->D_Term
        // 输出限幅
        if(r->Out > r->OutLimit)
            r->Out = r->OutLimit;
        else if(r->Out < -r->OutLimit)
            r->Out = -r->OutLimit;

        // 当前步误差赋值为上一步误差
        r->ErrPrev = r->Err;
        // 记录饱和输出和未饱和输出的差
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


// 初始化函数
void ACMSIMC_PIDTuner(){

    PID_iD->Kp  = d_sim.CL.SERIES_KP_D_AXIS;
    PID_iQ->Kp  = d_sim.CL.SERIES_KP_Q_AXIS;
    PID_Speed->Kp = d_sim.VL.SERIES_KP;

    PID_iD->Ki_CODE  = d_sim.CL.SERIES_KI_D_AXIS * d_sim.CL.SERIES_KP_D_AXIS * CL_TS;
    PID_iQ->Ki_CODE  = d_sim.CL.SERIES_KI_Q_AXIS * d_sim.CL.SERIES_KP_Q_AXIS * CL_TS;
    PID_Speed->Ki_CODE = d_sim.VL.SERIES_KI        * d_sim.VL.SERIES_KP        * VL_TS;

    PID_Speed->KFB = 0.0;

    PID_iD->OutLimit  = d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit  = d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;

    // pid2_ix.Kp = CURRENT_KP;
    // pid2_iy.Kp = CURRENT_KP;
    // pid2_ix.Ki = CURRENT_KI_CODE;
    // pid2_iy.Ki = CURRENT_KI_CODE;
    // pid2_ix.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    // pid2_iy.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;

    // PIDController_Init(&pid1_dispX);
    // PIDController_Init(&pid1_dispY);

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
