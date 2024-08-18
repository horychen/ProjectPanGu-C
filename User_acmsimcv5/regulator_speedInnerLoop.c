#include "ACMSim.h"

// int wubo_debug[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

#define WUBO_INCREMENTAL_PID TRUE
#define DYNAMIC_CLAMPING_FOR_KFB FALSE

#if WUBO_INCREMENTAL_PID
    void wubo_PID_calc(st_pid_regulator *r){

        r->Err = r->Ref - r->Fbk;
        r->Out = r->OutPrev \
                + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki_CODE * r->Err;

        r->ErrPrev = r->Err; 
        r->OutPrev = r->Out;

        // 控制器的限幅需要写在OutPrev幅值之后，否则电流环idq暂态跟踪误差会达到50%左右
        // 20240812：理论分析，限幅放在之前之后貌似没有区别，但仿真结果确实不同
        // if(r->OutPrev> r->OutLimit)
        //     r->OutPrev= r->OutLimit;
        // else if(r->OutPrev< -r->OutLimit)
        //     r->OutPrev= -r->OutLimit;
        if(r->Out > r->OutLimit)
            r->Out = r->OutLimit;
        else if(r->Out < -r->OutLimit)
            r->Out = -r->OutLimit;
    }

    #if 0
        void InnerLoopFeedback_calc(st_pid_regulator *r){
            #define DYNAMIC_CLAPMING FALSE
            #define DYNAMIC_CLAPMING_WUBO TRUE

            // 误差
            r->Err = r->Ref - r->Fbk;
            // 比例
            r->P_Term = r->Err * r->Kp;
            // 积分
            r->I_Term += r->Err * r->Ki_CODE;
            r->OutNonSat = r->I_Term;
            // Inner Loop
            r->KFB_Term = r->Fbk * r->KFB;

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
            r->Out = r->I_Term + r->P_Term - 0.15 * r->KFB_Term; // + r->D_Term
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
    #else
        void InnerLoopFeedback_calc(st_pid_regulator *r){

            r->Err = r->Ref - r->Fbk;
            r->Out = r->OutPrev + \
                    r->Kp * ( r->Err - r->ErrPrev ) \
                    + r->Ki_CODE * r->Err; // \
                    // - r->KFB_Term;

            r->ErrPrev = r->Err; 
            r->OutPrev = r->Out;

            #if 1
                r->KFB_Term = 1.0 * r->KFB * r->Fbk;
            #else
                r->KFB_Term = r->KFB_Term_Prev + r->KFB * ( r->Fbk - r->FbkPrev );
                r->FbkPrev       = r->Fbk;
                r->KFB_Term_Prev = r->KFB_Term;
            #endif

            #if DYNAMIC_CLAMPING_FOR_KFB
                r->P_Term = r->Err * r->Kp;
                if(r->KFB_Term > r->OutLimit - r->P_Term)
                    r->KFB_Term = r->OutLimit - r->P_Term;
                else if(r->KFB_Term < -r->OutLimit + r->P_Term)
                    r->KFB_Term = -r->OutLimit + r->P_Term;
            #endif

            r->Out = r->Out - r->KFB_Term;

            // 控制器的限幅需要写在OutPrev幅值之后，否则电流环idq暂态跟踪误差会达到50%左右
            // 20240812：理论分析，限幅放在之前之后貌似没有区别，但仿真结果确实不同
            if(r->OutPrev> r->OutLimit+r->KFB_Term)
                r->OutPrev= r->OutLimit+r->KFB_Term;
            else if(r->OutPrev< -r->OutLimit+r->KFB_Term)
                r->OutPrev= -r->OutLimit+r->KFB_Term;
            // # VERY BUGGY if you use incremental to implement inner loop 因为要对 OutPrev 限幅才对！
            if(r->Out > r->OutLimit)
                r->Out = r->OutLimit;
            else if(r->Out < -r->OutLimit)
                r->Out = -r->OutLimit;
        }
    #endif
#else
    void wubo_PID_calc(st_pid_regulator *r){
        #define DYNAMIC_CLAPMING TRUE
        #define DYNAMIC_CLAPMING_WUBO TRUE

        // 误差
        r->Err = r->Ref - r->Fbk;
        // 比例
        r->P_Term = r->Err * r->Kp;
        // 积分
        r->I_Term += r->Err * r->Ki_CODE;
        r->OutNonSat = r->I_Term;
        // Inner Loop
        r->KFB_Term = r->Fbk * r->KFB;

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

void _user_wubo_WC_Tuner(){
    // 吴波用：
    if(debug.who_is_user == USER_WB){
        PID_Speed->calc = (void (*)(Uint32)) InnerLoopFeedback_calc;
    }

    REAL zeta = d_sim.user.zeta;
    REAL omega_n = d_sim.user.omega_n;
    REAL max_CLBW_PER_min_CLBW = d_sim.user.max_CLBW_PER_min_CLBW;
    REAL Ld = d_sim.init.Ld;
    REAL Lq = d_sim.init.Lq;
    REAL R = d_sim.init.R;
    REAL Js = d_sim.init.Js;
    REAL npp = d_sim.init.npp;
    REAL KE = d_sim.init.KE;
    
    // motor parameters
    REAL KT = 1.5 * npp * KE;  // torque constant
    REAL K0 = KT / Js;  // motor constant

    REAL max_CLBW = zeta * omega_n * 4;
    REAL min_CLBW = zeta * omega_n * 2;

    // 通过一个小于1的比例系数来选取电流环带宽
    REAL FOC_CLBW = max_CLBW_PER_min_CLBW * max_CLBW +  (1-max_CLBW_PER_min_CLBW) * min_CLBW;
    // printf("FOC_CLBW = %f\n", FOC_CLBW);

    REAL Series_D_KP = FOC_CLBW * Ld;
    REAL Series_D_KI = R / Ld;
    REAL Series_Q_KP = FOC_CLBW * Lq;
    REAL Series_Q_KI = R / Lq;
    REAL Series_Speed_KP = omega_n * omega_n / (FOC_CLBW * K0);
    REAL Series_Speed_KFB = (2*zeta*omega_n*FOC_CLBW - 4*zeta*zeta*omega_n*omega_n) / (FOC_CLBW*K0);
    REAL Series_Speed_KI = ( FOC_CLBW - (sqrt(FOC_CLBW * FOC_CLBW - 4*FOC_CLBW*K0*Series_Speed_KFB)) ) * 0.5;


    PID_iD->Kp = Series_D_KP;
    PID_iQ->Kp = Series_Q_KP;
    PID_Speed->Kp = Series_Speed_KP;

    PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS;
    PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS;
    PID_Speed->Ki_CODE = Series_Speed_KI * Series_Speed_KP * VL_TS;
    PID_Speed->KFB = Series_Speed_KFB;

    PID_iD->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;

    #if PC_SIMULATION == TRUE
        printf("Series_D_KP = %f\n", Series_D_KP);
        printf("Series_D_KI = %f\n", Series_D_KI);
        printf("Series_Q_KP = %f\n", Series_Q_KP);
        printf("Series_Q_KI = %f\n", Series_Q_KI);
        printf("Series_Speed_KP = %f\n", Series_Speed_KP);
        printf("Series_Speed_KFB = %f\n", Series_Speed_KFB);
        printf("Series_Speed_KI = %f\n", Series_Speed_KI);
        if (FOC_CLBW - 4 * K0 * PID_Speed->KFB < 0){
            printf("can not do zero-pole cancellation\n");
        }else{
            printf(">>> Zero-pole cancellation can be done <<<\n");
        }
    #endif
}


void _user_controller_wubo(){

    if ((*CTRL).s->the_vc_count++ >= SPEED_LOOP_CEILING){
        (*CTRL).s->the_vc_count = 1;
        PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
        PID_Speed->Fbk = (*CTRL).i->varOmega;
        if(debug.who_is_user == USER_BEZIER){
            control_output(PID_Speed, &BzController);
        }else if(debug.who_is_user == USER_WB){
            PID_Speed->calc(PID_Speed);
        }else if(debug.who_is_user == USER_CJH){
            PID_Speed->calc(PID_Speed);
        }else{
            debug.error = 998;
        }
        (*CTRL).i->cmd_iDQ[1] = PID_Speed->Out;
    }

    // 帕克变换
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    /* 更新依赖于dq轴电流的物理量 */
    (*CTRL).i->Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0]) * (*CTRL).i->iDQ[1];     // 转矩 For luenberger position observer for HFSI
    (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0]) * (*CTRL).i->cmd_iDQ[1];
    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];


    /* 启动D_Cuurent = 0A */
    if(d_sim.user.Null_D_Control == TRUE)
    {
        (*CTRL).i->cmd_iDQ[0] = 0;
    }

    /* 速度双环的电流环 */
    // d-axis
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    PID_iD->calc(PID_iD);
    // q-axis
    PID_iQ->Fbk = (*CTRL).i->iDQ[1];
    PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
    PID_iQ->calc(PID_iQ);

    /* 电流环前馈DQ轴解耦 */
    REAL decoupled_d_axis_voltage;
    REAL decoupled_q_axis_voltage;
    if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
        decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
        decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
    }else{
        decoupled_d_axis_voltage = PID_iD->Out;
        decoupled_q_axis_voltage = PID_iQ->Out;
    }

    /* 对补偿后的dq轴电压进行限幅度 */
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;

    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    /// 7. 反帕克变换
    // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
    // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
    // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

    // (*CTRL).o->cmd_uAB[0+2] = MT2A((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);
    // (*CTRL).o->cmd_uAB[1+2] = MT2B((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);

    /// 8. 补偿逆变器非线性
    main_inverter_voltage_command(TRUE);
}

void _user_wubo_WC_Tuner_Online(){
    // 吴波用：
    if(debug.who_is_user == USER_WB){
        PID_Speed->calc = (void (*)(Uint32)) InnerLoopFeedback_calc;
    }

    /* Tuned by debug */
    REAL zeta = debug.zeta;
    REAL omega_n = debug.omega_n;
    REAL max_CLBW_PER_min_CLBW = debug.max_CLBW_PER_min_CLBW;

    REAL Ld = d_sim.init.Ld;
    REAL Lq = d_sim.init.Lq;
    REAL R = d_sim.init.R;
    REAL Js = d_sim.init.Js;
    REAL npp = d_sim.init.npp;
    REAL KE = d_sim.init.KE;
    
    // motor parameters
    REAL KT = 1.5 * npp * KE;  // torque constant
    REAL K0 = KT / Js;  // motor constant

    REAL max_CLBW = zeta * omega_n * 4;
    REAL min_CLBW = zeta * omega_n * 2;

    // 通过一个小于1的比例系数来选取电流环带宽
    REAL FOC_CLBW = max_CLBW_PER_min_CLBW * max_CLBW +  (1-max_CLBW_PER_min_CLBW) * min_CLBW;
    // printf("FOC_CLBW = %f\n", FOC_CLBW);

    REAL Series_D_KP = FOC_CLBW * Ld;
    REAL Series_D_KI = R / Ld;
    REAL Series_Q_KP = FOC_CLBW * Lq;
    REAL Series_Q_KI = R / Lq;
    REAL Series_Speed_KP = omega_n * omega_n / (FOC_CLBW * K0);
    REAL Series_Speed_KFB = (2*zeta*omega_n*FOC_CLBW - 4*zeta*zeta*omega_n*omega_n) / (FOC_CLBW*K0);
    REAL Series_Speed_KI = ( FOC_CLBW - (sqrt(FOC_CLBW * FOC_CLBW - 4*FOC_CLBW*K0*Series_Speed_KFB)) ) * 0.5;


    PID_iD->Kp = Series_D_KP;
    PID_iQ->Kp = Series_Q_KP;
    PID_Speed->Kp = Series_Speed_KP;

    PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS;
    PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS;
    PID_Speed->Ki_CODE = Series_Speed_KI * Series_Speed_KP * VL_TS;
    PID_Speed->KFB = Series_Speed_KFB;

    PID_iD->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;

    #if PC_SIMULATION == TRUE
        printf("Series_D_KP = %f\n", Series_D_KP);
        printf("Series_D_KI = %f\n", Series_D_KI);
        printf("Series_Q_KP = %f\n", Series_Q_KP);
        printf("Series_Q_KI = %f\n", Series_Q_KI);
        printf("Series_Speed_KP = %f\n", Series_Speed_KP);
        printf("Series_Speed_KFB = %f\n", Series_Speed_KFB);
        printf("Series_Speed_KI = %f\n", Series_Speed_KI);
        if (FOC_CLBW - 4 * K0 * PID_Speed->KFB < 0){
            printf("can not do zero-pole cancellation\n");
        }else{
            printf(">>> Zero-pole cancellation can be done <<<\n");
        }
    #endif
}

void _user_wubo_TI_Tuner_Online(){
    if(debug.who_is_user == USER_WB2){
        PID_Speed->calc = (void (*)(Uint32)) wubo_PID_calc;
    }
    /* Tuned by debug */
    REAL CLBW_HZ = debug.CLBW_HZ;
    REAL delta = debug.delta;

    // TODO: 下面的psi_A在IM上应该不能用，因为我拿的是电机参数中原始的KE
    REAL Ld = d_sim.init.Ld;
    REAL Lq = d_sim.init.Lq;
    REAL R = d_sim.init.R;
    REAL Js = d_sim.init.Js;
    REAL npp = d_sim.init.npp;
    REAL KE = d_sim.init.KE;

    // motor parameters
    REAL KT = 1.5 * npp * KE;  // torque constant
    REAL K0 = KT / Js;  // motor constant
    
    REAL Series_D_KP = CLBW_HZ * 2 * M_PI * Ld;
    REAL Series_D_KI = R / Ld;
    REAL Series_Q_KP = CLBW_HZ * 2 * M_PI * Lq;
    REAL Series_Q_KI = R / Lq;
    REAL Series_Speed_KI = 2* M_PI * CLBW_HZ / (delta * delta); //THIS IS INTEGRAL GAIN
    REAL Series_Speed_KP = delta * Series_Speed_KI / KT * Js;

    PID_iD->Kp = Series_D_KP;
    PID_iQ->Kp = Series_Q_KP;
    PID_Speed->Kp = Series_Speed_KP;

    PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS;
    PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS;
    PID_Speed->Ki_CODE = Series_Speed_KI * Series_Speed_KP * VL_TS;

    PID_iD->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;

    // 应该可以删去，但我不放心有trick bug
    PID_Speed->KFB = 0.0;
}