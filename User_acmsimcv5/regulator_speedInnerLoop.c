#include "ACMSim.h"

// int wubo_debug[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

REAL _user_wubo_WC_Tuner_Part2(REAL zeta, REAL omega_n, REAL max_CLBW_PER_min_CLBW){

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

    #if PC_SIMULATION == TRUE
        printf("Dual Loop Theoritical Bandwidth is: \n");
        printf("FOC_CLBW = %f rad/s\n", FOC_CLBW);
        printf("FOC_CLBW = %f Hz\n", FOC_CLBW * ONE_OVER_2PI);
        printf("K0       = %f\n",       K0);
    #endif

    return FOC_CLBW, K0;
}

    /* Tuned by d_sim */
void _user_wubo_WC_Tuner(){
    REAL FOC_CLBW, K0;
    FOC_CLBW, K0 = _user_wubo_WC_Tuner_Part2(d_sim.user.zeta, d_sim.user.omega_n, d_sim.user.max_CLBW_PER_min_CLBW);

    #if PC_SIMULATION == TRUE
        printf("FOC_CLBW = %f rad/s\n", FOC_CLBW);
        printf("FOC_CLBW = %f Hz\n", FOC_CLBW * ONE_OVER_2PI);
        printf("K0       = %f\n",       K0);
        if (FOC_CLBW - 4 * K0 * PID_Speed->KFB < 0){
            printf("can not do zero-pole cancellation\n");
        }else{
            printf(">>> Zero-pole cancellation can be done <<<\n");
        }
    #endif
    
    PID_iD->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit  = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;
    // >>实验<<限幅的部分我放在pangu-c的main.c中的measurement函数里面，也就是说，测量此时的Vdc，然后根据Vdc觉得限幅的大小
}

void _user_wubo_WC_Tuner_Online(){
    /* Tuned by debug */
    REAL FOC_CLBW, K0;
    FOC_CLBW, K0 = _user_wubo_WC_Tuner_Part2(debug.zeta, debug.omega_n, debug.max_CLBW_PER_min_CLBW);
    // >>实验<<限幅的部分我放在pangu-c的main.c中的measurement函数里面，也就是说，测量此时的Vdc，然后根据Vdc觉得限幅的大小
}

void _user_wubo_TI_Tuner_Online(){
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

    // 应该可以删去，但我不放心有trick bug
    PID_Speed->KFB = 0.0;
}

void _user_wubo_SpeedInnerLoop_controller(st_pid_regulator *r){
        r->Err = r->Ref - r->Fbk;
        r->Out = r->OutPrev + \
                r->Kp * ( r->Err - r->ErrPrev ) \
                + r->Ki_CODE * r->Err; // - r->KFB_Term;
        //* 这里对Out做限幅是为了后面给OutPrev的值时，make sure it is able to cover the speed steady error carried by KFB
        if(r->Out > r->OutLimit + r->KFB_Term)
            r->Out = r->OutLimit + r->KFB_Term;
        else if(r->Out < -r->OutLimit + r->KFB_Term)
            r->Out = -r->OutLimit + r->KFB_Term;

        r->ErrPrev = r->Err; 
        r->OutPrev = r->Out;

        //* 存储控制器的各项输出
        r->P_Term += r->Kp * ( r->Err - r->ErrPrev );
        r->I_Term += r->Ki_CODE * r->Err;
        r->KFB_Term = 1 * r->KFB * r->Fbk;

        r->Out = r->Out - r->KFB_Term;
        if(r->Out > r->OutLimit)
            r->Out = r->OutLimit;
        else if(r->Out < -r->OutLimit)
            r->Out = -r->OutLimit;
}

void _user_wubo_Sweeping_Command(){
    #if PC_SIMULATION //* 先让扫频只在simulation中跑
        ACM.TLoad = 0; // 强制将负载设置为0
        if ((*CTRL).timebase > debug.CMD_SPEED_SINE_END_TIME){
            // next frequency
            debug.CMD_SPEED_SINE_HZ += debug.CMD_SPEED_SINE_STEP_SIZE;
            // next end time
            debug.CMD_SPEED_SINE_LAST_END_TIME = debug.CMD_SPEED_SINE_END_TIME;
            debug.CMD_SPEED_SINE_END_TIME += 1.0/debug.CMD_SPEED_SINE_HZ; // 1.0 Duration for each frequency
            // WUBO：这里扫频的思路是每隔一个frequency的时间，扫频信号提升1个Hz，例如1Hz的信号走1s，1/2Hz的信号走0.5s，1/3Hz的信号走0.33s，
            // 依次类推，所以单单从图上看时间是不能直接看出 Bandwidth到底是多少，但是可以用级数求和直接表示？
        }
        if (debug.CMD_SPEED_SINE_HZ > debug.CMD_SPEED_SINE_HZ_CEILING){
            (*CTRL).i->cmd_varOmega = 0.0; // 到达扫频的频率上限，速度归零
            (*CTRL).i->cmd_iDQ[1] = 0.0;
        }else{
            if (debug.bool_sweeping_frequency_for_speed_loop == TRUE){
                (*CTRL).i->cmd_varOmega = RPM_2_MECH_RAD_PER_SEC * debug.CMD_SPEED_SINE_RPM * sin(2* M_PI *debug.CMD_SPEED_SINE_HZ*((*CTRL).timebase - debug.CMD_SPEED_SINE_LAST_END_TIME));
            }else{
                // 让电机转起来，然后在d轴上电流扫频？
                //* 所以让电机转起来的原因是？？？
                debug.bool_Null_D_Control = FALSE; //确保Null iD不开启
                (*CTRL).i->cmd_varOmega = debug.set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
                (*CTRL).i->cmd_varOmega = 100 * RPM_2_MECH_RAD_PER_SEC;
                
                (*CTRL).i->cmd_iDQ[0]   = debug.CMD_CURRENT_SINE_AMPERE * sin(2* M_PI *debug.CMD_SPEED_SINE_HZ*((*CTRL).timebase - debug.CMD_SPEED_SINE_LAST_END_TIME));
            }
        }
    #endif
}


/* 20240910 教师节 下面的代码可能成为历史 史称910离散PI运动 */
// #define WUBO_INCREMENTAL_PID TRUE
// #define WUBO_INCREMENTAL_SPEED_PID TRUE
// #define DYNAMIC_CLAMPING_FOR_KFB FALSE

// #if WUBO_INCREMENTAL_PID
//     void wubo_PID_calc(st_pid_regulator *r){

//         r->Err = r->Ref - r->Fbk;
//         r->Out = r->OutPrev \
//                 + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki_CODE * r->Err;

//         // 限幅必须在这里做
//         if(r->Out > r->OutLimit)
//             r->Out = r->OutLimit;
//         else if(r->Out < -r->OutLimit)
//             r->Out = -r->OutLimit;

//         r->ErrPrev = r->Err; 
//         r->OutPrev = r->Out;
//     }
// #else
//     void wubo_PID_calc(st_pid_regulator *r){
//         #define DYNAMIC_CLAPMING TRUE
//         #define DYNAMIC_CLAPMING_WUBO TRUE

//         // 误差
//         r->Err = r->Ref - r->Fbk;
//         // 比例
//         r->P_Term = r->Err * r->Kp;
//         // 积分
//         r->I_Term += r->Err * r->Ki_CODE;
//         r->OutNonSat = r->I_Term;
//         // Inner Loop
//         r->KFB_Term = r->Fbk * r->KFB;

//         // 添加积分饱和特性
//         #if DYNAMIC_CLAPMING
//             #if DYNAMIC_CLAPMING_WUBO
//                 // dynamic clamping wubo
//                 if( r->I_Term > r->OutLimit - r->P_Term)
//                     r->I_Term = r->OutLimit - r->P_Term;
//                 else if( r->I_Term < -r->OutLimit + r->P_Term)
//                     r->I_Term =      -r->OutLimit + r->P_Term; // OutLimit is a positive constant
//             #else
//                 // dynamic clamping
//                 if( r->I_Term > r->OutLimit - r->Out)
//                     r->I_Term = r->OutLimit - r->Out;
//                 else if( r->I_Term < -r->OutLimit + r->Out)
//                     r->I_Term =      -r->OutLimit + r->Out; // OutLimit is a positive constant
//             #endif
//         #else
//             // static clamping
//             if( r->I_Term > r->OutLimit)
//                 r->I_Term = r->OutLimit; 
//             else if( r->I_Term < -r->OutLimit)
//                 r->I_Term = -r->OutLimit;
//         #endif

//         // 微分
//         // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

//         // 输出
//         r->Out = r->I_Term + r->P_Term - r->KFB_Term; // + r->D_Term
//         r->OutNonSat += r->P_Term; // + r->D_Term

//         // 输出限幅
//         if(r->Out > r->OutLimit)
//             r->Out = r->OutLimit;
//         else if(r->Out < -r->OutLimit)
//             r->Out = -r->OutLimit;

//         // 当前步误差赋值为上一步误差
//         r->ErrPrev = r->Err;
//         // 记录饱和输出和未饱和输出的差
//         r->SatDiff = r->Out - r->OutNonSat;
//     }
// #endif

// /* Speed Loop Discretized Controller*/
// #if WUBO_INCREMENTAL_SPEED_PID
//     void InnerLoopFeedback_calc(st_pid_regulator *r){

//         r->Err = r->Ref - r->Fbk;
//         r->Out = r->OutPrev + \
//                 r->Kp * ( r->Err - r->ErrPrev ) \
//                 + r->Ki_CODE * r->Err; // - r->KFB_Term;

//         if(r->Out > r->OutLimit + r->KFB_Term)
//             r->Out = r->OutLimit + r->KFB_Term;
//         else if(r->Out < -r->OutLimit + r->KFB_Term)
//             r->Out = -r->OutLimit + r->KFB_Term;

//         r->ErrPrev = r->Err; 
//         r->OutPrev = r->Out;

//         //* 存储控制器的各项输出
//         r->P_Term += r->Kp * ( r->Err - r->ErrPrev );
//         r->I_Term += r->Ki_CODE * r->Err;

//         #if 1
//             r->KFB_Term = 1 * r->KFB * r->Fbk;
//         #else
//             r->KFB_Term = r->KFB_Term_Prev + r->KFB * ( r->Fbk - r->FbkPrev );
//             r->FbkPrev       = r->Fbk;
//             r->KFB_Term_Prev = r->KFB_Term;
//         #endif

//         #if DYNAMIC_CLAMPING_FOR_KFB
//             if(r->KFB_Term > r->OutLimit - r->P_Term)
//                 r->KFB_Term = r->OutLimit - r->P_Term;
//             else if(r->KFB_Term < -r->OutLimit + r->P_Term)
//                 r->KFB_Term = -r->OutLimit + r->P_Term;
//         #endif

//         //* 确保KFB_Term加上后不会导致Out变符号，否则Out取0，但reason？ I dont Know yet 20240909
//         #if 0
//             if ( sign(r->KFB_Term) != sign(r->Out) ){
//                 r->Out = r->Out - r->KFB_Term;
//             }
//             if ( (fabs(r->KFB_Term) > fabs(r->Out)) && (sign(r->KFB_Term) == sign(r->Out)) ){
//                 r->Out = 0;
//             }
//             // if ( sign(r->KFB_Term) != sign(r->Out) ){
//             //     r->Out = r->Out - r->KFB_Term;
//             // }else if ( fabs(r->KFB_Term) > fabs(r->Out) ){
//             //     r->Out = 0;
//             // }else{
//             //     r->Out = r->Out - r->KFB_Term;
//             // }
//         #else
//             r->Out = r->Out - r->KFB_Term;
//         #endif

//         // # VERY BUGGY if you use incremental to implement inner loop 因为要对 OutPrev 限幅才对！
//         if(r->Out > r->OutLimit)
//             r->Out = r->OutLimit;
//         else if(r->Out < -r->OutLimit)
//             r->Out = -r->OutLimit;
//     }
// #else
//     void InnerLoopFeedback_calc(st_pid_regulator *r){
//         #define DYNAMIC_CLAPMING TRUE
//         #define DYNAMIC_CLAPMING_WUBO TRUE
//         #define ANTI_WINDUP_FOR_INNER_LOOP TRUE

//         // 误差
//         r->Err = r->Ref - r->Fbk;
//         // 比例
//         r->P_Term = r->Err * r->Kp;
//         // 积分
//         r->I_Term += r->Err * r->Ki_CODE;
//         r->OutNonSat = r->I_Term;
//         // Inner Loop
//         r->KFB_Term = r->Fbk * r->KFB;

//         // 添加积分饱和特性
//         #if DYNAMIC_CLAPMING
//             #if DYNAMIC_CLAPMING_WUBO
//                 // dynamic clamping wubo
//                 if( r->I_Term > r->OutLimit + r->KFB_Term - r->P_Term)
//                     r->I_Term = r->OutLimit + r->KFB_Term - r->P_Term;
//                 else if( r->I_Term < -r->OutLimit + r->KFB_Term + r->P_Term)
//                     r->I_Term =      -r->OutLimit + r->KFB_Term + r->P_Term; // OutLimit is a positive constant
//             #else   
//                 // dynamic clamping
//                 if( r->I_Term > r->OutLimit + r->KFB_Term - r->Out)
//                     r->I_Term = r->OutLimit + r->KFB_Term - r->Out;
//                 else if( r->I_Term < -r->OutLimit + r->KFB_Term + r->Out)
//                     r->I_Term =      -r->OutLimit + r->KFB_Term + r->Out; // OutLimit is a positive constant
//             #endif
//         #else
//             // static clamping
//             if( r->I_Term > r->OutLimit)
//                 r->I_Term = r->OutLimit; 
//             else if( r->I_Term < -r->OutLimit)
//                 r->I_Term = -r->OutLimit;
//         #endif

//         // #if ANTI_WINDUP_FOR_INNER_LOOP
//         // if ( )

//         // 微分
//         // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

//         // 输出
//         r->Out = r->P_Term + r->I_Term - 1 * r->KFB_Term; // + r->D_Term
//         r->OutNonSat += r->P_Term; // + r->D_Term

//         // 输出限幅
//         if(r->Out > r->OutLimit)
//             r->Out = r->OutLimit;
//         else if(r->Out < -r->OutLimit)
//             r->Out = -r->OutLimit;

//         // 当前步误差赋值为上一步误差
//         r->ErrPrev = r->Err;
//         // 记录饱和输出和未饱和输出的差
//         r->SatDiff = r->Out - r->OutNonSat;
//     }
// #endif

// void _user_wubo_controller(){
//     if ((*CTRL).s->the_vc_count++ >= SPEED_LOOP_CEILING){
//         (*CTRL).s->the_vc_count = 1;
//         PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
//         PID_Speed->Fbk = (*CTRL).i->varOmega;
//         if(debug.who_is_user == USER_BEZIER){
//             control_output(PID_Speed, &BzController);
//         }else if(debug.who_is_user == USER_WB){
//             PID_Speed->calc(PID_Speed);
//         }else if(debug.who_is_user == USER_CJH){
//             PID_Speed->calc(PID_Speed);
//         }else{
//             debug.error = 998;
//         }
//         // Get iQ Command
//         (*CTRL).i->cmd_iDQ[1] = PID_Speed->Out; 
//     }

//     // 帕克变换
//     (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
//     (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
//     (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
//     (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

//     /* 更新依赖于dq轴电流的物理量 */
//     (*CTRL).i->Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0]) * (*CTRL).i->iDQ[1];     // 转矩 For luenberger position observer for HFSI
//     (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0]) * (*CTRL).i->cmd_iDQ[1];
//     MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];


//     /* 启动D_Cuurent = 0A */
//     if(debug.bool_Null_D_Control == TRUE) (*CTRL).i->cmd_iDQ[0] = 0;

//     /* 速度双环的电流环 */
//     // d-axis
//     PID_iD->Fbk = (*CTRL).i->iDQ[0];
//     PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
//     PID_iD->calc(PID_iD);
//     // q-axis
//     PID_iQ->Fbk = (*CTRL).i->iDQ[1];
//     PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
//     PID_iQ->calc(PID_iQ);

//     /* Current Loop DQ Decoupling and Saturation*/
//     REAL decoupled_d_axis_voltage;
//     REAL decoupled_q_axis_voltage;
//     if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
//         decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
//         decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
//     }else{
//         decoupled_d_axis_voltage = PID_iD->Out;
//         decoupled_q_axis_voltage = PID_iQ->Out;
//     }
//     if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
//     else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
//     if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
//     else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;
//     (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
//     (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;



//     /// 7. 反帕克变换
//     // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
//     // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
//     // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
//     (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
//     (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
//     (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
//     (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
//     (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
//     (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

//     // (*CTRL).o->cmd_uAB[0+2] = MT2A((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);
//     // (*CTRL).o->cmd_uAB[1+2] = MT2B((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);

//     /* 8. 补偿逆变器非线性 */
//     // main_inverter_voltage_command(TRUE);
//     wubo_inverter_Compensation( (*CTRL).i->iAB );
// }
