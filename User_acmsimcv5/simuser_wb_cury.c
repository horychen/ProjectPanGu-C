#include "simuser_wb_cury.h"
#include "ACMSim.h"



#if WHO_IS_USER == USER_WB

#if !PC_SIMULATION
    extern Uint32 position_count_CAN_ID0x01_fromCPU2;
    extern Uint32 position_count_CAN_ID0x03_fromCPU2;
    extern Uint32 position_count_SCI_hip_fromCPU2;
    extern Uint32 position_count_SCI_shank_fromCPU2;
    extern REAL target_position_cnt;
    extern REAL target_position_cnt_shank;
    extern REAL target_position_cnt_hip;
#endif

CURYCONTROLLER cury_controller = {
        .CONTROLLER_TYPE = -1,
        .IECON_HEIGHT = 0.72,
        .L1 = 0.4,
        .L2 = 0.39495,
        .theta1 = INIT_THETA1,
        .theta2 = INIT_THETA2,
        .dot_theta1 = 0.0,
        .dot_theta2 = 0.0,
        .T = 1.5,
        .T_inv = 1.0 / 1.5,
        .height_limit = {0.68, 0.78},
        .C = {{0.0, 0.0}, {0.1, 1.0}, {0.21, 0.93}, {1.0, 1.0}},
        .order = BEZIER_ORDER,
        .legBouncingSpeed = 50,
        .hipBouncingFreq = 10,
        .legBouncingIq = 2,
        .hipBouncingIq = 1.5,
        .bool_TEMP = TRUE
    };



/* 线性差值（偏向于公共功能的函数） */
REAL linearInterpolate(REAL x, REAL x1, REAL x2, REAL y1, REAL y2){
    return y1 + (x - x1) / (x2 - x1) * (y2 - y1);
}

REAL hip_shank_angle_to_can(REAL angle, int type){
    if (type == HIP_TYPE){
        return linearInterpolate(angle, HIP_MIN, HIP_MAX, CAN01_MIN, CAN01_MAX);
    }
    else{
        return linearInterpolate(angle, SHANK_MIN, SHANK_MAX, CAN03_MIN, CAN03_MAX);
    }
}

/* 获取关节角度的期望值（reference generation）*/
void calc_dot_theta_from_height(REAL height){
    REAL theta1_front, theta2_front;
    // 余弦定理！
    theta1_front = acos((pow(cury_controller.L1, 2) + pow(height, 2) - pow(cury_controller.L2, 2)) / (2 * cury_controller.L1 * height));
    theta2_front = acos((pow(cury_controller.L2, 2) + pow(height, 2) - pow(cury_controller.L1, 2)) / (2 * cury_controller.L2 * height));
    cury_controller.dot_theta1 = (theta1_front - cury_controller.theta1) * (cury_controller.T_inv * 1e3 );
    cury_controller.dot_theta2 = (theta2_front - cury_controller.theta2) * (cury_controller.T_inv * 1e3 );
}

void calc_theta_from_height(REAL height){
    cury_controller.theta1 = acos((pow(cury_controller.L1, 2) + pow(height, 2) - pow(cury_controller.L2, 2)) / (2 * cury_controller.L1 * height));
    cury_controller.theta2 = acos((pow(cury_controller.L2, 2) + pow(height, 2) - pow(cury_controller.L1, 2)) / (2 * cury_controller.L2 * height));
}

void reset_position(){
    calc_theta_from_height(cury_controller.height_limit[1]);
}

void linear_controller(REAL t){
    REAL height, height_front, t_front;

    int period = (int)(t * cury_controller.T_inv);
    t = t - period * cury_controller.T;
    // 上面的t可以换一种不用除法的写法(但我的while循环是一个傻逼玩意！不如用空间换时间)
    // while (t > cury_controller.T){
        // t = t - cury_controller.T;
    // }
    
    // cury做保持末端点(end point)与base连线，垂直于ground的上下竖直运动
    if (period % 2 == 0){
        height = cury_controller.height_limit[0] + (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t * cury_controller.T_inv;
    }else{
        height = cury_controller.height_limit[1] - (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t * cury_controller.T_inv;
    }

    // 求ref的导数，使位置闭环无static error
    t_front  = t + (cury_controller.T * 1e-3);
    period  += (int)(t_front * cury_controller.T_inv);
    t_front -= ((int)(t_front * cury_controller.T_inv)) * cury_controller.T;
    if (period % 2 == 0){
        height_front = cury_controller.height_limit[0] + (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t_front * cury_controller.T_inv;
    }else{
        height_front = cury_controller.height_limit[1] - (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t_front * cury_controller.T_inv;
    }

    calc_dot_theta_from_height(height_front);
    calc_theta_from_height(height);
}

void sinusoidal_controller(REAL t){
    REAL height = (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * 0.5 * sin(2 * M_PI * t * cury_controller.T_inv) + (cury_controller.height_limit[1] + cury_controller.height_limit[0]) * 0.5;
    REAL height_front = (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * 0.5 * sin(2 * M_PI * (t + (cury_controller.T / 1000)) * cury_controller.T_inv) + (cury_controller.height_limit[1] + cury_controller.height_limit[0]) * 0.5;
    
    calc_theta_from_height(height);
    calc_dot_theta_from_height(height_front);
}

void get_bezier_points(){
    REAL t;
    int i, j;
    for (i = 0; i <= BEZIER_TRACE_SIZE; i++){
        t = (REAL)i / BEZIER_TRACE_SIZE;
        cury_controller.bezier_trace[i][0] = 0.0;
        cury_controller.bezier_trace[i][1] = 0.0;
        for (j = 0; j < cury_controller.order; j++){
            cury_controller.bezier_trace[i][0] += cury_controller.C[j][0] * pow(1 - t, cury_controller.order - 1 - j) * pow(t, j);
            cury_controller.bezier_trace[i][1] += cury_controller.C[j][1] * pow(1 - t, cury_controller.order - 1 - j) * pow(t, j);
        }
    }
    for (i = 0; i < BEZIER_TRACE_SIZE; i++){
        for (j = i + 1; j < BEZIER_TRACE_SIZE; j++){
            if (cury_controller.bezier_trace[i][0] > cury_controller.bezier_trace[j][0]){
                REAL temp[2];
                temp[0] = cury_controller.bezier_trace[i][0];
                temp[1] = cury_controller.bezier_trace[i][1];
                cury_controller.bezier_trace[i][0] = cury_controller.bezier_trace[j][0];
                cury_controller.bezier_trace[i][1] = cury_controller.bezier_trace[j][1];
                cury_controller.bezier_trace[j][0] = temp[0];
                cury_controller.bezier_trace[j][1] = temp[1];
            }
        }
    }
}

REAL bezier_linear_interpoolation(REAL t){
    int i;
    for (i = 0; i < BEZIER_TRACE_SIZE; i++){
        if (cury_controller.bezier_trace[i][0] <= t && t <= cury_controller.bezier_trace[i + 1][0]){
            REAL height = (cury_controller.bezier_trace[i][1] +
                           (cury_controller.bezier_trace[i + 1][1] - cury_controller.bezier_trace[i][1]) * (t - cury_controller.bezier_trace[i][0]) / (cury_controller.bezier_trace[i + 1][0] - cury_controller.bezier_trace[i][0])) *
                              (cury_controller.height_limit[1] - cury_controller.height_limit[0]) +
                          cury_controller.height_limit[0];
            return height;
        }
    }
    return cury_controller.height_limit[1];
}

void bezier_controller(REAL t){
    REAL height, height_front, t_front;
    int period = (int)(t / cury_controller.T);
    t = t - period * cury_controller.T;
    if (period % 2 == 0){
        height = bezier_linear_interpoolation(t / cury_controller.T);
    }
    else{
        height = cury_controller.height_limit[1] - (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t / cury_controller.T;
    }
    t_front = t + (cury_controller.T / 1000);
    period = period + (int)(t_front / cury_controller.T);
    t_front -= (int)(t_front / cury_controller.T) * cury_controller.T;
    if (period % 2 == 0){
        height_front = bezier_linear_interpoolation(t_front / cury_controller.T);
    }
    else{
        height_front = cury_controller.height_limit[1] - (cury_controller.height_limit[1] - cury_controller.height_limit[0]) * t_front / cury_controller.T;
    }
    calc_theta_from_height(height);
    calc_dot_theta_from_height(height_front);
}

void height_controller(REAL height){
    calc_theta_from_height(height);
    cury_controller.dot_theta1 = 0.0;
    cury_controller.dot_theta2 = 0.0;
}

void run_iecon_main(Uint64 t){
    REAL t_trans = (double)(t*0.0001);
    switch (cury_controller.CONTROLLER_TYPE){
    case 0:
        linear_controller(t_trans);
        break;
    case 1:
        sinusoidal_controller(t_trans);
        break;
    case 2:
        bezier_controller(t_trans);
        break;
    case -1:
        reset_position();
        break;
    case 3:
        height_controller(cury_controller.IECON_HEIGHT);
        break;
    default:
        break;
    }
}

/* 纯控制部分 */
void Cury_call_position_loop_controller(){

    // get the ref position 这里的run_iecon_main实际只是在生成轨迹，没有任何控制的部分！
    #if PC_SIMULATION == FALSE
        run_iecon_main((*CTRL).timebase_counter);
    #else
        run_iecon_main((*CTRL).timebase * 1e4);
    #endif

    // run the control
    if (d_sim.user.positionLoopType == TWOMOTOR_POSITION_CONTROL)    control_two_motor_position();   // 1
    else if (d_sim.user.positionLoopType == SINGLE_POSITION_CONTROL) control_single_motor_position();// 2
    else if (d_sim.user.positionLoopType == SHANK_LOOP_RUN)          run_shank_loop();               // 3
    else if (d_sim.user.positionLoopType == HIP_LOOP_RUN)            run_hip_loop();                 // 4
    else if (d_sim.user.positionLoopType == BOTH_LOOP_RUN)           run_both_loop();                // 5
    else if (d_sim.user.positionLoopType == IMPEDANCE_CONTROL)       run_impedance_control();        // 6
}

void control_two_motor_position(){
    if (axisCnt == 0){
        #if NUMBER_OF_AXES == 2
            PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
            PID_Position->Ref = hip_shank_angle_to_can(
                cury_controller.theta1 + cury_controller.theta2,
                SHANK_TYPE);
        #else
            PID_Position->Fbk = 1;
            PID_Position->Ref = hip_shank_angle_to_can(
                cury_controller.theta1 + cury_controller.theta2,
                SHANK_TYPE);
        #endif
    }
    if (axisCnt == 1){
        #if NUMBER_OF_AXES == 2
                PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
                PID_Position->Ref = hip_shank_angle_to_can(
                    cury_controller.theta1,
                    HIP_TYPE);
        #else
            PID_Position->Fbk = 1;
            PID_Position->Ref = hip_shank_angle_to_can(
                cury_controller.theta1 + cury_controller.theta2,
                SHANK_TYPE);
        #endif
    }
    // 位置环
    // 长弧和短弧，要选短的。
    #if PC_SIMULATION == FALSE
        PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
        if (PID_Position->Err > (CAN_QMAX * 0.5)){
            PID_Position->Err -= CAN_QMAX;
        }
        if (PID_Position->Err < -(CAN_QMAX * 0.5)){
            PID_Position->Err += CAN_QMAX;
        }
        if (axisCnt == 0){
        #if NUMBER_OF_AXES == 2
            PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * (cury_controller.dot_theta1+cury_controller.dot_theta2) * 60 * 0.15915494309189535;
        #endif
        }
        if (axisCnt == 1){
        #if NUMBER_OF_AXES == 2
            PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * cury_controller.dot_theta1* 60 * 0.15915494309189535;
        #endif
        }
        if (PID_Position->Out > PID_Position->OutLimit){
            PID_Position->Out = PID_Position->OutLimit;
        }
        if (PID_Position->Out < -PID_Position->OutLimit){
            PID_Position->Out = -PID_Position->OutLimit;
        }
        if (axisCnt == 1)
            (*CTRL).i->cmd_varOmega = -PID_Position->Out;
        else
            (*CTRL).i->cmd_varOmega = PID_Position->Out;
    #endif
}

void control_single_motor_position(){
    #if PC_SIMULATION == FALSE
        if (axisCnt == 0){
            #if NUMBER_OF_AXES == 2
                    PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
                    PID_Position->Ref = target_position_cnt_shank;
            #endif
        }
        if (axisCnt == 1){
            #if NUMBER_OF_AXES == 2
                    PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
                    PID_Position->Ref = target_position_cnt_hip;
            #endif
        }
    #else
        // 将(*CTRL).i->varTheta的值映射到绝对值编码器上
        REAL vartheta_2_encoder = fmod((*CTRL).i->varTheta, 2 * M_PI);
        if (vartheta_2_encoder < 0) {
            vartheta_2_encoder += 2 * M_PI;
        }
        vartheta_2_encoder = vartheta_2_encoder  * ONE_OVER_2PI * CAN_QMAX;

        PID_Position->Fbk = vartheta_2_encoder;
        PID_Position->Ref = hip_shank_angle_to_can(
                cury_controller.theta1 + cury_controller.theta2,
                SHANK_TYPE);
        (*CTRL).i->cmd_varTheta = PID_Position->Ref * CAN_QMAX_INV * 2 * M_PI;
    #endif
    
    PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
    if (PID_Position->Err > (CAN_QMAX * 0.5)){
        PID_Position->Err -= CAN_QMAX;
    }
    if (PID_Position->Err < -(CAN_QMAX * 0.5)){
        PID_Position->Err += CAN_QMAX;
    }

    PID_Position->Out = PID_Position->Err * PID_Position->Kp * CAN_QMAX_INV * 2 * M_PI;
    if (PID_Position->Out > PID_Position->OutLimit){
        PID_Position->Out = PID_Position->OutLimit;
    }
    if (PID_Position->Out < -PID_Position->OutLimit){
        PID_Position->Out = -PID_Position->OutLimit;
    }
    if (axisCnt == 1)
        (*CTRL).i->cmd_varOmega = -PID_Position->Out;
    else
        (*CTRL).i->cmd_varOmega = PID_Position->Out;
}

void run_shank_loop(){ // shank motor only
    #if PC_SIMULATION == FALSE
        if (axisCnt == 0){
            // Axis->flag_overwrite_theta_d = FALSE;

            //            Axis->Set_current_loop = TRUE;
            //            if (position_count_CAN_ID0x03_fromCPU2 > 62000)
            //            {
            //                Axis->Set_manual_current_iq = -1;
            //            }
            //            else if (position_count_CAN_ID0x03_fromCPU2 < 33000)
            //            {
            //                Axis->Set_manual_current_iq = 1;
            //            }
            if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MIN){
                (*CTRL).i->cmd_varOmega = -cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // *ref_speed_RPM = -cury_controller.legBouncingSpeed;
            }
            else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MAX){
                (*CTRL).i->cmd_varOmega = cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // *ref_speed_RPM = cury_controller.legBouncingSpeed;
            }
        }
        if (axisCnt == 1){
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN){
                (*CTRL).i->cmd_varOmega = cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // *ref_speed_RPM = cury_controller.legBouncingSpeed;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX){
                (*CTRL).i->cmd_varOmega = -cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // *ref_speed_RPM = -cury_controller.legBouncingSpeed;
            }
        }
    #endif
}

void run_hip_loop(){ 
    // hip motor only
#if NUMBER_OF_AXES == 2
    if (axisCnt == 1){
        if (cury_controller.bool_TEMP == TRUE){
            // Axis->flag_overwrite_theta_d = FALSE;
            // Axis->Set_current_loop = TRUE;
            debug->mode_select = 3; // 3 -> onlyFOC
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN){
                (*CTRL).i->iDQ[1] = cury_controller.hipBouncingIq;
                // Axis->Set_manual_current_iq = hipBouncingIq;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX){
                (*CTRL).i->iDQ[1]= -cury_controller.hipBouncingIq;
                // Axis->Set_manual_current_iq = -hipBouncingIq;
            }
        }
        else{
            // Axis->flag_overwrite_theta_d = FALSE;
            // Axis->Set_current_loop = FALSE;
            debug->mode_select = 4; // 4 -> speed loop with FOC
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN){
                (*CTRL).i->cmd_varOmega = -cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // Axis->Set_manual_rpm = -cury_controller.legBouncingSpeed;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX){
                (*CTRL).i->cmd_varOmega = cury_controller.legBouncingSpeed * RPM_2_MECH_RAD_PER_SEC;
                // Axis->Set_manual_rpm = cury_controller.legBouncingSpeed;
            }
        }

        //
        //               Axis_2.flag_overwrite_theta_d = TRUE;
        //                Axis_2.Set_current_loop = TRUE;
        //                Axis_2.Set_manual_current_iq = 5;
        //                if (position_count_CAN_ID0x01_fromCPU2 > 58000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = 10;
        //                }
        //                else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = -10;
        //                }
    }
#else
    // Axis->flag_overwrite_theta_d = TRUE;
    // Axis->Set_current_loop = TRUE;
    // Axis->Set_manual_current_iq = 5;
    // if (position_count_CAN_ID0x01_fromCPU2 > 58000)
    // {
    //     Axis->Overwrite_Current_Frequency = hipBouncingFreq;
    // }
    // else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
    // {
    //     Axis->Overwrite_Current_Frequency = -hipBouncingFreq;
    // }
#endif
}

void run_both_loop(){
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0)
    {
        // CAN03越大，小腿越伸出，legBouncingIq, rpm为正数，小腿伸出
        // CAN03越小，小腿越收起，legBouncingIq, rpm为负数，小腿收起
        // Axis_1.flag_overwrite_theta_d = FALSE;
        // Axis_1.Set_current_loop = TRUE;
        debug->mode_select = 3; // 3 -> onlyFOC
        if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MAX){
            (*CTRL).i->iDQ[1] = cury_controller.legBouncingIq; // CAN03太大，小腿伸出过头，需要iq为负数，收起小腿
            // Axis_1.Set_manual_current_iq = legBouncingIq; // CAN03太大，小腿伸出过头，需要iq为负数，收起小腿
        }
        else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MIN){
            (*CTRL).i->iDQ[1] = -cury_controller.legBouncingIq;
            // Axis_1.Set_manual_current_iq = -legBouncingIq;
        }
    }
    // 大腿电机
    if (axisCnt == 1)
    {
        // CAN01越小,大腿越抬起,hipBouncingIq为正数，大腿抬起
        // CAN01越大,大腿越放下,hipBouncingIq为负数，大腿放下
        // Axis_2.flag_overwrite_theta_d = FALSE;
        // Axis_2.Set_current_loop = TRUE;
        debug->mode_select = 3; // 3 -> onlyFOC
        if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MAX){
            (*CTRL).i->iDQ[1] = cury_controller.legBouncingIq;
            // Axis_2.Set_manual_current_iq = hipBouncingIq; // CAN01太大,大腿放下过多,需要iq为正数,抬起大腿
        }
        else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MIN){
            (*CTRL).i->iDQ[1] = -cury_controller.legBouncingIq;
            // Axis_2.Set_manual_current_iq = -hipBouncingIq;
        }
    }
#endif
}

void run_impedance_control(){
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0){
        // Axis_1.flag_overwrite_theta_d = FALSE;
        // Axis_1.Set_current_loop = TRUE;
        debug->mode_select = 3; // 3 -> onlyFOC
        // Axis_1.Set_manual_current_iq = Impendence_Control_cal(
        (*CTRL).i->iDQ[1] = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
    if (axisCnt == 1){
        // Axis_2.flag_overwrite_theta_d = FALSE;
        // Axis_2.Set_current_loop = TRUE;
        debug->mode_select = 3; // 3 -> onlyFOC
        // Axis_2.Set_manual_current_iq = Impendence_Control_cal(
        (*CTRL).i->iDQ[1] = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
#endif
}



#endif


































