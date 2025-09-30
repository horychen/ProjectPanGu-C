#ifndef SIMUSER_YZK_H
#define SIMUSER_YZK_H

#include "ACMSim.h"


/* 电机参数，YZK悬浮电机专用 */
typedef struct {
    REAL alpha;
    REAL alpha_inv;

    // YZK的魔改高级电机
    REAL npp;
    REAL npp_inv;
    REAL Js;
    REAL Js_inv;
    REAL M_rotor;  // rotor mass
    REAL ge;       // air gap length
    REAL mu_0;     // 真空磁导率
    REAL S;        // alpha正对的面积
    REAL N_alpha;  // alpha线圈匝数
    REAL N_beta;   // beta线圈匝数
    REAL g;
} p4ps5_motor_suspension_parameters;



/* 控制器变量，YZK专用 */
typedef struct {
    /* XY方向，位置环 */
    REAL CMD_X;
    REAL CMD_Y;
    REAL Err_X;
    REAL Err_Y;
    REAL KP_X;
    REAL KP_Y;
    REAL KI_X;
    REAL KI_Y;
    REAL KD_X;
    REAL KD_Y;
    /* 磁链 */
    REAL CMD_psi_alpha;
    REAL CMD_psi_beta;
    REAL Err_psi_alpha;
    REAL Err_psi_beta;
    REAL CMD_I_alpha;
    REAL CMD_I_beta;
    REAL CMD_U_alpha;
    REAL CMD_U_beta;
    REAL Err_I_alpha;
    REAL Err_I_beta;
    /* FOC */
    REAL disFbk_X;
    REAL disFbk_Y;
    REAL encFbk;
    REAL prev_error_X;
    REAL prev_error_Y;
    // REAL I_Term_prev_iD;
    // REAL I_Term_prev_iQ;
    // REAL K_INVERSE_iD;
    // REAL K_INVERSE_iQ;
    /* Misc*/
    REAL varTheta;
    REAL Out;

    
} YZK_2025_TIA_CTRL;

void _init_YZK_2025_ALL();




// typedef struct {
//     float32 Ref;
//     float32 Fbk;
//     float32 Err;
//     float32 ErrPrev;
//     float32 P_Term; 
//     float32 I_Term; 
//     float32 D_Term;
//     float32 OutNonSat;
//     float32 OutLimit;
//     float32 Out;
//     float32 OutPrev; // for incremental pid
//     float32 Kp;
//     float32 Ki_CODE;
//     float32 Kd;
//     float32 SatDiff;
//     float32 FbkPrev;
//     void (*calc)();
// } st_pid_regulator;



// ===== 低通滤波器结构 =====
typedef struct {
    REAL prev_output;
    REAL alpha;
    REAL TAU;
    REAL de_raw; // delta error before filtering
    REAL de; // delta error
} LowPassFilter;


/* 悬浮控制函数 */
void suspension_p4ps5_PD_Yaxis(REAL Y_Pos);
void suspension_p4ps5_PD_Xaxis(REAL X_Pos);
extern YZK_2025_TIA_CTRL *YZK_CTRL;
extern LowPassFilter *YZK_LPF;
extern st_pid_regulator *YZK_PID;
// extern st_pid_regulator YZK_XXXXXX;
extern p4ps5_motor_suspension_parameters *YZK_p4ps5;


#endif