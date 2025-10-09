#ifndef SIMUSER_YZK_H
#define SIMUSER_YZK_H

#include "ACMSim.h"


/* 电机参数，YZK悬浮电机专用 */


// // IIR
typedef struct {
        REAL b0, b1, b2;
        REAL a1, a2;
        REAL s1x, s2x;   // 状态（DF2-T）
        REAL s1y, s2y;   // 状态（DF2-T）
        // REAL x1s, x2s; // x[n-1], x[n-2]
        // REAL y1s, y2s, y0s; // y[n-1], y[n-2]
        // REAL lp_ch0;
        // REAL lp_ch1;
} biquad_t;

typedef struct {
    biquad_t lp_ch0;   /* 低通通道 x位移 */
    biquad_t lp_ch1;   /* 低通通道 y位移 */
} filters_t;

typedef struct {
        REAL prev_output;
        REAL alpha;
        REAL TAU;
        REAL de_raw_X; // delta error before filtering
        REAL de_raw_Y;
        REAL de_X; // delta error
        REAL de_Y;

} LPFs;

/* 控制器变量，YZK专用 */
struct YZK_2025_TIA_CTRL{
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
    REAL prev_error_I_alpha;
    REAL prev_error_I_beta;
    // REAL I_Term_prev_iD;
    // REAL I_Term_prev_iQ;
    // REAL K_INVERSE_iD;
    // REAL K_INVERSE_iQ;
    /* Misc*/
    REAL varTheta;
    REAL OutPrev_alpha;
    REAL OutPrev_beta;
    REAL Out_alpha;
    REAL Out_beta;
    struct {
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
        REAL K_X;
        REAL K_Y;
    } motor;
    // #define p4ps5_motor_suspension_parameters YZK_CTRL.motor
    // #define biquad_t YZK_CTRL.biq
    struct {
        REAL Ref;
        REAL Fbk;
        REAL Err;
        REAL ErrPrev;
        REAL P_Term; 
        REAL I_Term; 
        REAL D_Term;
        REAL OutNonSat;
        REAL OutLimit;
        REAL Out;
        REAL OutPrev; // for incremental pid
        REAL Kp;
        REAL Ki_CODE;
        REAL Kd;
        REAL SatDiff;
        REAL FbkPrev; 
        void (*calc)();
    } pids;
    LPFs LPFs;
    biquad_t biq;
    filters_t filt;
    // #define st_pid_regulator_suspension YZK_CTRL.pids
    // #define LowPassFilter YZK_CTRL.LPFs

};

extern struct YZK_2025_TIA_CTRL YZK_CTRL;






// ===== 低通滤波器结构 =====



/* 悬浮控制函数 */
void suspension_p4ps5_PD_Yaxis(REAL Y_Pos);
void suspension_p4ps5_PD_Xaxis(REAL X_Pos);
// extern struct YZK_CTRL;
// extern LowPassFilter *YZK_LPF;
// extern st_pid_regulator *YZK_PID;
// extern st_pid_regulator YZK_XXXXXX;
// extern p4ps5_motor_suspension_parameters *YZK_p4ps5;
// extern filters_t *g_filters;

// void _IIR_lpf(biquad_t *f_out, float fs, float fc, float Q);
// REAL biquad_process(biquad_t *f, float x);
// void biquad_init(biquad_t *f, float b0, float b1, float b2, float a1, float a2);
// void filters_init(void);
void init_YZK_ALL();
#endif