#ifndef SIMUSER_YZK_H
#define SIMUSER_YZK_H

#include "ACMSim.h"


/* 电机参数，YZK悬浮电机专用 */


// // IIR
typedef struct {
        float b0, b1, b2;
        float a1, a2;
        float s1, s2;   // 状态（DF2-T）
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
        REAL de_raw; // delta error before filtering
        REAL de; // delta error
} LPFs;

/* 控制器变量，YZK专用 */
struct YZK_2025_TIA_CTRL{
    /* XY方向，位置环 */
    double CMD_X;
    double CMD_Y;
    double Err_X;
    double Err_Y;
    double KP_X;
    double KP_Y;
    double KI_X;
    double KI_Y;
    double KD_X;
    double KD_Y;
    /* 磁链 */
    double CMD_psi_alpha;
    double CMD_psi_beta;
    double Err_psi_alpha;
    double Err_psi_beta;
    double CMD_I_alpha;
    double CMD_I_beta;
    double CMD_U_alpha;
    double CMD_U_beta;
    double Err_I_alpha;
    double Err_I_beta;
    /* FOC */
    double disFbk_X;
    double disFbk_Y;
    double encFbk;
    double prev_error_X;
    double prev_error_Y;
    // double I_Term_prev_iD;
    // double I_Term_prev_iQ;
    // double K_INVERSE_iD;
    // double K_INVERSE_iQ;
    /* Misc*/
    double varTheta;
    double Out;

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
        REAL K;
    } motor;
    // #define p4ps5_motor_suspension_parameters YZK_CTRL.motor
    // #define biquad_t YZK_CTRL.biq
    struct {
        float32 Ref;
        float32 Fbk;
        float32 Err;
        float32 ErrPrev;
        float32 P_Term; 
        float32 I_Term; 
        float32 D_Term;
        float32 OutNonSat;
        float32 OutLimit;
        float32 Out;
        float32 OutPrev; // for incremental pid
        float32 Kp;
        float32 Ki_CODE;
        float32 Kd;
        float32 SatDiff;
        float32 FbkPrev;
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