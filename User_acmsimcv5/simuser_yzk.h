#ifndef SIMUSER_YZK_H
#define SIMUSER_YZK_H

#include "ACMSim.h"

extern float test_sus;
extern BOOL BOOL_DIRECT_FIELD_TEST;
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
        REAL prev_output_x;
        REAL prev_output_y;
        REAL alpha_x;
        REAL alpha_y;
        REAL TAU_x;      // 
        REAL TAU_y;      // 
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
    REAL Err_X_1;
    REAL Err_Y_1;
    REAL Err_X_2;
    REAL Err_Y_2;
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
    REAL CMD_F_X;
    REAL CMD_F_Y;
    REAL CMD_F_X_prime;
    REAL CMD_F_Y_prime;
    REAL CMD_F_X_Kp;
    REAL CMD_F_X_Kd;
    REAL CMD_F_Y_Kp;
    REAL CMD_F_Y_Kd;
    REAL CMD_F_alpha;
    REAL CMD_F_beta;
    REAL CMD_I_alpha_1;
    REAL CMD_I_beta_1;
    REAL CMD_U_alpha_1;
    REAL CMD_U_beta_1;
    REAL Err_I_alpha_1;
    REAL Err_I_beta_1;
    REAL CMD_I_alpha_2;
    REAL CMD_I_beta_2;
    REAL CMD_U_alpha_2;
    REAL CMD_U_beta_2;
    REAL Err_I_alpha_2;
    REAL Err_I_beta_2;
    /* FOC */
    REAL disFbk_X;
    REAL disFbk_Y;
    REAL encFbk;
    REAL prev_error_X_1;
    REAL prev_error_Y_1;
    REAL prev_error_X_2;
    REAL prev_error_Y_2;
    REAL prev_error_I_alpha_1;
    REAL prev_error_I_beta_1;
    REAL prev_error_I_alpha_2;
    REAL prev_error_I_beta_2;
    // REAL I_Term_prev_iD;
    // REAL I_Term_prev_iQ;
    // REAL K_INVERSE_iD;
    // REAL K_INVERSE_iQ;
    REAL dc_bus_utilization_ratio_1;
    REAL dc_bus_utilization_ratio_2;
    /* Misc*/
    REAL varTheta;
    REAL OutPrev_alpha_1;
    REAL OutPrev_beta_1;
    REAL Out_alpha_1;
    REAL Out_beta_1;
    REAL Out_alpha_KI_1;
    REAL Out_beta_KI_1;
    REAL OutPrev_alpha_2;
    REAL OutPrev_beta_2;
    REAL Out_alpha_2;
    REAL Out_beta_2;
    REAL Out_alpha_KI_2;
    REAL Out_beta_KI_2;
    REAL KDLimit;
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
        REAL OutLimit_1;
        REAL OutLimit_alphaKI_1;
        REAL OutLimit_betaKI_1;
        REAL OutLimit_2;
        REAL OutLimit_alphaKI_2;
        REAL OutLimit_betaKI_2;
        REAL Out;
        REAL OutPrev; // for incremental pid
        REAL Kp_alpha_1;
        REAL Kp_beta_1;
        REAL Ki_CODE_alpha_1;
        REAL Ki_CODE_beta_1;
        REAL Kp_alpha_2;
        REAL Kp_beta_2;
        REAL Ki_CODE_alpha_2;
        REAL Ki_CODE_beta_2;
        REAL Kd;
        REAL SatDiff;
        REAL FbkPrev; 
        void (*calc)();
    } pids;
    LPFs LPFs_x;
    LPFs LPFs_y;
    biquad_t biq;
    filters_t filt;
    // #define st_pid_regulator_suspension YZK_CTRL.pids
    // #define LowPassFilter YZK_CTRL.LPFs

};

extern struct YZK_2025_TIA_CTRL YZK_CTRL;






// ===== 低通滤波器结构 =====



/* 悬浮控制函数 */
// void suspension_p4ps5_PD_Yaxis();
void suspension_p4ps5_PD_doubleaxis(REAL X_Pos, REAL Y_Pos);
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