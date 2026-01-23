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
    /* TEST*/
    REAL id_iq_amps[3];
    REAL ix_amps[3];
    REAL iy_amps[3];

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
    REAL varThetaOffset;
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

/* ===== 电流轮廓生成器 (Current Profile Generator) ===== */
#define MAX_IDIQ_AMPLITUDES 10   // 最大Id/Iq幅值数量
#define MAX_IX_AMPLITUDES 10     // 最大Ix幅值数量
#define MAX_IY_AMPLITUDES 10     // 最大Iy幅值数量
#define MAX_ANGLE_SEGMENTS 36    // 最大角度分段数量

typedef struct {
    // 配置参数
    REAL id_iq_amplitudes[MAX_IDIQ_AMPLITUDES];  // Id/Iq矢量幅值数组 (A)
    REAL ix_amplitudes[MAX_IX_AMPLITUDES];       // Ix轴电流幅值数组 (A)
    REAL iy_amplitudes[MAX_IY_AMPLITUDES];       // Iy轴电流幅值数组 (A)
    int num_idiq_amplitudes;                     // Id/Iq幅值数量
    int num_ix_amplitudes;                       // Ix幅值数量
    int num_iy_amplitudes;                       // Iy幅值数量
    int num_angle_segments;                      // 角度分段数量（360度等分）
    REAL duration_per_segment;                   // 每个角度段持续时间 (s) - 平台时间
    int cycles_per_segment;                      // 每个角度段的周期数（可选，若>0则覆盖duration_per_segment）
    int circles_per_amplitude;                   // 每个Id/Iq幅值重复的圆周次数（增加测量可信度）
    REAL sample_time;                            // 采样时间 (s), 例如 0.0001s for 10kHz
    
    // 状态变量
    int ix_index;                                // 当前Ix幅值索引
    int iy_index;                                // 当前Iy幅值索引
    int amp_index;                               // 当前Id/Iq幅值索引
    int circle_index;                            // 当前圆周索引（0到circles_per_amplitude-1）
    int angle_index;                             // 当前角度索引
    REAL segment_timer;                          // 当前段内计时器 (s)
    
    // 当前输出值
    REAL current_id;                             // 当前d轴电流 (A)
    REAL current_iq;                             // 当前q轴电流 (A)
    REAL current_ix;                             // 当前x轴电流 (A)
    REAL current_iy;                             // 当前y轴电流 (A)
    
    // 控制标志
    int is_initialized;                          // 是否已初始化
    int is_running;                              // 是否正在运行
    int is_completed;                            // 是否已完成所有序列
    
    // 暂停控制
    REAL pause_duration;                         // 完成后暂停时间 (s), 0表示不暂停
    REAL pause_timer;                            // 暂停计时器 (s)
    int is_pausing;                              // 是否处于暂停状态
    int auto_restart;                            // 暂停后是否自动重启 (1=是, 0=否)
    
} CurrentProfileGenerator;

// 函数声明
void CurrentProfileGenerator_Init(CurrentProfileGenerator *gen,
                                   REAL *id_iq_amps, int num_idiq,
                                   REAL *ix_amps, int num_ix,
                                   REAL *iy_amps, int num_iy,
                                   int num_angles,
                                   int circles_per_amplitude,
                                   REAL platform_duration,
                                   int cycles_per_segment,
                                   REAL sample_time,
                                   REAL pause_duration,
                                   int auto_restart);
                                   
void CurrentProfileGenerator_Reset(CurrentProfileGenerator *gen);

void CurrentProfileGenerator_Update(CurrentProfileGenerator *gen, 
                                     REAL *id_out, 
                                     REAL *iq_out,
                                     REAL *ix_out,
                                     REAL *iy_out);

int CurrentProfileGenerator_IsCompleted(CurrentProfileGenerator *gen);

#endif