#include "ACMSim.h"


YZK_2025_TIA_CTRL *YZK_CTRL;
LowPassFilter *YZK_LPF;
st_pid_regulator *YZK_PID;
p4ps5_motor_suspension_parameters *YZK_p4ps5;
filters_t *g_filters;


// K = tan(pi * fc / fs)
// norm = 1 + K/Q + K^2
// b0 = K^2 / norm
// b1 = 2*b0
// b2 = b0
// a1 = 2*(K^2 - 1) / norm
// a2 = (1 - K/Q + K^2) / norm

    /* IIR */
static const REAL LP0_b0 = 0.04613180207f;
static const REAL LP0_b1 = 0.09226360415f;
static const REAL LP0_b2 = 0.04613180207f;
static const REAL LP0_a1 = -1.30728502829f;
static const REAL LP0_a2 = 0.49181223659f;

static const REAL LP1_b0 = 0.04613180207f;
static const REAL LP1_b1 = 0.09226360415f;
static const REAL LP1_b2 = 0.04613180207f;
static const REAL LP1_a1 = -1.30728502829f;
static const REAL LP1_a2 = 0.49181223659f;

static const REAL KP_X = 0.2;
static const REAL KP_Y = 0.5;
// static const REAL YZK_CTRL->KI_X = 0.0;
// static const REAL YZK_CTRL->KI_Y = 0.0;
// static const REAL YZK_CTRL->KD_X = 0.0;
// static const REAL YZK_CTRL->KD_Y = 0.0;
/* Initialising */
void _init_YZK_ALL(){
    /* XY方向 */
    YZK_CTRL->CMD_X = 1722575;
    YZK_CTRL->CMD_Y = 1712097;
    YZK_CTRL->Err_X = 0.0;
    YZK_CTRL->Err_Y = 0.0;
    YZK_CTRL->KP_X = 0.2;
    YZK_CTRL->KP_Y = 0.2;
    YZK_CTRL->KI_X = 0.0;
    YZK_CTRL->KI_Y = 0.0;
    YZK_CTRL->KD_X = 0.0;
    YZK_CTRL->KD_Y = 0.0;
    /* 磁链 */
    YZK_CTRL->CMD_psi_alpha = 0.0;
    YZK_CTRL->CMD_psi_beta  = 0.0;
    YZK_CTRL->Err_psi_alpha = 0.0;
    YZK_CTRL->Err_psi_beta  = 0.0;
    YZK_CTRL->CMD_I_alpha   = 0.0;
    YZK_CTRL->CMD_I_beta    = 0.0;
    YZK_CTRL->CMD_U_alpha   = 0.0;
    YZK_CTRL->CMD_U_beta    = 0.0;
    YZK_CTRL->Err_I_alpha   = 0.0;
    YZK_CTRL->Err_I_beta    = 0.0;
    // YZK_CTRL->I_Term_prev_iD = 0.0;
    // YZK_CTRL->I_Term_prev_iQ = 0.0;
    // YZK_CTRL->K_INVERSE_iD = 0.0;
    // YZK_CTRL->K_INVERSE_iQ = 0.0;

    /* FOC */
    YZK_CTRL->disFbk_X = 0.0;
    YZK_CTRL->disFbk_Y = 0.0;
    YZK_CTRL->encFbk = 0.0;
    YZK_CTRL->prev_error_X = 0.0;
    YZK_CTRL->prev_error_Y = 0.0;

    YZK_PID->Kp = 0.0;
    YZK_PID->Ki_CODE = 0.0;
    YZK_PID->Kd = 0.0;
    YZK_PID->OutLimit = 25;

    /* 参数初始化 */
    YZK_p4ps5->npp = 5;
    YZK_p4ps5->npp_inv = 0.2;
    YZK_p4ps5->Js = 1;
    YZK_p4ps5->Js_inv = 1;
    YZK_p4ps5->M_rotor = 150;  // rotor mass
    YZK_p4ps5->ge = 6.5;       // air gap length
    YZK_p4ps5->mu_0 = 4*M_PI*1e-7;     // 真空磁导率
    YZK_p4ps5->S = 1;        // alpha正对的面积
    YZK_p4ps5->N_alpha = 120;  // alpha线圈匝数
    YZK_p4ps5->N_beta = 120;   // beta线圈匝数
    YZK_p4ps5->g = 10;
    YZK_p4ps5->K = 1 / (YZK_p4ps5->N_alpha * YZK_p4ps5->N_alpha * YZK_p4ps5->mu_0 * YZK_p4ps5->S);
}


void incremental_PI_YZK(st_pid_regulator *r){
    r->Err = r->Ref - r->Fbk;
    r->Out = r->OutPrev + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki_CODE * r->Err;
    if(r->Out > r->OutLimit) r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit) r->Out = -r->OutLimit;
    r->ErrPrev = r->Err;
    r->OutPrev = r->Out;
    YZK_CTRL->Out = r->Out;
}

// 一阶低通滤波器 (离散化: Tustin/双线性近似)
double lowpass_update(LowPassFilter *f, double input) {
    double alpha = CL_TS / (YZK_LPF->TAU + CL_TS);   // 滤波系数
    f->prev_output = f->prev_output + alpha * (input - f->prev_output);
    return f->prev_output;
}

// Y_Pos 
void suspension_p4ps5_PD_Yaxis(REAL Y_Pos){
    /* 位置环 */    
    // 1. 误差
    YZK_CTRL->prev_error_Y = YZK_CTRL->Err_Y; // 保存上次误差
    YZK_CTRL->disFbk_Y = Y_Pos;
    YZK_CTRL->varTheta = (*CTRL).i->varTheta;
    YZK_CTRL->Err_Y = YZK_CTRL->CMD_Y - YZK_CTRL->disFbk_Y;

    // 2. 误差微分 (差分法)
    YZK_LPF->de_raw = (YZK_CTRL->Err_Y - YZK_CTRL->prev_error_Y) * CL_TS_INVERSE;

    // 3. 低通滤波(获得/dot{Err_Y})
    YZK_LPF->de = lowpass_update(YZK_LPF, YZK_LPF->de_raw);

    // 4. 控制律: 磁链参考 看那张纸上的公式，找不到找YZK
    YZK_CTRL->CMD_psi_beta = YZK_p4ps5->M_rotor * YZK_p4ps5->g
    - KP_Y * YZK_CTRL->Err_Y 
    - YZK_CTRL->KD_Y * YZK_LPF->de;

    if (YZK_CTRL->CMD_psi_beta < 0.0) YZK_CTRL->CMD_psi_beta = 0.0;   // 避免 sqrt 负数

    YZK_CTRL->CMD_psi_beta = sqrt(YZK_CTRL->CMD_psi_beta);

    // 5. 电流参考
    // K = 1 / N_alpha;
    YZK_CTRL->CMD_I_beta = YZK_p4ps5->K * YZK_CTRL->CMD_psi_beta * 1 / (YZK_p4ps5->ge - YZK_CTRL->disFbk_Y);

    // 6. PI
    incremental_PI(YZK_PID);
    YZK_CTRL->CMD_U_beta = YZK_CTRL->Out;

    // 更新状态
    // YZK_CTRL->prev_error = YZK_CTRL->Err_Y;

    // return psi_cmd;
}
// X_Pos 
void suspension_p4ps5_PD_Xaxis(REAL X_Pos){
    /* 位置环 */    
    // 1. 误差
    YZK_CTRL->prev_error_X = YZK_CTRL->Err_X; // 保存上次误差
    YZK_CTRL->disFbk_X = X_Pos;
    YZK_CTRL->varTheta = (*CTRL).i->varTheta;
    YZK_CTRL->Err_X = YZK_CTRL->CMD_X - YZK_CTRL->disFbk_X;

    // 2. 误差微分 (差分法)
    YZK_LPF->de_raw = (YZK_CTRL->Err_X - YZK_CTRL->prev_error_X) * CL_TS_INVERSE;

    // 3. 低通滤波(获得/dot{Err_X})
    YZK_LPF->de = lowpass_update(YZK_LPF, YZK_LPF->de_raw);

    // 4. 控制律: 磁链参考 看那张纸上的公式，找不到找YZK
    YZK_CTRL->CMD_psi_alpha = YZK_p4ps5->M_rotor * YZK_p4ps5->g
    - KP_X * YZK_CTRL->Err_X
    - YZK_CTRL->KD_X * YZK_LPF->de;

    if (YZK_CTRL->CMD_psi_alpha < 0.0) YZK_CTRL->CMD_psi_alpha = 0.0;   // 避免 sqrt 负数

    YZK_CTRL->CMD_psi_alpha = sqrt(YZK_CTRL->CMD_psi_alpha);

    // 5. 电流参考
    // K = 0;
    YZK_CTRL->CMD_I_alpha = YZK_p4ps5->K * YZK_CTRL->CMD_psi_alpha * 1 / (YZK_p4ps5->ge - YZK_CTRL->disFbk_X);

    /* 电流环 */
    incremental_PI_YZK(YZK_PID);
    YZK_CTRL->CMD_U_alpha = YZK_CTRL->Out;

    // 更新状态
    // YZK_CTRL->prev_error = YZK_CTRL->Err_Y;

    // return psi_cmd;
}

// IIR
void _IIR_lpf(biquad_t *f_out, float fs, float fc, float Q){
    REAL w0 = 2.0f * (REAL)M_PI * fc / fs;
    REAL cosw0 = cosf(w0);
    REAL sinw0 = sinf(w0);
    REAL alpha = sinw0 / (2.0f * Q);

    REAL b0 = (1.0f - cosw0) * 0.5f;
    REAL b1 = 1.0f - cosw0;
    REAL b2 = b0;
    REAL a0 = 1.0f + alpha;
    REAL a1 = -2.0f * cosw0;
    REAL a2 = 1.0f - alpha;

    // 归一化
    b0 /= a0; b1 /= a0; b2 /= a0;
    a1 /= a0; a2 /= a0;

    biquad_init(f_out, b0, b1, b2, a1, a2);
}

REAL biquad_process(biquad_t *f, float x) {
    // y = b0*x + s1
    float y = f->b0 * x + f->s1;
    // update states
    float s1_new = f->b1 * x - f->a1 * y + f->s2;
    float s2_new = f->b2 * x - f->a2 * y;
    f->s1 = s1_new;
    f->s2 = s2_new;
    return y;
}

void biquad_init(biquad_t *f, float b0, float b1, float b2, float a1, float a2) {
    f->b0 = b0; f->b1 = b1; f->b2 = b2;
    f->a1 = a1; f->a2 = a2;
    f->s1 = 0.0f; f->s2 = 0.0f;
}

void filters_init(void) {
    biquad_init(&g_filters->lp_ch0, LP0_b0, LP0_b1, LP0_b2, LP0_a1, LP0_a2);
    biquad_init(&g_filters->lp_ch1, LP1_b0, LP1_b1, LP1_b2, LP1_a1, LP1_a2);
}




