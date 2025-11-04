#include "ACMSim.h"


// YZK_2025_TIA_CTRL *YZK_CTRL;
// LowPassFilter *YZK_LPF;
// st_pid_regulator *YZK_PID;
// p4ps5_motor_suspension_parameters *YZK_p4ps5;
// filters_t *g_filters;

const REAL I_ampa;
const REAL I_ampb;
const REAL F_freq;
struct YZK_2025_TIA_CTRL YZK_CTRL;

// K = tan(pi * fc / fs)
// norm = 1 + K/Q + K^2
// b0 = K^2 / norm
// b1 = 2*b0
// b2 = b0
// a1 = 2*(K^2 - 1) / norm
// a2 = (1 - K/Q + K^2) / norm

    /* IIR */
static const REAL LP0_b0 = 0.003621681514928641;       //0.003621681514928641;             0.000087655375;
static const REAL LP0_b1 = 0.007243363029857282;       //0.007243363029857282;            0.000175310749;
static const REAL LP0_b2 = 0.003621681514928641;       //0.003621681514928641;             0.000087655375;
static const REAL LP0_a1 = -1.822694925196308;      //-1.822694925196308;                -1.973340329766;
static const REAL LP0_a2 = 0.8371816512560227;       //0.8371816512560227;    200Hz       0.973690951265;

static const REAL LP1_b0 =0.003621681514928641;                          //0.000087655375;              //0.013359200027856523f;                   //0.020083365564211256f;                          //0.04613180207
static const REAL LP1_b1 =0.007243363029857282;                          //0.000175310749;              //0.026718400055713045f;                   //0.04016673112842251f;                            //0.09226360415
static const REAL LP1_b2 =0.003621681514928641;                          //0.000087655375;              //0.013359200027856523f;                   //0.020083365564211256f;                          //0.04613180207
static const REAL LP1_a1 =-1.822694925196308;                          //-1.973340329766;             //-1.6474599810769766f;                    //-1.561018075800718f;                            //-1.30728502829
static const REAL LP1_a2 =0.8371816512560227;   //200Hz                       //0.973690951265;    //30Hz          //0.7008967811884026f;     //400Hz            //0.6413515380575631f;    //500Hz        //0.49181223659      //800Hz

static const REAL C3 = -1.29166723e-18;
static const REAL C2 =  1.36515629e-10;
static const REAL C1 = -4.80387525e-03;
static const REAL C0 =  5.62798031e+04;

static const REAL L_A = 1.76766429e-07;
static const REAL L_B = -5.89365901e+00;
// static const REAL KP_X = 0.2;
// static const REAL KP_Y = 0.5;

// static const REAL YZK_CTRL->KI_X = 0.0;
// static const REAL YZK_CTRL->KI_Y = 0.0;
// static const REAL YZK_CTRL->KD_X = 0.0;
// static const REAL YZK_CTRL->KD_Y = 0.0;
/* Initialising */
void init_YZK_ALL(){
    /* XY方向 */
    YZK_CTRL.CMD_X = 3.28999996;
    YZK_CTRL.CMD_Y = 3.9000001;
    YZK_CTRL.Err_X = 0.0;
    YZK_CTRL.Err_Y = 0.0;
    YZK_CTRL.KP_X = 200;
    YZK_CTRL.KP_Y = 200;
    YZK_CTRL.KI_X = 0.0;
    YZK_CTRL.KI_Y = 0.0;
    YZK_CTRL.KD_X = 5e-3;
    YZK_CTRL.KD_Y = 8e-3;
    /* 磁链 */
    YZK_CTRL.CMD_psi_alpha = 0.0;
    YZK_CTRL.CMD_psi_beta  = 0.0;
    YZK_CTRL.Err_psi_alpha = 0.0;
    YZK_CTRL.Err_psi_beta  = 0.0;
    YZK_CTRL.CMD_F_X       = 0.0;
    YZK_CTRL.CMD_F_Y       = 0.0;
    YZK_CTRL.CMD_F_alpha   = 0.0;
    YZK_CTRL.CMD_F_beta    = 0.0;
    YZK_CTRL.CMD_I_alpha   = 0.0;
    YZK_CTRL.CMD_I_beta    = 0.0;
    YZK_CTRL.CMD_U_alpha   = 0.0;
    YZK_CTRL.CMD_U_beta    = 0.0;
    YZK_CTRL.Err_I_alpha   = 0.0;
    YZK_CTRL.Err_I_beta    = 0.0;
    YZK_CTRL.OutPrev_alpha = 0.0;
    YZK_CTRL.OutPrev_beta = 0.0;
    YZK_CTRL.Out_alpha = 0.0;
    YZK_CTRL.Out_beta = 0.0;
    // YZK_CTRL.I_Term_prev_iD = 0.0;
    // YZK_CTRL.I_Term_prev_iQ = 0.0;
    // YZK_CTRL.K_INVERSE_iD = 0.0;
    // YZK_CTRL.K_INVERSE_iQ = 0.0;
    /* LPFs*/
    YZK_CTRL.LPFs.TAU = 0.5026548;

    /* FOC */
    YZK_CTRL.disFbk_X = 0.0;
    YZK_CTRL.disFbk_Y = 0.0;
    YZK_CTRL.encFbk = 0.0;
    YZK_CTRL.prev_error_X = 0.0;
    YZK_CTRL.prev_error_Y = 0.0;
    YZK_CTRL.prev_error_I_alpha = 0.0;
    YZK_CTRL.prev_error_I_beta = 0.0;

    YZK_CTRL.pids.Kp = 0.7793;
    YZK_CTRL.pids.Ki_CODE = 0.1794;
    YZK_CTRL.pids.Kd = 0.0;
    YZK_CTRL.pids.OutLimit = 6;

    /* 参数初始化 */
    YZK_CTRL.motor.npp = 5;
    YZK_CTRL.motor.npp_inv = 0.2;
    YZK_CTRL.motor.Js = 1;
    YZK_CTRL.motor.Js_inv = 1;
    YZK_CTRL.motor.M_rotor = 1.50;  // rotor mass
    YZK_CTRL.motor.ge = 3800;       // air gap length
    YZK_CTRL.motor.mu_0 = 4*M_PI*1e-7;     // 真空磁导率
    YZK_CTRL.motor.S = 1;        // alpha正对的面积
    YZK_CTRL.motor.N_alpha = 120;  // alpha线圈匝数
    YZK_CTRL.motor.N_beta = 120;   // beta线圈匝数
    YZK_CTRL.motor.g = 10;
    // YZK_CTRL.motor.K_X = 50000;
    // YZK_CTRL.motor.K_Y = 50000;
    YZK_CTRL.motor.K_X = 0.1;  // 1 A / 10 N
    YZK_CTRL.motor.K_Y = 0.1;  // 1 A / 10 N
}


// void incremental_PI_YZK(st_pid_regulator *r){
//     r->Err = r->Ref - r->Fbk;
//     r->Out = r->OutPrev + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki_CODE * r->Err;
//     if(r->Out > r->OutLimit) r->Out = r->OutLimit;
//     else if(r->Out < -r->OutLimit) r->Out = -r->OutLimit;
//     r->ErrPrev = r->Err;
//     r->OutPrev = r->Out;
//     YZK_CTRL.Out = r->Out;
// }

// 一阶低通滤波器 (离散化: Tustin/双线性近似)
double lowpass_update(LPFs *f, double input) {
    double alpha = CL_TS / (f->TAU + CL_TS);   // 滤波系数
    f->prev_output = f->prev_output + alpha * (input - f->prev_output);
    return f->prev_output;
}

// Y_Pos 
// void suspension_p4ps5_PD_Yaxis(REAL Y_Pos){
    
// }
// X_Pos 
void suspension_p4ps5_PD_doubleaxis(REAL X_Pos, REAL Y_Pos){
    /* 位置环 */    
    // 1. 误差
    // YZK_CTRL.prev_error_X = YZK_CTRL.Err_X; // 保存上次误差
    // YZK_CTRL.disFbk_X = X_Pos;
    YZK_CTRL.varTheta = (*CTRL).i->theta_d_elec;
    YZK_CTRL.Err_X = - YZK_CTRL.CMD_X + YZK_CTRL.disFbk_X;
    // YZK_CTRL.varTheta = (*CTRL).i->theta_d_elec;
    YZK_CTRL.Err_Y = - YZK_CTRL.CMD_Y + YZK_CTRL.disFbk_Y;
// 
    // 2. 误差微分 (差分法)
    YZK_CTRL.LPFs.de_raw_X = (YZK_CTRL.Err_X - YZK_CTRL.prev_error_X) * CL_TS_INVERSE;
    YZK_CTRL.LPFs.de_raw_Y = (YZK_CTRL.Err_Y - YZK_CTRL.prev_error_Y) * CL_TS_INVERSE;
// 
    // 3. 低通滤波(获得/dot{Err_X})
    // YZK_CTRL.LPFs.de_X = YZK_CTRL.LPFs.de_raw_X;
    YZK_CTRL.LPFs.de_X = lowpass_update(&YZK_CTRL.LPFs, YZK_CTRL.LPFs.de_raw_X);
    YZK_CTRL.LPFs.de_Y = lowpass_update(&YZK_CTRL.LPFs, YZK_CTRL.LPFs.de_raw_Y);
// 
    // 4. 控制律: 磁链参考 看那张纸上的公式，找不到找YZK
    // YZK_CTRL.CMD_psi_alpha = YZK_CTRL.KP_X * YZK_CTRL.Err_X + YZK_CTRL.KD_X * YZK_CTRL.LPFs.de_X;
// 
    // if (YZK_CTRL.CMD_psi_alpha < 0.0) YZK_CTRL.CMD_psi_alpha = - sqrt(-YZK_CTRL.CMD_psi_alpha);   // 避免 sqrt 负数
// 
    // if (YZK_CTRL.CMD_psi_alpha > 0.0) YZK_CTRL.CMD_psi_alpha = sqrt(YZK_CTRL.CMD_psi_alpha);    //注意传感器反装 如果不是这里要改
// 
    // 5. 电流参考
    // YZK_CTRL.CMD_F_X = I_ampa * cos(2 * F_freq * M_PI * CTRL->timebase);
    // YZK_CTRL.CMD_F_Y = I_ampa * sin(2 * F_freq * M_PI * CTRL->timebase);
    YZK_CTRL.CMD_F_X = YZK_CTRL.KP_X * YZK_CTRL.Err_X + YZK_CTRL.KD_X * YZK_CTRL.LPFs.de_X;
    YZK_CTRL.CMD_F_Y = YZK_CTRL.KP_Y * YZK_CTRL.Err_Y + YZK_CTRL.KD_Y * YZK_CTRL.LPFs.de_Y;

    // K = 0;
    YZK_CTRL.CMD_F_alpha = YZK_CTRL.CMD_F_X * cos(M_PI/6) - YZK_CTRL.CMD_F_Y * sin(M_PI/6);
    YZK_CTRL.CMD_F_beta  = YZK_CTRL.CMD_F_X * sin(M_PI/6) + YZK_CTRL.CMD_F_Y * cos(M_PI/6);
    YZK_CTRL.CMD_I_alpha = YZK_CTRL.motor.K_X * (YZK_CTRL.CMD_F_alpha * cos(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * sin(YZK_CTRL.varTheta));
    YZK_CTRL.CMD_I_beta  = YZK_CTRL.motor.K_Y * ( - YZK_CTRL.CMD_F_alpha * sin(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * cos(YZK_CTRL.varTheta));
    // YZK_CTRL.CMD_I_alpha = I_ampa * cos(YZK_CTRL.varTheta - M_PI/3) * cos(2 * F_freq * M_PI * CTRL->timebase);
    // YZK_CTRL.CMD_I_alpha = cos(YZK_CTRL.varTheta) * YZK_CTRL.motor.K_X * YZK_CTRL.CMD_psi_alpha * 1 / (YZK_CTRL.motor.ge - YZK_CTRL.disFbk_X);
    YZK_CTRL.Err_I_alpha = YZK_CTRL.CMD_I_alpha - CTRL->i->iAB[0];
    YZK_CTRL.Err_I_beta = YZK_CTRL.CMD_I_beta - CTRL->i->iAB[1];
// 
    /* 电流环 */
    YZK_CTRL.Out_alpha = YZK_CTRL.OutPrev_alpha + YZK_CTRL.pids.Kp * ( YZK_CTRL.Err_I_alpha - YZK_CTRL.prev_error_I_alpha ) + YZK_CTRL.pids.Ki_CODE * YZK_CTRL.Err_I_alpha;
    if(YZK_CTRL.Out_alpha > YZK_CTRL.pids.OutLimit) YZK_CTRL.Out_alpha = YZK_CTRL.pids.OutLimit;
    else if(YZK_CTRL.Out_alpha < -YZK_CTRL.pids.OutLimit) YZK_CTRL.Out_alpha = -YZK_CTRL.pids.OutLimit;
    YZK_CTRL.prev_error_I_alpha = YZK_CTRL.Err_I_alpha; 
    YZK_CTRL.OutPrev_alpha = YZK_CTRL.Out_alpha;
    YZK_CTRL.CMD_U_alpha = YZK_CTRL.Out_alpha;
    // incremental_PI_YZK(&YZK_CTRL.pids);
    // YZK_CTRL.CMD_U_alpha = YZK_CTRL.Out;
    (*CTRL).o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha;
    // 更新状态
    YZK_CTRL.prev_error_X = YZK_CTRL.Err_X;

    // 6. PI
    YZK_CTRL.Out_beta = YZK_CTRL.OutPrev_beta + YZK_CTRL.pids.Kp * ( YZK_CTRL.Err_I_beta - YZK_CTRL.prev_error_I_beta ) + YZK_CTRL.pids.Ki_CODE * YZK_CTRL.Err_I_beta;
    if(YZK_CTRL.Out_beta > YZK_CTRL.pids.OutLimit) YZK_CTRL.Out_beta = YZK_CTRL.pids.OutLimit;
    else if(YZK_CTRL.Out_beta < -YZK_CTRL.pids.OutLimit) YZK_CTRL.Out_beta = -YZK_CTRL.pids.OutLimit;
    YZK_CTRL.prev_error_I_beta = YZK_CTRL.Err_I_beta; 
    YZK_CTRL.OutPrev_beta = YZK_CTRL.Out_beta;
    YZK_CTRL.CMD_U_beta = YZK_CTRL.Out_beta;
    // incremental_PI_YZK(&YZK_CTRL.pids);
    // YZK_CTRL.CMD_U_beta = YZK_CTRL.Out;
    (*CTRL).o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta;
    // 7.更新状态
    YZK_CTRL.prev_error_Y = YZK_CTRL.Err_Y;
    // return psi_cmd;
}

// // IIR
void _IIR_lpf(biquad_t *f_out, REAL fs, REAL fc, REAL Q){
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

// REAL biq_X = 0.0;
// REAL biq_Y = 0.0;
REAL biquad_process(biquad_t *f, REAL x0) {
    // f->y0s = f->b0*x0 + f->b1*f->x1s + f->b2*f->x2s - f->a1*f->y1s - f->a2*f->y2s;
    // shift
    // f->x2s = f->x1s;
    // f->y2s = f->y1s;
    // f->y1s = f->y0s;
    // return f->y0s;
    REAL y = f->b0 * x0 + f->s1x;
    // f->x1s = x0;
    REAL s1_new = f->b1 * x0 - f->a1 * y + f->s2x;
    REAL s2_new = f->b2 * x0 - f->a2 * y;
    // write back
    f->s1x = s1_new;
    f->s2x = s2_new;
    return y;
}

void biquad_init(biquad_t *f, REAL b0, REAL b1, REAL b2, REAL a1, REAL a2) {
    f->b0 = b0; f->b1 = b1; f->b2 = b2;
    f->a1 = a1; f->a2 = a2;
    f->s1x = 0.0f; f->s2x = 0.0f;
}

void filters_init(void) {
    biquad_init(&YZK_CTRL.biq, LP0_b0, LP0_b1, LP0_b2, LP0_a1, LP0_a2);
    biquad_init(&YZK_CTRL.biq, LP1_b0, LP1_b1, LP1_b2, LP1_a1, LP1_a2);
}

/* 传感器三次拟合 暂时弃用*/
// REAL sensor_to_distance(REAL dis_input, double valid_min, double valid_max) {
    // REAL x = ((C3 * dis_input + C2) * dis_input + C1) * dis_input + C0;
    // sanity checks
    // if (!isfinite(x)) {
        // fallback
        // x = L_A * dis_input + L_B;
    // }
// 
    // If cubic gives an out-of-range result, clamp to valid range.
    // Optionally, you can fallback to linear if outside by large margin.
    // if (x < valid_min || x > valid_max) {
        // double xl = L_A * dis_input + L_B;
        // choose the one closer to the valid interval center
        // double mid = 0.5 * (valid_min + valid_max);
        // double d_cubic = (x < valid_min) ? (valid_min - x) : (x - valid_max);
        // double d_lin = fabs(xl - mid);
        // If linear seems reasonable, use it; otherwise clamp cubic
        // if (isfinite(xl) && d_lin < d_cubic*10.0) {
            // x = xl;
        // } else {
            // if (x < valid_min) x = valid_min;
            // if (x > valid_max) x = valid_max;
        // }
    // }
    // return x;
// }
