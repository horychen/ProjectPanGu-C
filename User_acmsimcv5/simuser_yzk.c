#include "ACMSim.h"
#include "All_Definition.h"

#define DC_BUS_VOLTAGE_INVERSE_1 (1.732 / Axis_1.vdc)
#define DC_BUS_VOLTAGE_INVERSE_2 (1.732 / Axis_2.vdc)
// YZK_2025_TIA_CTRL *YZK_CTRL;
// LowPassFilter *YZK_LPF;
// st_pid_regulator *YZK_PID;
// p4ps5_motor_suspension_parameters *YZK_p4ps5;
// filters_t *g_filters;

const REAL I_ampa;
const REAL I_ampb;
const REAL I_ampc;
const REAL I_ampd;
const REAL I_DQ[2];
const REAL F_freq_1;
const REAL F_freq_2;
const REAL virtual_angle;
REAL pseudo_Encoder;
REAL BOOL_CurrentProfileGenerator = FALSE;
CurrentProfileGenerator my_gen;
struct YZK_2025_TIA_CTRL YZK_CTRL;

extern float test_sus = 0.0;
extern BOOL BOOL_DIRECT_FIELD_TEST = TRUE;
REAL ONLY_CURRENT_LOOP_TEST = TRUE;
REAL BOOL_eccentricity = FALSE;
REAL sensor_1;
REAL sensor_2;
REAL sensor_3;
REAL sensor_4;
// REAL r_0;
// REAL r_45;
REAL S_0;               // 形参
REAL S_45;              // 形参
REAL A_r = 20;            // 半长轴
REAL B_r = 12;            // 半短轴
REAL r_0;               // 极径 0
REAL r_45;              // 极径 45
REAL SQRT_ARCTG_0_num;
REAL SQRT_ARCTG_45_num;
REAL SQRT_ARCTG_0_den;
REAL SQRT_ARCTG_45_den;
REAL varTHETA_0;
REAL varTHETA_45;
REAL V0_V45_N;
REAL V0_V45_P;
extern REAL place_sensor[8];
// K = tan(pi * fc / fs)1
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

static const REAL LP1_b0 = 0.003621681514928641;                          //0.000087655375;              //0.013359200027856523f;                   //0.020083365564211256f;                          //0.04613180207
static const REAL LP1_b1 = 0.007243363029857282;                          //0.000175310749;              //0.026718400055713045f;                   //0.04016673112842251f;                            //0.09226360415
static const REAL LP1_b2 = 0.003621681514928641;                          //0.000087655375;              //0.013359200027856523f;                   //0.020083365564211256f;                          //0.04613180207
static const REAL LP1_a1 = -1.822694925196308;                          //-1.973340329766;             //-1.6474599810769766f;                    //-1.561018075800718f;                            //-1.30728502829
static const REAL LP1_a2 = 0.8371816512560227;   //200Hz                       //0.973690951265;    //30Hz          //0.7008967811884026f;     //400Hz            //0.6413515380575631f;    //500Hz        //0.49181223659      //800Hz

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
    YZK_CTRL.CMD_X = 1.445;
    YZK_CTRL.CMD_Y = 0.3;
    YZK_CTRL.Err_X_1 = 0.0;
    YZK_CTRL.Err_Y_1 = 0.0;
    YZK_CTRL.Err_X_2 = 0.0;
    YZK_CTRL.Err_Y_2 = 0.0;
    YZK_CTRL.KP_X = 20;
    YZK_CTRL.KP_Y = 20;
    YZK_CTRL.KI_X = 0.0;
    YZK_CTRL.KI_Y = 0.0;
    YZK_CTRL.KD_X = 0.1;
    YZK_CTRL.KD_Y = 8e-3;
    /* 磁链 */
    YZK_CTRL.CMD_psi_alpha = 0.0;
    YZK_CTRL.CMD_psi_beta  = 0.0;
    YZK_CTRL.Err_psi_alpha = 0.0;
    YZK_CTRL.Err_psi_beta  = 0.0;
    YZK_CTRL.CMD_F_X       = 0.0;
    YZK_CTRL.CMD_F_Y       = 0.0;
    YZK_CTRL.CMD_F_X_prime = 0.0;
    YZK_CTRL.CMD_F_Y_prime = 0.0;
    YZK_CTRL.CMD_F_X_Kp    = 0.0;
    YZK_CTRL.CMD_F_X_Kd    = 0.0;
    YZK_CTRL.CMD_F_Y_Kp    = 0.0;
    YZK_CTRL.CMD_F_Y_Kd    = 0.0;
    YZK_CTRL.CMD_F_alpha   = 0.0;
    YZK_CTRL.CMD_F_beta    = 0.0;
    YZK_CTRL.KDLimit       = 40.0;
    YZK_CTRL.CMD_I_alpha_1   = 0.0;
    YZK_CTRL.CMD_I_beta_1    = 0.0;
    YZK_CTRL.CMD_U_alpha_1   = 0.0;
    YZK_CTRL.CMD_U_beta_1    = 0.0;
    YZK_CTRL.Err_I_alpha_1   = 0.0;
    YZK_CTRL.Err_I_beta_1    = 0.0;
    YZK_CTRL.OutPrev_alpha_1 = 0.0;
    YZK_CTRL.OutPrev_beta_1 = 0.0;
    YZK_CTRL.Out_alpha_1 = 0.0;
    YZK_CTRL.Out_beta_1 = 0.0;
    YZK_CTRL.Out_alpha_KI_1 = 0.0;
    YZK_CTRL.Out_beta_KI_1 = 0.0;
    
    YZK_CTRL.CMD_I_alpha_2   = 0.0;
    YZK_CTRL.CMD_I_beta_2    = 0.0;
    YZK_CTRL.CMD_U_alpha_2   = 0.0;
    YZK_CTRL.CMD_U_beta_2    = 0.0;
    YZK_CTRL.Err_I_alpha_2   = 0.0;
    YZK_CTRL.Err_I_beta_2    = 0.0;
    YZK_CTRL.OutPrev_alpha_2 = 0.0;
    YZK_CTRL.OutPrev_beta_2 = 0.0;
    YZK_CTRL.Out_alpha_2 = 0.0;
    YZK_CTRL.Out_beta_2 = 0.0;
    YZK_CTRL.Out_alpha_KI_2 = 0.0;
    YZK_CTRL.Out_beta_KI_2 = 0.0;
    
    // YZK_CTRL.I_Term_prev_iD = 0.0;
    // YZK_CTRL.I_Term_prev_iQ = 0.0;
    // YZK_CTRL.K_INVERSE_iD = 0.0;
    // YZK_CTRL.K_INVERSE_iQ = 0.0;
    /* LPFs*/
    YZK_CTRL.LPFs_x.TAU_x = 3.978873577e-4 * 2;
    YZK_CTRL.LPFs_y.TAU_y = 3.978873577e-4 * 2;

    /* FOC */
    YZK_CTRL.disFbk_X = 0.0;
    YZK_CTRL.disFbk_Y = 0.0;
    YZK_CTRL.encFbk   = 0.0;
    YZK_CTRL.prev_error_X_1 = 0.0;
    YZK_CTRL.prev_error_Y_1 = 0.0;
    YZK_CTRL.prev_error_X_2 = 0.0;
    YZK_CTRL.prev_error_Y_2 = 0.0;
    YZK_CTRL.prev_error_I_alpha_1 = 0.0;
    YZK_CTRL.prev_error_I_beta_1 = 0.0;
    YZK_CTRL.prev_error_I_alpha_2 = 0.0;
    YZK_CTRL.prev_error_I_beta_2 = 0.0;

    YZK_CTRL.pids.Kp_alpha_1 = 5;
    YZK_CTRL.pids.Kp_beta_1 = 5;
    YZK_CTRL.pids.Ki_CODE_alpha_1 = 3500e-4;
    YZK_CTRL.pids.Ki_CODE_beta_1 = 3500e-4;
    // YZK_CTRL.pids.Kd = 0.0;
    YZK_CTRL.pids.OutLimit_1 = 14;
    YZK_CTRL.pids.OutLimit_alphaKI_1 = 5;
    YZK_CTRL.pids.OutLimit_betaKI_1 = 5;

    YZK_CTRL.pids.Kp_alpha_2 = 5;
    YZK_CTRL.pids.Kp_beta_2 = 5;
    YZK_CTRL.pids.Ki_CODE_alpha_2 = 3500e-4;
    YZK_CTRL.pids.Ki_CODE_beta_2 = 3500e-4;
    // YZK_CTRL.pids.Kd = 0.0;
    YZK_CTRL.pids.OutLimit_2 = 14;
    YZK_CTRL.pids.OutLimit_alphaKI_2 = 5;
    YZK_CTRL.pids.OutLimit_betaKI_2 = 5;

    /* 参数初始化 */
    YZK_CTRL.motor.npp = 4;
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
    YZK_CTRL.dc_bus_utilization_ratio_1 = 0;
    YZK_CTRL.dc_bus_utilization_ratio_2 = 0;

    YZK_CTRL.varTheta = 0;
    YZK_CTRL.varThetaOffset = 0;

    // TEST //
    YZK_CTRL.id_iq_amps[0] = 2.0f;
    YZK_CTRL.id_iq_amps[1] = 4.0f;
    YZK_CTRL.id_iq_amps[2] = 6.0f;

    YZK_CTRL.ix_amps[0] = 0.0f;
    YZK_CTRL.ix_amps[1] = 0.0f;
    YZK_CTRL.ix_amps[2] = 0.0f;

    YZK_CTRL.iy_amps[0] = 0.0f;
    YZK_CTRL.iy_amps[1] = 0.0f;
    YZK_CTRL.iy_amps[2] = 0.0f;
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
REAL lowpass_update_x(LPFs *f_x, REAL input_x) {
    REAL alpha_x = CL_TS / (f_x->TAU_x + CL_TS);   // 滤波系数
    f_x->prev_output_x = f_x->prev_output_x + alpha_x * (input_x - f_x->prev_output_x);
    return f_x->prev_output_x;
}

REAL lowpass_update_y(LPFs *f_y, REAL input_y) {
    REAL alpha_y = CL_TS / (f_y->TAU_y + CL_TS);   // 滤波系数
    f_y->prev_output_y = f_y->prev_output_y + alpha_y * (input_y - f_y->prev_output_y);
    return f_y->prev_output_y;
}

// Y_Pos 
// void suspension_p4ps5_PD_Yaxis(REAL Y_Pos){

// }
// X_Pos 
void suspension_p4ps5_PD_doubleaxis(REAL X_Pos, REAL Y_Pos){
    
    /* pseudo encoder */
    if (fabsf(debug_2.Overwrite_Current_Frequency) > 0){
        debug_2.Overwrite_theta_d += CL_TS * debug_2.Overwrite_Current_Frequency * 2 * M_PI;
        if (debug_2.Overwrite_theta_d > M_PI)  debug_2.Overwrite_theta_d -= 2 * M_PI;
        if (debug_2.Overwrite_theta_d < - M_PI) debug_2.Overwrite_theta_d += 2 * M_PI;
    }
    else{
        debug_2.Overwrite_theta_d = 0.0;
    }

    // 用椭圆检测相减的距离 记得是45° 给进来给到椭圆角度
        place_sensor[0] = place_sensor[0] * 30 / 4.964 - 15;
        place_sensor[1] = place_sensor[1] * 30 / 4.964 - 15;
        // 做一个数据截断 x轴
        if (place_sensor[0] >= 0)
        place_sensor[0] = floor(place_sensor[0] * 300.0) / 300;
        else
        place_sensor[0] = ceil(place_sensor[0] * 300.0)  / 300;
        // 做一个数据截断 y轴
        if (place_sensor[1] >= 0)
        place_sensor[1] = floor(place_sensor[1] * 300.0) / 300;
        else
        place_sensor[1] = ceil(place_sensor[1] * 300.0)  / 300;

        place_sensor[5] = place_sensor[5] * 30 / 5.01 - 15;
        place_sensor[7] = place_sensor[7] * 30 / 5.01 - 15;
        // 做一个数据截断 x轴
        if (place_sensor[5] >= 0)
        place_sensor[5] = floor(place_sensor[5] * 300.0) / 300;
        else
        place_sensor[5] = ceil(place_sensor[5] * 300.0)  / 300;
        // 做一个数据截断 y轴
        if (place_sensor[7] >= 0)
        place_sensor[7] = floor(place_sensor[7] * 300.0) / 300;
        else
        place_sensor[7] = ceil(place_sensor[7] * 300.0)  / 300;
    if(BOOL_eccentricity)
        {
        sensor_1 = place_sensor[0];
        sensor_2 = place_sensor[1];
        sensor_3 = place_sensor[5];
        sensor_4 = place_sensor[7];
        r_0 = (40 + sensor_2 + sensor_3) * 0.5;
        r_45 = (40 + sensor_1 + sensor_4) * 0.5;
        S_0 = A_r * B_r / r_0;
        S_45 = A_r * B_r / r_45;

        SQRT_ARCTG_0_num = sqrt(S_0 * S_0 - B_r * B_r);
        SQRT_ARCTG_45_num = sqrt(S_45 * S_45 - B_r * B_r);
        SQRT_ARCTG_0_den = sqrt(A_r * A_r - S_0 * S_0);
        SQRT_ARCTG_45_den = sqrt(A_r * A_r - S_45 * S_45);
        varTHETA_0 = atan2(SQRT_ARCTG_0_num, SQRT_ARCTG_0_den);
        varTHETA_45 = atan2(SQRT_ARCTG_45_num, SQRT_ARCTG_45_den);
        // 3 cases
        if(varTHETA_0 + varTHETA_45 == M_PI*0.25)
        {
            YZK_CTRL.varTheta = varTHETA_0 * YZK_CTRL.motor.npp;
        }
        else if(varTHETA_0 - varTHETA_45 == M_PI*0.25)
        {
            YZK_CTRL.varTheta = varTHETA_0 * YZK_CTRL.motor.npp;
        }
        else if(varTHETA_45 - varTHETA_0 == M_PI*0.25)
        {
            YZK_CTRL.varTheta = - varTHETA_0 * YZK_CTRL.motor.npp;
        }else
        {
            YZK_CTRL.varTheta = varTHETA_0 * YZK_CTRL.motor.npp;
        }
        
        YZK_CTRL.varTheta -= YZK_CTRL.varThetaOffset;

        if (YZK_CTRL.varTheta > M_PI) YZK_CTRL.varTheta -= 2.0*M_PI;
        if (YZK_CTRL.varTheta < -M_PI) YZK_CTRL.varTheta += 2.0*M_PI;

        V0_V45_N = varTHETA_45 - varTHETA_0;
        V0_V45_P = varTHETA_45 + varTHETA_0;
        }
    if(! ONLY_CURRENT_LOOP_TEST){
        /* 位置环 */    
        // 1. 误差
        // YZK_CTRL.prev_error_X = YZK_CTRL.Err_X; // 保存上次误差
        // YZK_CTRL.disFbk_X = X_Pos;
        // YZK_CTRL.varTheta = (*CTRL).i->theta_d_elec;
        YZK_CTRL.Err_X_1 = - YZK_CTRL.CMD_X + YZK_CTRL.disFbk_X;
        YZK_CTRL.Err_X_2 = - YZK_CTRL.CMD_X + YZK_CTRL.disFbk_X;
        // YZK_CTRL.varTheta = (*CTRL).i->theta_d_elec;
        YZK_CTRL.Err_Y_1 = - YZK_CTRL.CMD_Y + YZK_CTRL.disFbk_Y;
        YZK_CTRL.Err_Y_2 = - YZK_CTRL.CMD_Y + YZK_CTRL.disFbk_Y;

        YZK_CTRL.Err_X_1 = _lpf(YZK_CTRL.Err_X_1, YZK_CTRL.prev_error_X_1, 2513.27); // 400Hz
        YZK_CTRL.Err_Y_1 = _lpf(YZK_CTRL.Err_Y_1, YZK_CTRL.prev_error_Y_1, 2513.27); // 400Hz

        YZK_CTRL.Err_X_2 = _lpf(YZK_CTRL.Err_X_2, YZK_CTRL.prev_error_X_2, 2513.27); // 400Hz
        YZK_CTRL.Err_Y_2 = _lpf(YZK_CTRL.Err_Y_2, YZK_CTRL.prev_error_Y_2, 2513.27); // 400Hz

        // 2. 误差微分 (差分法)
        YZK_CTRL.LPFs_x.de_raw_X = (YZK_CTRL.Err_X_1 - YZK_CTRL.prev_error_X_1) * CL_TS_INVERSE;
        YZK_CTRL.LPFs_y.de_raw_Y = (YZK_CTRL.Err_Y_1 - YZK_CTRL.prev_error_Y_1) * CL_TS_INVERSE;

        // 3. 低通滤波(获得/dot{Err_X})
        // YZK_CTRL.LPFs.de_X = YZK_CTRL.LPFs.de_raw_X;
        // lpf test //
        // YZK_CTRL.LPFs.de_raw_X = cos(2 * F_freq_1 * M_PI * CTRL->timebase);
        // YZK_CTRL.LPFs.de_raw_Y = cos(2 * F_freq_2 * M_PI * CTRL->timebase);
        // lpf test //
        // 低通滤波器：测量值，上一步的滤波器输出，时间常数的倒数
        YZK_CTRL.LPFs_x.de_X = lowpass_update_x(&YZK_CTRL.LPFs_x, YZK_CTRL.LPFs_x.de_raw_X);
        YZK_CTRL.LPFs_y.de_Y = lowpass_update_y(&YZK_CTRL.LPFs_y, YZK_CTRL.LPFs_y.de_raw_Y);
        // YZK_CTRL.LPFs.de_X = 1; // lowpass_update(&YZK_CTRL.LPFs, YZK_CTRL.LPFs.de_raw_X);
        // YZK_CTRL.LPFs.de_Y = 1; // lowpass_update(&YZK_CTRL.LPFs, YZK_CTRL.LPFs.de_raw_Y);

    
        // 4. 控制律: 磁链参考 看那张纸上的公式，找不到找YZK
        // YZK_CTRL.CMD_psi_alpha = YZK_CTRL.KP_X * YZK_CTRL.Err_X + YZK_CTRL.KD_X * YZK_CTRL.LPFs.de_X;
    
        // if (YZK_CTRL.CMD_psi_alpha < 0.0) YZK_CTRL.CMD_psi_alpha = - sqrt(-YZK_CTRL.CMD_psi_alpha);   // 避免 sqrt 负数
    
        // if (YZK_CTRL.CMD_psi_alpha > 0.0) YZK_CTRL.CMD_psi_alpha = sqrt(YZK_CTRL.CMD_psi_alpha);    //注意传感器反装 如果不是这里要改
    
        // 5. 电流参考

        // YZK_CTRL.CMD_F_X = I_ampa * cos(2 * F_freq_1 * M_PI * CTRL->timebase);
        // YZK_CTRL.CMD_F_Y = I_ampa * sin(2 * F_freq_2 * M_PI * CTRL->timebase);
        YZK_CTRL.CMD_F_X_Kp = YZK_CTRL.KP_X * YZK_CTRL.Err_X_1;
        YZK_CTRL.CMD_F_X_Kd = YZK_CTRL.KD_X * YZK_CTRL.LPFs_x.de_X; 
        if(YZK_CTRL.CMD_F_X_Kd > YZK_CTRL.KDLimit) YZK_CTRL.CMD_F_X_Kd = YZK_CTRL.KDLimit;
        else if(YZK_CTRL.CMD_F_X_Kd < -YZK_CTRL.KDLimit) YZK_CTRL.CMD_F_X_Kd = -YZK_CTRL.KDLimit;

        // YZK_CTRL.CMD_F_X = YZK_CTRL.CMD_F_X_Kp + YZK_CTRL.CMD_F_X_Kd;

        YZK_CTRL.CMD_F_Y_Kp = YZK_CTRL.KP_Y * YZK_CTRL.Err_Y_1;
        YZK_CTRL.CMD_F_Y_Kd = YZK_CTRL.KD_Y * YZK_CTRL.LPFs_y.de_Y;
        if(YZK_CTRL.CMD_F_Y_Kd > YZK_CTRL.KDLimit) YZK_CTRL.CMD_F_Y_Kd = YZK_CTRL.KDLimit;
        else if(YZK_CTRL.CMD_F_Y_Kd < -YZK_CTRL.KDLimit) YZK_CTRL.CMD_F_Y_Kd = -YZK_CTRL.KDLimit;

        // YZK_CTRL.CMD_F_Y = YZK_CTRL.CMD_F_Y_Kp + YZK_CTRL.CMD_F_Y_Kd;
        // K = 0;
        // YZK_CTRL.CMD_F_X_prime = YZK_CTRL.CMD_F_X * cos(5 * M_PI / 12) - YZK_CTRL.CMD_F_Y * sin(5 * M_PI / 12);
        // YZK_CTRL.CMD_F_Y_prime = YZK_CTRL.CMD_F_X * sin(5 * M_PI / 12) + YZK_CTRL.CMD_F_Y * cos(5 * M_PI / 12);
        // 定子安装时的1号槽倾角 此时为0度
        YZK_CTRL.CMD_F_alpha = YZK_CTRL.CMD_F_X;
        // 3 A -> 15 N mass of the rotor
        YZK_CTRL.CMD_F_beta  = YZK_CTRL.CMD_F_Y;
        // YZK_CTRL.CMD_F_beta  = YZK_CTRL.CMD_F_Y + 30;

        // YZK_CTRL.CMD_I_alpha = YZK_CTRL.motor.K_X * YZK_CTRL.CMD_F_alpha;
        // YZK_CTRL.CMD_I_beta  = YZK_CTRL.motor.K_Y * YZK_CTRL.CMD_F_beta;

        YZK_CTRL.CMD_I_alpha_1 = YZK_CTRL.motor.K_X * (YZK_CTRL.CMD_F_alpha * cos(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * sin(YZK_CTRL.varTheta));
        YZK_CTRL.CMD_I_beta_1  = YZK_CTRL.motor.K_Y * ( - YZK_CTRL.CMD_F_alpha * sin(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * cos(YZK_CTRL.varTheta));
        YZK_CTRL.CMD_I_alpha_2 = YZK_CTRL.motor.K_X * (YZK_CTRL.CMD_F_alpha * cos(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * sin(YZK_CTRL.varTheta));
        YZK_CTRL.CMD_I_beta_2  = YZK_CTRL.motor.K_Y * ( - YZK_CTRL.CMD_F_alpha * sin(YZK_CTRL.varTheta) + YZK_CTRL.CMD_F_beta * cos(YZK_CTRL.varTheta));
        // YZK_CTRL.CMD_I_alpha = I_ampa * cos(YZK_CTRL.varTheta - M_PI/3) * cos(2 * F_freq * M_PI * CTRL->timebase);
        // YZK_CTRL.CMD_I_alpha = cos(YZK_CTRL.varTheta) * YZK_CTRL.motor.K_X * YZK_CTRL.CMD_psi_alpha * 1 / (YZK_CTRL.motor.ge - YZK_CTRL.disFbk_X);
    
    }
    // /* Torque Part */
    // (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    // (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    // /* D-Axis Current Loop */
    // PID_iD->Fbk = 0.5 * CTRL_2.i->iDQ[0] + CTRL_1.i->iDQ[0];
    // PID_iD->Ref = CTRL_2.i->cmd_iDQ[0];
    // PID_iD->calc(PID_iD);
    // /* Q-Axis Current Loop */
    // PID_iQ->Fbk = 0.5 * CTRL_2.i->iDQ[1] + CTRL_1.i->iDQ[1];
    // PID_iQ->Ref = CTRL_2.i->cmd_iDQ[1];
    // PID_iQ->calc(PID_iQ);

    // REAL decoupled_d_axis_voltage;
    // REAL decoupled_q_axis_voltage;
    // decoupled_d_axis_voltage = PID_iD->Out;
    // decoupled_q_axis_voltage = PID_iQ->Out;
    // /* 对补偿后的dq轴电压进行限幅度 */
    // if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    // else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    // if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    // else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;
    // (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    // (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    // (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    // (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

    /* Torque Part */ 
    CTRL_1.i->cmd_iDQ[0] = - 0.5 * debug_2.set_id_command;
    CTRL_1.i->cmd_iDQ[1] = - 0.5 * debug_2.set_iq_command;
    CTRL_2.i->cmd_iDQ[0] = 0.5 * debug_2.set_id_command;
    CTRL_2.i->cmd_iDQ[1] = 0.5 * debug_2.set_iq_command;

    // (*CTRL).i->theta_d_elec
    if(pseudo_Encoder)
    {
        (*CTRL).s->cosT = cos(debug_2.Overwrite_theta_d);
        (*CTRL).s->sinT = sin(debug_2.Overwrite_theta_d);
    }
    else if(YZK_CTRL.varTheta < M_PI || YZK_CTRL.varTheta > -M_PI)
    {
        (*CTRL).s->cosT = cos(YZK_CTRL.varTheta);
        (*CTRL).s->sinT = sin(YZK_CTRL.varTheta);
    }
    
    if(BOOL_CurrentProfileGenerator){
        CurrentProfileGenerator_Update(&my_gen,
                                    &YZK_CTRL.CMD_I_alpha_1,
                                    &YZK_CTRL.CMD_I_beta_1,
                                    &CTRL_1.i->cmd_iDQ[0],
                                    &CTRL_1.i->cmd_iDQ[1]);
        
        CurrentProfileGenerator_Update(&my_gen,
                                    &YZK_CTRL.CMD_I_alpha_2,
                                    &YZK_CTRL.CMD_I_beta_2,
                                    &CTRL_2.i->cmd_iDQ[0],
                                    &CTRL_2.i->cmd_iDQ[1]);
        
        // CTRL_1.i->cmd_iDQ[0] = - 1.0 * CTRL_1.i->cmd_iDQ[0];
        // CTRL_1.i->cmd_iDQ[1] = - 1.0 * CTRL_1.i->cmd_iDQ[1];
        CTRL_1.i->cmd_iDQ[0] = - 1.0 * CTRL_1.i->cmd_iDQ[0];
        CTRL_1.i->cmd_iDQ[1] = - 1.0 * CTRL_1.i->cmd_iDQ[1];
        CTRL_2.i->cmd_iDQ[0] = 1.0 * CTRL_2.i->cmd_iDQ[0];
        CTRL_2.i->cmd_iDQ[1] = 1.0 * CTRL_2.i->cmd_iDQ[1];
    }

    CTRL_1.i->cmd_iDQ[0] = - I_DQ[0];
    CTRL_1.i->cmd_iDQ[1] = - I_DQ[1];
    CTRL_2.i->cmd_iDQ[0] = I_DQ[0] * cos( - M_PI * 0.33333333) + I_DQ[1] * sin( - M_PI * 0.33333333);
    CTRL_2.i->cmd_iDQ[1] = I_DQ[0] * (-sin( - M_PI * 0.33333333)) + I_DQ[1] * cos( - M_PI * 0.33333333);

    CTRL_1.o->cmd_iAB[0] = MT2A(CTRL_1.i->cmd_iDQ[0], CTRL_1.i->cmd_iDQ[1], CTRL_1.s->cosT, CTRL_1.s->sinT);
    CTRL_1.o->cmd_iAB[1] = MT2B(CTRL_1.i->cmd_iDQ[0], CTRL_1.i->cmd_iDQ[1], CTRL_1.s->cosT, CTRL_1.s->sinT);
    CTRL_2.o->cmd_iAB[0] = MT2A(CTRL_2.i->cmd_iDQ[0], CTRL_2.i->cmd_iDQ[1], CTRL_2.s->cosT, CTRL_2.s->sinT);
    CTRL_2.o->cmd_iAB[1] = MT2B(CTRL_2.i->cmd_iDQ[0], CTRL_2.i->cmd_iDQ[1], CTRL_2.s->cosT, CTRL_2.s->sinT);

    if(axisCnt == 0)
    {   
        YZK_CTRL.CMD_I_alpha_1 = I_ampc;
        YZK_CTRL.CMD_I_beta_1 = I_ampd;

        // YZK_CTRL.CMD_I_alpha_1 = I_ampa * cos(2 * F_freq_1 * M_PI * CTRL->timebase) * cos(virtual_angle * M_PI);
        // YZK_CTRL.CMD_I_beta_1 = I_ampb * sin(2 * F_freq_2 * M_PI * CTRL->timebase) * sin(virtual_angle * M_PI);
        
        // YZK_CTRL.CMD_I_alpha_1 = I_ampa * cos(2 * F_freq_1 * M_PI * CTRL->timebase);
        // YZK_CTRL.CMD_I_beta_1 = I_ampb * sin(2 * F_freq_2 * M_PI * CTRL->timebase);

        YZK_CTRL.Err_I_alpha_1 = CTRL_1.o->cmd_iAB[0] + YZK_CTRL.CMD_I_alpha_1 - CTRL_1.i->iAB[0];
        YZK_CTRL.Err_I_beta_1  = CTRL_1.o->cmd_iAB[1] + YZK_CTRL.CMD_I_beta_1 - CTRL_1.i->iAB[1];
        
        _lpf(YZK_CTRL.Err_I_alpha_1, YZK_CTRL.prev_error_I_alpha_1, 12513.27);
        _lpf(YZK_CTRL.Err_I_beta_1, YZK_CTRL.prev_error_I_beta_1, 12513.27);

        /* 电流环 */
        YZK_CTRL.Out_alpha_KI_1 = YZK_CTRL.pids.Ki_CODE_alpha_1 * YZK_CTRL.Err_I_alpha_1;

        if(YZK_CTRL.Out_alpha_KI_1 > YZK_CTRL.pids.OutLimit_alphaKI_1)       YZK_CTRL.Out_alpha_KI_1 = YZK_CTRL.pids.OutLimit_alphaKI_1;
        else if(YZK_CTRL.Out_alpha_KI_1 < - YZK_CTRL.pids.OutLimit_alphaKI_1) YZK_CTRL.Out_alpha_KI_1 = -YZK_CTRL.pids.OutLimit_alphaKI_1;

        YZK_CTRL.Out_alpha_1 = YZK_CTRL.OutPrev_alpha_1 + YZK_CTRL.pids.Kp_alpha_1 * ( YZK_CTRL.Err_I_alpha_1 - YZK_CTRL.prev_error_I_alpha_1 ) \
        + YZK_CTRL.Out_alpha_KI_1;

        if(YZK_CTRL.Out_alpha_1 > YZK_CTRL.pids.OutLimit_1)       YZK_CTRL.Out_alpha_1 = YZK_CTRL.pids.OutLimit_1;
        else if(YZK_CTRL.Out_alpha_1 < - YZK_CTRL.pids.OutLimit_1) YZK_CTRL.Out_alpha_1 = -YZK_CTRL.pids.OutLimit_1;

        YZK_CTRL.prev_error_I_alpha_1 = YZK_CTRL.Err_I_alpha_1;
        YZK_CTRL.OutPrev_alpha_1 = YZK_CTRL.Out_alpha_1;
        YZK_CTRL.CMD_U_alpha_1 = YZK_CTRL.Out_alpha_1;
        // incremental_PI_YZK(&YZK_CTRL.pids);
        // YZK_CTRL.CMD_U_alpha = YZK_CTRL.Out;

        CTRL_1.o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha_1;
        // CTRL_2.o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha;
        // 更新状态
        YZK_CTRL.prev_error_X_1 = YZK_CTRL.Err_X_1;

        // 6. PI
        YZK_CTRL.Out_beta_KI_1 = YZK_CTRL.pids.Ki_CODE_beta_1 * YZK_CTRL.Err_I_beta_1;

        if(YZK_CTRL.Out_beta_KI_1 > YZK_CTRL.pids.OutLimit_betaKI_1)       YZK_CTRL.Out_beta_KI_1 = YZK_CTRL.pids.OutLimit_betaKI_1;
        else if(YZK_CTRL.Out_beta_KI_1 < - YZK_CTRL.pids.OutLimit_betaKI_1) YZK_CTRL.Out_beta_KI_1 = -YZK_CTRL.pids.OutLimit_betaKI_1;

        YZK_CTRL.Out_beta_1 = YZK_CTRL.OutPrev_beta_1 + YZK_CTRL.pids.Kp_beta_1 * ( YZK_CTRL.Err_I_beta_1 - YZK_CTRL.prev_error_I_beta_1 ) \
        + YZK_CTRL.Out_beta_KI_1;

        if(YZK_CTRL.Out_beta_1 > YZK_CTRL.pids.OutLimit_1)       YZK_CTRL.Out_beta_1 = YZK_CTRL.pids.OutLimit_1;
        else if(YZK_CTRL.Out_beta_1 < - YZK_CTRL.pids.OutLimit_1) YZK_CTRL.Out_beta_1 = -YZK_CTRL.pids.OutLimit_1;

        YZK_CTRL.prev_error_I_beta_1 = YZK_CTRL.Err_I_beta_1;
        YZK_CTRL.OutPrev_beta_1 = YZK_CTRL.Out_beta_1;
        YZK_CTRL.CMD_U_beta_1 = YZK_CTRL.Out_beta_1;
        // incremental_PI_YZK(&YZK_CTRL.pids);
        // YZK_CTRL.CMD_U_beta = YZK_CTRL.Out;

        // CTRL_1.o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta;
        CTRL_1.o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta_1;
        // 7.更新状态
        YZK_CTRL.prev_error_Y_1 = YZK_CTRL.Err_Y_1;
        // return psi_cmd;
        YZK_CTRL.dc_bus_utilization_ratio_1 = DC_BUS_VOLTAGE_INVERSE_1 * sqrtf( YZK_CTRL.CMD_U_alpha_1
                                                                            * YZK_CTRL.CMD_U_alpha_1
                                                                            + YZK_CTRL.CMD_U_beta_1
                                                                            * YZK_CTRL.CMD_U_beta_1 );
    }
    if (axisCnt == 1)
    {
        YZK_CTRL.CMD_I_alpha_2 = I_ampc * cos(YZK_CTRL.varTheta - M_PI * 0.33333333);
        YZK_CTRL.CMD_I_beta_2 = I_ampd * cos(YZK_CTRL.varTheta - M_PI * 0.33333333);

        // YZK_CTRL.CMD_I_alpha_2 = I_ampa * cos(2 * F_freq_1 * M_PI * CTRL->timebase) * cos(virtual_angle * M_PI);
        // YZK_CTRL.CMD_I_beta_2 = I_ampb * sin(2 * F_freq_2 * M_PI * CTRL->timebase) * sin(virtual_angle * M_PI);

        // YZK_CTRL.CMD_I_alpha_2 = I_ampa * cos(2 * F_freq_1 * M_PI * CTRL->timebase);
        // YZK_CTRL.CMD_I_beta_2 = I_ampb * sin(2 * F_freq_2 * M_PI * CTRL->timebase);

        YZK_CTRL.Err_I_alpha_2 = CTRL_2.o->cmd_iAB[0] + YZK_CTRL.CMD_I_alpha_2 - CTRL_2.i->iAB[0];
        YZK_CTRL.Err_I_beta_2  = CTRL_2.o->cmd_iAB[1] + YZK_CTRL.CMD_I_beta_2 - CTRL_2.i->iAB[1];

        _lpf(YZK_CTRL.Err_I_alpha_2, YZK_CTRL.prev_error_I_alpha_2, 12513.27);
        _lpf(YZK_CTRL.Err_I_beta_2, YZK_CTRL.prev_error_I_beta_2, 12513.27);

        /* 电流环 */
        YZK_CTRL.Out_alpha_KI_2 = YZK_CTRL.pids.Ki_CODE_alpha_2 * YZK_CTRL.Err_I_alpha_2;

        if(YZK_CTRL.Out_alpha_KI_2 > YZK_CTRL.pids.OutLimit_alphaKI_2)       YZK_CTRL.Out_alpha_KI_2 = YZK_CTRL.pids.OutLimit_alphaKI_2;
        else if(YZK_CTRL.Out_alpha_KI_2 < - YZK_CTRL.pids.OutLimit_alphaKI_2) YZK_CTRL.Out_alpha_KI_2 = -YZK_CTRL.pids.OutLimit_alphaKI_2;

        YZK_CTRL.Out_alpha_2 = YZK_CTRL.OutPrev_alpha_2 + YZK_CTRL.pids.Kp_alpha_2 * ( YZK_CTRL.Err_I_alpha_2 - YZK_CTRL.prev_error_I_alpha_2 ) \
        + YZK_CTRL.Out_alpha_KI_2;

        if(YZK_CTRL.Out_alpha_2 > YZK_CTRL.pids.OutLimit_2)       YZK_CTRL.Out_alpha_2 = YZK_CTRL.pids.OutLimit_2;
        else if(YZK_CTRL.Out_alpha_2 < - YZK_CTRL.pids.OutLimit_2) YZK_CTRL.Out_alpha_2 = -YZK_CTRL.pids.OutLimit_2;

        YZK_CTRL.prev_error_I_alpha_2 = YZK_CTRL.Err_I_alpha_2; 
        YZK_CTRL.OutPrev_alpha_2 = YZK_CTRL.Out_alpha_2;
        YZK_CTRL.CMD_U_alpha_2 = YZK_CTRL.Out_alpha_2;
        // incremental_PI_YZK(&YZK_CTRL.pids);
        // YZK_CTRL.CMD_U_alpha = YZK_CTRL.Out;

        CTRL_2.o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha_2;
        // CTRL_2.o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha;
        // 更新状态
        YZK_CTRL.prev_error_X_2 = YZK_CTRL.Err_X_2;

        // 6. PI
        YZK_CTRL.Out_beta_KI_2 = YZK_CTRL.pids.Ki_CODE_beta_2 * YZK_CTRL.Err_I_beta_2;

        if(YZK_CTRL.Out_beta_KI_2 > YZK_CTRL.pids.OutLimit_betaKI_2)       YZK_CTRL.Out_beta_KI_2 = YZK_CTRL.pids.OutLimit_betaKI_2;
        else if(YZK_CTRL.Out_beta_KI_2 < - YZK_CTRL.pids.OutLimit_betaKI_2) YZK_CTRL.Out_beta_KI_2 = -YZK_CTRL.pids.OutLimit_betaKI_2;

        YZK_CTRL.Out_beta_2 = YZK_CTRL.OutPrev_beta_2 + YZK_CTRL.pids.Kp_beta_2 * ( YZK_CTRL.Err_I_beta_2 - YZK_CTRL.prev_error_I_beta_2 ) \
        + YZK_CTRL.Out_beta_KI_2;

        if(YZK_CTRL.Out_beta_2 > YZK_CTRL.pids.OutLimit_2)       YZK_CTRL.Out_beta_2 = YZK_CTRL.pids.OutLimit_2;
        else if(YZK_CTRL.Out_beta_2 < - YZK_CTRL.pids.OutLimit_2) YZK_CTRL.Out_beta_2 = -YZK_CTRL.pids.OutLimit_2;

        YZK_CTRL.prev_error_I_beta_2 = YZK_CTRL.Err_I_beta_2;
        YZK_CTRL.OutPrev_beta_2 = YZK_CTRL.Out_beta_2;
        YZK_CTRL.CMD_U_beta_2 = YZK_CTRL.Out_beta_2;
        // incremental_PI_YZK(&YZK_CTRL.pids);
        // YZK_CTRL.CMD_U_beta = YZK_CTRL.Out;

        // CTRL_1.o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta;
        CTRL_2.o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta_2;
        // 7.更新状态
        YZK_CTRL.prev_error_Y_2 = YZK_CTRL.Err_Y_2;
        // return psi_cmd;
        YZK_CTRL.dc_bus_utilization_ratio_2 = DC_BUS_VOLTAGE_INVERSE_2 * sqrtf( YZK_CTRL.CMD_U_alpha_2
                                                                            * YZK_CTRL.CMD_U_alpha_2
                                                                            + YZK_CTRL.CMD_U_beta_2
                                                                            * YZK_CTRL.CMD_U_beta_2 );
    }
    if (CurrentProfileGenerator_IsCompleted(&my_gen)) {
             // 测试完成，可以停止或重置
             CurrentProfileGenerator_Reset(&my_gen);  // 重新开始
    }
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

// static inline float safe_atan2f(float y, float x, float last_angle) 
// {
//     if (isnanf(x) || isnanf(y)) return last_angle;
//     if (x == 0.0f && y == 0.0f) return last_angle; // 没有信息，返回上一值（或者 0）
//     return atan2f(y, x); // 返回 (-pi, pi]
// }

// static inline float wrap_to_pi(float a) {

//     float r = fmodf(a + M_PI, 2*M_PI);
//     if (r < 0.0f) r += 2*M_PI;
//     return r - M_PI;
// }

// /* 计算两个角度之间的最短差值，返回范围 (-pi, pi] */
// static inline float ang_diff(float to, float from) {
//     float d = to - from;
//     // 折叠到 (-pi, pi]
//     if (d <= -M_PI) d += 2*M_PI;
//     else if (d > M_PI) d -= 2*M_PI;
//     return d;
// }


/* ========================================================================
 *  电流轮廓生成器实现 (Current Profile Generator Implementation)
 * ========================================================================
 *  用途: 生成多层级的Id, Iq电流指令，用于电机测试
 *  
 *  工作原理:
 *  - 三层嵌套循环: Ix幅值 -> Id/Iq幅值 -> 角度
 *  - 在dq平面上画圆，每个圆由多个角度段组成
 *  - 每个角度段持续固定时间
 *  - 适合实时嵌入式系统，无需大数组存储
 * ======================================================================== */

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
                                   int auto_restart)
{
    int i;

    // 参数检查（确保不越界）
    if (num_idiq > MAX_IDIQ_AMPLITUDES) num_idiq = MAX_IDIQ_AMPLITUDES;
    if (num_ix   > MAX_IX_AMPLITUDES)   num_ix   = MAX_IX_AMPLITUDES;
    if (num_iy   > MAX_IX_AMPLITUDES)   num_iy   = MAX_IX_AMPLITUDES; // 可改为单独 MAX_IY_AMPLITUDES
    if (num_angles > MAX_ANGLE_SEGMENTS) num_angles = MAX_ANGLE_SEGMENTS;

    // 复制配置参数
    for (i = 0; i < num_idiq; i++) {
        gen->id_iq_amplitudes[i] = id_iq_amps[i];
    }
    for (i = 0; i < num_ix; i++) {
        gen->ix_amplitudes[i] = ix_amps[i];
    }
    for (i = 0; i < num_iy; i++) {
        gen->iy_amplitudes[i] = iy_amps[i];
    }

    gen->num_idiq_amplitudes = num_idiq;
    gen->num_ix_amplitudes   = num_ix;
    gen->num_iy_amplitudes   = num_iy;
    gen->num_angle_segments  = num_angles;
    gen->circles_per_amplitude = (circles_per_amplitude > 0) ? circles_per_amplitude : 1;  // 至少1圈
    gen->cycles_per_segment  = cycles_per_segment;
    gen->sample_time = sample_time;
    
    // 如果指定了周期数(>0)，则根据周期数计算持续时间；否则使用平台时间参数
    if (cycles_per_segment > 0) {
        gen->duration_per_segment = cycles_per_segment * sample_time;
    } else {
        gen->duration_per_segment = platform_duration;
    }
    
    gen->pause_duration = pause_duration;
    gen->auto_restart = auto_restart;

    // 初始化状态
    gen->ix_index = 0;
    gen->iy_index = 0;
    gen->amp_index = 0;
    gen->circle_index = 0;
    gen->angle_index = 0;
    gen->segment_timer = 0.0;
    gen->pause_timer = 0.0;
    gen->is_pausing = 0;

    // 计算初始电流值 (第一个角度点: theta = 0)
    if (num_idiq > 0 && num_ix > 0 && num_iy > 0) {
        gen->current_id = gen->id_iq_amplitudes[0];  // cos(0) = 1
        gen->current_iq = 0.0;                        // sin(0) = 0
        gen->current_ix = gen->ix_amplitudes[0];
        gen->current_iy = gen->iy_amplitudes[0];
    } else {
        gen->current_id = 0.0;
        gen->current_iq = 0.0;
        gen->current_ix = 0.0;
        gen->current_iy = 0.0;
    }

    // 设置标志
    gen->is_initialized = 1;
    gen->is_running = 1;
    gen->is_completed = 0;
}

void CurrentProfileGenerator_Reset(CurrentProfileGenerator *gen)
{
    gen->ix_index = 0;
    gen->iy_index = 0;
    gen->amp_index = 0;
    gen->circle_index = 0;
    gen->angle_index = 0;
    gen->segment_timer = 0.0;
    gen->pause_timer = 0.0;
    gen->is_pausing = 0;

    // 重置到初始电流值
    if (gen->num_idiq_amplitudes > 0 && gen->num_ix_amplitudes > 0 && gen->num_iy_amplitudes > 0) {
        gen->current_id = gen->id_iq_amplitudes[0];
        gen->current_iq = 0.0;
        gen->current_ix = gen->ix_amplitudes[0];
        gen->current_iy = gen->iy_amplitudes[0];
    } else {
        gen->current_id = 0.0;
        gen->current_iq = 0.0;
        gen->current_ix = 0.0;
        gen->current_iy = 0.0;
    }

    gen->is_running = 1;
    gen->is_completed = 0;
}

void CurrentProfileGenerator_Update(CurrentProfileGenerator *gen, 
                                     REAL *id_out, 
                                     REAL *iq_out,
                                     REAL *ix_out,   // 新增输出参数，用于返回当前 ix
                                     REAL *iy_out)   // 新增输出参数，用于返回当前 iy
{
    REAL theta;
    REAL amplitude;

    // 如果未初始化，返回零电流
    if (!gen->is_initialized) {
        if (id_out) *id_out = 0.0;
        if (iq_out) *iq_out = 0.0;
        if (ix_out) *ix_out = 0.0;
        if (iy_out) *iy_out = 0.0;
        return;
    }

    // 处理暂停状态
    if (gen->is_pausing) {
        if (id_out) *id_out = 0.0;  // 暂停期间输出零电流
        if (iq_out) *iq_out = 0.0;
        if (ix_out) *ix_out = 0.0;
        if (iy_out) *iy_out = 0.0;

        // 更新暂停计时器
        gen->pause_timer += gen->sample_time;

        // 检查暂停时间是否结束
        if (gen->pause_timer >= gen->pause_duration) {
            gen->pause_timer = 0.0;
            gen->is_pausing = 0;

            // 根据auto_restart决定是否重新开始
            if (gen->auto_restart) {
                CurrentProfileGenerator_Reset(gen);  // 自动重启
            } else {
                gen->is_completed = 1;  // 标记为完成
                gen->is_running = 0;
            }
        }
        return;
    }

    // 如果已完成（非暂停状态），返回零电流
    if (gen->is_completed) {
        if (id_out) *id_out = 0.0;
        if (iq_out) *iq_out = 0.0;
        if (ix_out) *ix_out = 0.0;
        if (iy_out) *iy_out = 0.0;
        return;
    }

    // 如果手动暂停，返回当前值
    if (!gen->is_running) {
        if (id_out) *id_out = gen->current_id;
        if (iq_out) *iq_out = gen->current_iq;
        if (ix_out) *ix_out = gen->current_ix;
        if (iy_out) *iy_out = gen->current_iy;
        return;
    }

    // 输出当前电流值
    if (id_out) *id_out = gen->current_id;
    if (iq_out) *iq_out = gen->current_iq;
    if (ix_out) *ix_out = gen->current_ix;
    if (iy_out) *iy_out = gen->current_iy;

    // 更新时间
    gen->segment_timer += gen->sample_time;

    // 检查是否需要切换到下一个段
    if (gen->segment_timer >= gen->duration_per_segment) {
        gen->segment_timer = 0.0;
        gen->angle_index++;

        // 检查是否完成一圈
        if (gen->angle_index >= gen->num_angle_segments) {
            gen->angle_index = 0;
            gen->circle_index++;  // 完成一圈，圆周计数器加1

            // 检查是否完成当前幅值的所有圆周
            if (gen->circle_index >= gen->circles_per_amplitude) {
                gen->circle_index = 0;  // 重置圆周计数器
                gen->amp_index++;       // 切换到下一个幅值

                // 检查是否完成所有Id/Iq幅值
                if (gen->amp_index >= gen->num_idiq_amplitudes) {
                    gen->amp_index = 0;
                    gen->iy_index++;   // 先推进 iy（内层于 ix）

                    // 检查是否完成所有Iy幅值
                    if (gen->iy_index >= gen->num_iy_amplitudes) {
                        gen->iy_index = 0;
                        gen->ix_index++; // 然后推进 ix（最外层）

                        // 检查是否完成所有Ix幅值
                        if (gen->ix_index >= gen->num_ix_amplitudes) {
                            // 如果设置了暂停时间，进入暂停状态
                            if (gen->pause_duration > 0.0) {
                                gen->is_pausing = 1;
                                gen->pause_timer = 0.0;
                                gen->current_id = 0.0;
                                gen->current_iq = 0.0;
                                gen->current_ix = 0.0;
                                gen->current_iy = 0.0;
                            } else {
                                // 没有暂停时间，直接完成
                                gen->is_completed = 1;
                                gen->is_running = 0;
                                gen->current_id = 0.0;
                                gen->current_iq = 0.0;
                                gen->current_ix = 0.0;
                                gen->current_iy = 0.0;
                            }
                            return;
                        }
                    }
                }
            }
        }

        // 计算新的电流值
        // theta = 2*PI * angle_index / num_angle_segments
        theta = 6.28318530718 * (REAL)gen->angle_index / (REAL)gen->num_angle_segments;
        amplitude = gen->id_iq_amplitudes[gen->amp_index];

        // 在dq平面上的圆周投影
        gen->current_id = amplitude * cos(theta);
        gen->current_iq = amplitude * sin(theta);
        gen->current_ix = gen->ix_amplitudes[gen->ix_index];
        gen->current_iy = gen->iy_amplitudes[gen->iy_index];
    }
}

/**
 * @brief 检查生成器是否完成所有序列
 * 
 * @param gen 生成器结构指针
 * @return 1=已完成, 0=未完成
 */
int CurrentProfileGenerator_IsCompleted(CurrentProfileGenerator *gen)
{
    return gen->is_completed;
}

/* ======================================================================== 
 *  使用示例:
 * ========================================================================
 *  
 *  // 1. 定义生成器变量
 *  CurrentProfileGenerator my_gen;
 *  
 *  // 2. 定义测试幅值
 *  REAL id_iq_amps[] = {1.0, 2.0, 3.0};      // 三个圆的半径
 *  REAL ix_amps[] = {0.0, 0.0, 0.0};              // 三个Ix层级
 *  
 *  // 3. 初始化生成器 (方式一：使用平台时间)
 *  CurrentProfileGenerator_Init(&my_gen,
 *                                id_iq_amps, 3,     // 3个Id/Iq幅值
 *                                ix_amps, 3,        // 3个Ix幅值
 *                                8,                 // 8个角度段 (每45度)
 *                                3,                 // 每个幅值重复3圈 (增加测量可信度)
 *                                1.0,               // 每个平台停留1秒
 *                                0,                 // 不使用周期数（设为0）
 *                                0.0001,            // 10kHz采样
 *                                0.0,               // 无暂停
 *                                0);                // 不自动重启
 *  
 *  // 或方式二：使用周期数 (更精确控制)
 *  CurrentProfileGenerator_Init(&my_gen,
 *                                id_iq_amps, 3,     // 3个Id/Iq幅值
 *                                ix_amps, 3,        // 3个Ix幅值
 *                                8,                 // 8个角度段
 *                                3,                 // 每个幅值重复3圈
 *                                0.0,               // 平台时间（当cycles_per_segment>0时被忽略）
 *                                10000,             // 每段运行10000个周期
 *                                0.0001,            // 10kHz采样 (10000*0.0001=1秒)
 *                                0.0,               // 无暂停
 *                                0);                // 不自动重启
 *  
 *  // 4. 在控制循环中调用
 *  void ControlLoop_ISR(void) {
 *      REAL id_ref, iq_ref;
 *      
 *      // 更新并获取电流指令
 *      CurrentProfileGenerator_Update(&my_gen, &id_ref, &iq_ref);
 *      
 *      // 使用id_ref和iq_ref进行电流控制
 *      // ...
 *      
 *      // 检查是否完成
 *      if (CurrentProfileGenerator_IsCompleted(&my_gen)) {
 *          // 测试完成，可以停止或重置
 *          CurrentProfileGenerator_Reset(&my_gen);  // 重新开始
 *      }
 *  }
 * 
 * ======================================================================== */
