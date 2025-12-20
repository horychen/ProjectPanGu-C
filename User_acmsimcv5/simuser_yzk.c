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
const REAL F_freq_1;
const REAL F_freq_2;
struct YZK_2025_TIA_CTRL YZK_CTRL;

extern float test_sus = 0.0;
extern BOOL BOOL_DIRECT_FIELD_TEST = TRUE;
REAL ONLY_CURRENT_LOOP_TEST = TRUE;

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
    YZK_CTRL.CMD_X = - 5.0;
    YZK_CTRL.CMD_Y = 10.10;
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

    YZK_CTRL.pids.Kp_alpha_1 = 15;
    YZK_CTRL.pids.Kp_beta_1 = 15;
    YZK_CTRL.pids.Ki_CODE_alpha_1 = 3500e-4;
    YZK_CTRL.pids.Ki_CODE_beta_1 = 3500e-4;
    // YZK_CTRL.pids.Kd = 0.0;
    YZK_CTRL.pids.OutLimit_1 = 28;
    YZK_CTRL.pids.OutLimit_alphaKI_1 = 10;
    YZK_CTRL.pids.OutLimit_betaKI_1 = 10;

    YZK_CTRL.pids.Kp_alpha_2 = 15;
    YZK_CTRL.pids.Kp_beta_2 = 15;
    YZK_CTRL.pids.Ki_CODE_alpha_2 = 3500e-4;
    YZK_CTRL.pids.Ki_CODE_beta_2 = 3500e-4;
    // YZK_CTRL.pids.Kd = 0.0;
    YZK_CTRL.pids.OutLimit_2 = 28;
    YZK_CTRL.pids.OutLimit_alphaKI_2 = 10;
    YZK_CTRL.pids.OutLimit_betaKI_2 = 10;

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
    YZK_CTRL.dc_bus_utilization_ratio_1 = 0;
    YZK_CTRL.dc_bus_utilization_ratio_2 = 0;
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
    
    if(! ONLY_CURRENT_LOOP_TEST){
        /* 位置环 */    
        // 1. 误差
        // YZK_CTRL.prev_error_X = YZK_CTRL.Err_X; // 保存上次误差
        // YZK_CTRL.disFbk_X = X_Pos;
        YZK_CTRL.varTheta = (*CTRL).i->theta_d_elec;
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

        YZK_CTRL.CMD_I_alpha_1 = YZK_CTRL.motor.K_X * (YZK_CTRL.CMD_F_alpha * cos(CTRL->i->theta_d_elec) + YZK_CTRL.CMD_F_beta * sin(CTRL->i->theta_d_elec));
        YZK_CTRL.CMD_I_beta_1  = YZK_CTRL.motor.K_Y * ( - YZK_CTRL.CMD_F_alpha * sin(CTRL->i->theta_d_elec) + YZK_CTRL.CMD_F_beta * cos(CTRL->i->theta_d_elec));
        YZK_CTRL.CMD_I_alpha_2 = YZK_CTRL.motor.K_X * (YZK_CTRL.CMD_F_alpha * cos(CTRL->i->theta_d_elec) + YZK_CTRL.CMD_F_beta * sin(CTRL->i->theta_d_elec));
        YZK_CTRL.CMD_I_beta_2  = YZK_CTRL.motor.K_Y * ( - YZK_CTRL.CMD_F_alpha * sin(CTRL->i->theta_d_elec) + YZK_CTRL.CMD_F_beta * cos(CTRL->i->theta_d_elec));
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
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    CTRL_1.o->cmd_iAB[0] = MT2A(CTRL_1.i->cmd_iDQ[0], CTRL_1.i->cmd_iDQ[1], CTRL_1.s->cosT_compensated_1p5omegaTs, CTRL_1.s->sinT_compensated_1p5omegaTs);
    CTRL_1.o->cmd_iAB[1] = MT2B(CTRL_1.i->cmd_iDQ[0], CTRL_1.i->cmd_iDQ[1], CTRL_1.s->cosT_compensated_1p5omegaTs, CTRL_1.s->sinT_compensated_1p5omegaTs);
    CTRL_2.o->cmd_iAB[0] = MT2A(CTRL_2.i->cmd_iDQ[0], CTRL_2.i->cmd_iDQ[1], CTRL_2.s->cosT_compensated_1p5omegaTs, CTRL_2.s->sinT_compensated_1p5omegaTs);
    CTRL_2.o->cmd_iAB[1] = MT2B(CTRL_2.i->cmd_iDQ[0], CTRL_2.i->cmd_iDQ[1], CTRL_2.s->cosT_compensated_1p5omegaTs, CTRL_2.s->sinT_compensated_1p5omegaTs);

    // overwrite_sweeping_f_1.quency_1.;
    if(axisCnt == 0)
    {
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
        CTRL_2.o->cmd_uAB_to_inverter[1] = YZK_CTRL.CMD_U_beta_1;
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

        CTRL_1.o->cmd_uAB_to_inverter[0] = YZK_CTRL.CMD_U_alpha_2;
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
