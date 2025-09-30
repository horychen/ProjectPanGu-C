#include "ACMSim.h"


YZK_2025_TIA_CTRL *YZK_CTRL;
LowPassFilter *YZK_LPF;
st_pid_regulator *YZK_PID;
p4ps5_motor_suspension_parameters *YZK_p4ps5;
REAL K; // 一个神秘的导出系数



/* Initialising */
void _init_YZK_ALL(){
    /* XY方向 */
    YZK_CTRL->CMD_X = 0.0;
    YZK_CTRL->CMD_Y = 0.0;
    YZK_CTRL->Err_X = 0.0;
    YZK_CTRL->Err_Y = 0.0;
    YZK_CTRL->KP_X = 0.0;
    YZK_CTRL->KP_Y = 0.0;
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

// X_Pos 
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
    - YZK_CTRL->KP_Y * YZK_CTRL->Err_Y 
    - YZK_CTRL->KD_Y * YZK_LPF->de;

    if (YZK_CTRL->CMD_psi_beta < 0.0) YZK_CTRL->CMD_psi_beta = 0.0;   // 避免 sqrt 负数

    YZK_CTRL->CMD_psi_beta = sqrt(YZK_CTRL->CMD_psi_beta);

    // 5. 电流参考
    K = 0;
    YZK_CTRL->CMD_I_beta = K * YZK_CTRL->CMD_psi_beta * 1 / (YZK_p4ps5->ge - YZK_CTRL->disFbk_Y);

    // 6. PI
    incremental_PI(YZK_PID);
    YZK_CTRL->CMD_U_beta = YZK_CTRL->Out;

    // 更新状态
    // YZK_CTRL->prev_error = YZK_CTRL->Err_Y;

    // return psi_cmd;
}

void suspension_p4ps5_PD_Xaxis(REAL Y_Pos){
    /* 位置环 */    
    // 1. 误差
    YZK_CTRL->prev_error_X = YZK_CTRL->Err_X; // 保存上次误差
    YZK_CTRL->disFbk_X = Y_Pos;
    YZK_CTRL->varTheta = (*CTRL).i->varTheta;
    YZK_CTRL->Err_X = YZK_CTRL->CMD_X - YZK_CTRL->disFbk_X;

    // 2. 误差微分 (差分法)
    YZK_LPF->de_raw = (YZK_CTRL->Err_X - YZK_CTRL->prev_error_X) * CL_TS_INVERSE;

    // 3. 低通滤波(获得/dot{Err_X})
    YZK_LPF->de = lowpass_update(YZK_LPF, YZK_LPF->de_raw);

    // 4. 控制律: 磁链参考 看那张纸上的公式，找不到找YZK
    YZK_CTRL->CMD_psi_alpha = YZK_p4ps5->M_rotor * YZK_p4ps5->g
    - YZK_CTRL->KP_X * YZK_CTRL->Err_X 
    - YZK_CTRL->KD_X * YZK_LPF->de;

    if (YZK_CTRL->CMD_psi_alpha < 0.0) YZK_CTRL->CMD_psi_alpha = 0.0;   // 避免 sqrt 负数

    YZK_CTRL->CMD_psi_alpha = sqrt(YZK_CTRL->CMD_psi_alpha);

    // 5. 电流参考
    K = 0;
    YZK_CTRL->CMD_I_alpha = K * YZK_CTRL->CMD_psi_alpha * 1 / (YZK_p4ps5->ge - YZK_CTRL->disFbk_X);

    /* 电流环 */
    incremental_PI_YZK(YZK_PID);
    YZK_CTRL->CMD_U_alpha = YZK_CTRL->Out;

    // 更新状态
    // YZK_CTRL->prev_error = YZK_CTRL->Err_Y;

    // return psi_cmd;
}
