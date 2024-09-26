#ifndef IM_CONTROLLER_H
#define IM_CONTROLLER_H
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

// 这个结构体声明的是基本的IFOC中所没有的变量的集合体。
struct Marino2005{
    REAL kz;     // zd, zq
    REAL k_omega; // e_omega
    REAL kappa;  // e_omega
    REAL gamma_inv; // TL
    REAL delta_inv; // alpha
    REAL lambda_inv; // omega

    REAL xTL_Max;
    REAL xAlpha_Max;
    REAL xAlpha_min;

    REAL xRho;
    REAL xTL;
    REAL xAlpha;
    REAL xOmg;

    REAL deriv_xTL;
    REAL deriv_xAlpha;
    REAL deriv_xOmg;

    REAL psi_Dmu;
    REAL psi_Qmu;

    REAL zD;
    REAL zQ;
    REAL e_iDs;
    REAL e_iQs;
    REAL e_psi_Dmu;
    REAL e_psi_Qmu;

    REAL deriv_iD_cmd;
    REAL deriv_iQ_cmd;

    REAL Gamma_D;
    REAL Gamma_Q;

    REAL torque_cmd;
    REAL torque__fb;
};
extern struct Marino2005 marino;

struct PMSMife{
    REAL kz;     // zd, zq
    REAL ki;
    REAL k_omega; // e_omega
    REAL kappa;  // e_omega
    REAL gamma_inv; // TL
    REAL delta_inv; // alpha
    REAL lambda_inv; // omega
    
    REAL xTL_Max;

    REAL xRho;
    REAL xTL;
    REAL xOmg;
    REAL TL;
    REAL dot_TL;
    REAL deriv_xTL;
    REAL deriv_xAlpha;
    REAL deriv_xOmg;

    REAL psi_D2;
    REAL psi_Q2;

    REAL zD;
    REAL zQ;
    REAL e_iDs;
    REAL e_iQs;
    REAL e_psi_D2;
    REAL e_psi_Q2;

    REAL deriv_iD_cmd;
    REAL deriv_iQ_cmd;

    REAL Gamma_D;
    REAL Gamma_Q;

    REAL torque_cmd;
    REAL torque__fb;

    REAL cosT;
    REAL sinT;
};
extern struct PMSMife PMife;
// 控制器
void controller_IFOC();
void controller_marino2005();
void controller_marino2005_with_commands();
void init_im_controller();
void controller_PMSMife();
void controller_PMSMife_with_commands();
void init_PMSMife_controller();
// 初始化
// void experiment_init();
// void CTRL_init();
// void allocate_CTRL(struct ControllerForExperiment *CTRL);


// void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
// void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);


#endif
#endif
#endif
