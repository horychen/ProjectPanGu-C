#ifndef ADD_PMSM_OBSERVER_H
#define ADD_PMSM_OBSERVER_H
#if MACHINE_TYPE == 2

/* One Big Struct for all PMSM observers */
struct ObserverForExperiment{
    /* Common */
    struct RK4_DATA{
        REAL us[2];
        REAL is[2];
        REAL us_curr[2];
        REAL is_curr[2];
        REAL us_prev[2];
        REAL is_prev[2];

        REAL is_lpf[2];
        REAL is_hpf[2];
        REAL is_bpf[2];

        REAL current_lpf_register[2];
        REAL current_hpf_register[2];
        REAL current_bpf_register1[2];
        REAL current_bpf_register2[2];

        // REAL omg_elec; // omg_elec = npp * omg_mech
        // REAL theta_d;
    } rk4;

/* Qiao.Xia 2013 SMO for Emf for SPMSM */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Qiao_Xia

    struct Declare_QiaoXia2013{
        #define NS_QIAO_XIA_2013 5
        REAL x[NS_QIAO_XIA_2013];
        REAL xIab[2];
        REAL xEmf[2];
        REAL xOmg;
        REAL theta_d;
        REAL xEmf_raw[2]; // SM terms
        REAL output_error[2];
        REAL sigmoid_coeff;
        REAL smo_gain;
        REAL adapt_gain;
        REAL mras_gain;
    } qiaoxia;
#define qiaoxia OBSV.qiaoxia
#endif

/* Chi.Xu 2009 SMO for Emf for SPMSM */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Chi_Xu

    struct Declare_ChiXu2009{
        #define NS_CHI_XU_2009 6
        REAL x[NS_CHI_XU_2009];
        REAL xIab[2];
        REAL xZeq[2];
        REAL xOmg;
        REAL xTheta_d; // PLL
        REAL theta_d;
        REAL xEmf_raw[2]; // SM terms
        REAL output_error[2];

        REAL sigmoid_coeff;
        REAL smo_gain;
        REAL ell4xZeq; // VSS
        REAL omega_lpf_4_xZeq;
        REAL omega_lpf_4_xZeq_const_part;
        REAL PLL_KP;
        REAL PLL_KI;
        REAL smo_gain_scale; // 几倍反电势
    } chixu;
#define chixu OBSV.chixu
#endif

/* Park.Sul 2014 FADO in replace of CM */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Park_Sul

    struct Declare_ParkSul2014{
        #define NS_PARK_SUL_2014 12
        REAL x[NS_PARK_SUL_2014];
        REAL xD[2]; // estimated disturbance
        REAL xPsi1[2]; // corrected stator flux estimate
        REAL xHatPsi1[2]; // auxilliary stator flux estimate (not used)
        REAL xPsi2[2]; // rotor flux estimate
        REAL xPsi2_Limited[2]; // limited rotor flux
        REAL xPsi2_Amplitude;
        REAL xT2S_1[2];
        REAL xT2S_2[2];
        REAL t2s_1_sin_error;
        REAL t2s_2_sin_error;
        REAL xCM_state[2];

        REAL xTheta_d;
        REAL xOmg;

        REAL theta_d; // angle of estimated active flux

        REAL output_error[2];
        REAL internal_error[2];

        REAL omega_f; // estimated stator flux angular speed

        REAL T2S_1_KP;
        REAL T2S_1_KI;
        REAL T2S_2_KP;
        REAL T2S_2_KI;
        REAL CM_KP;
        REAL CM_KI;
        REAL k_df;
        REAL k_af;
        REAL limiter_KE;
        REAL limiter_Flag;
    } parksul;
#define parksul OBSV.parksul
#endif

/* Natural Speed Observer with active flux concept Chen2020 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_NSOAF

    struct Chen20_NSO_AF{
        #define NS_CHEN_2020 6
        REAL xIq;
        REAL xOmg;
        REAL xTL;
        REAL x[NS_CHEN_2020];
        REAL xBest[3]; // the actual 3 states

        REAL KP;
        REAL KI;
        REAL KD;
        REAL omega_ob; // one parameter tuning
        REAL set_omega_ob;

        REAL output_error; // \tilde i_q

        REAL active_power_real;
        REAL active_power_est;
        REAL active_power_error;

        REAL xTem;

        /* Select Signals from Block Diagram*/
        REAL LoadTorquePI;
        REAL load_torque_pid_output;
        REAL q_axis_voltage;
    } nsoaf;
#define nsoaf OBSV.nsoaf
#endif

/* Farza 2009 HGO */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Farza_2009

    struct FARZA09_EEMF_HGO{
        REAL xPsi[2];
        REAL xEemf[2];
        REAL xOmg;
        REAL xGammaOmg;
        REAL xPhi[4];

        // output
        REAL theta_d;

        REAL output_error[2]; // \varepsilon

        REAL vartheta;
        REAL vartheta_inv;
    } hgo4eemf;
#define hgo4eemf OBSV.hgo4eemf
#endif

/* CJH EEMF AO Design */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_CJH_EEMF

    struct CJH_EEMF_AO_Design{
        REAL xPsi[2];
        REAL xChi[2];
        REAL xOmg;

        // output
        REAL xEemf[2];
        REAL theta_d;

        REAL xEta[2];     // \eta = G(p)\phi\hat\omega
        REAL xVarsigma[2];

        REAL xUpsilon[2]; // \Upsilon = [G(p) \phi]
        REAL xZeta[2];

        REAL output_error[2]; // \varepsilon
        REAL effective_output_error[2]; // \varepsilon_\mathrm{eff}

        REAL k1;
        REAL k2;
        REAL gamma_omg;
    } cjheemf;
#define cjheemf OBSV.cjheemf
#endif

/* Harnefors 2006 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Harnefors_2006

    struct Harnefors2006{
        REAL theta_d;
        REAL omg_elec;

        REAL deriv_id;
        REAL deriv_iq;

        // SVF for d/q current derivative
        REAL svf_p0;
        REAL xSVF_curr[2];
        REAL xSVF_prev[2];
        REAL is_dq[2];
        REAL is_dq_curr[2];
        REAL is_dq_prev[2];
        REAL pis_dq[2];
    } harnefors;
#define harnefors OBSV.harnefors
#endif
};
extern struct ObserverForExperiment OBSV;

#if PC_SIMULATION
// Allow for easy access to child structs
// #define qiaoxia OBSV.qiaoxia
// #define chixu OBSV.chixu
// #define parksul OBSV.parksul
// #define nsoaf OBSV.nsoaf
// #define hgo4eemf OBSV.hgo4eemf
// #define cjheemf OBSV.cjheemf
// #define harnefors OBSV.harnefors
#else
#endif

void init_QiaoXia2013();
void init_ChiXu2009();
void init_parksul2014();
void init_nsoaf();
void init_hgo4eemf();
void init_cjheemf();
void init_harnefors();


#define rk4 OBSV.rk4
void init_rk4();

/* Macro for External Access Interface */
#define US(X)   rk4.us[X]
#define IS(X)   rk4.is[X]
#define US_C(X) rk4.us_curr[X] // 当前步电压是伪概念，测量的时候，没有电压传感器，所以也测量不到当前电压；就算有电压传感器，由于PWM比较寄存器没有更新，输出电压也是没有变化的。
#define IS_C(X) rk4.is_curr[X]
#define US_P(X) rk4.us_prev[X]
#define IS_P(X) rk4.is_prev[X]

/* Call all pmsm observers at one place here */
void pmsm_observers();
void init_pmsm_observers();

/* Macros for using SVF in Harnefors 2006 */
    // void init_harnefors();
    // void harnefors_scvm();
    #define SVF_POLE_0_VALUE (2000*2*M_PI) /* 定子电阻在高速不准确，就把SVF极点加大！加到3000反而比20000要差。*/
    #define SVF_POLE_0 harnefors.svf_p0 
    #define SVF_C(X)   harnefors.xSVF_curr[X]
    #define SVF_P(X)   harnefors.xSVF_prev[X]
    #define IDQ(X)     harnefors.is_dq[X]
    #define IDQ_C(X)   harnefors.is_dq_curr[X]
    #define IDQ_P(X)   harnefors.is_dq_prev[X]
    #define PIDQ(X)    harnefors.pis_dq[X]


#endif
#endif
