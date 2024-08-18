#ifndef ADD_PMSM_OBSERVER_H
#define ADD_PMSM_OBSERVER_H

    /* Commissioning */
    #define EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B TRUE
    #if PC_SIMULATION
        #define ENABLE_COMMISSIONING FALSE /*Simulation*/
        #define SELF_COMM_INVERTER FALSE
        #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE
    #else
        #define ENABLE_COMMISSIONING FALSE /*Experiment*/
        #define SELF_COMM_INVERTER FALSE
        #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE
        /*As we use (*CTRL).o->iab_cmd for look up, now dead-time compensation during ENABLE_COMMISSIONING is not active*/
    #endif

    /* Select Algorithm 2*/
        #define ALG_NSOAF 1
        #define ALG_Park_Sul 2
        #define ALG_Chi_Xu 3
        #define ALG_Qiao_Xia 4
        #define ALG_CJH_EEMF 5
        #define ALG_Farza_2009 6
        #define ALG_Harnefors_2006 7
        #define ALG_ESOAF 10

    // #define SELECT_ALGORITHM ALG_ESOAF
    #define SELECT_ALGORITHM ALG_NSOAF
    // #define SELECT_ALGORITHM ALG_Chi_Xu

        #if SELECT_ALGORITHM == ALG_Chi_Xu
            #define PMSM_ELECTRICAL_SPEED_FEEDBACK    OBSV.chixu.xOmg
            #define PMSM_ELECTRICAL_POSITION_FEEDBACK OBSV.chixu.theta_d
        #elif SELECT_ALGORITHM == ALG_NSOAF
            #define PMSM_ELECTRICAL_SPEED_FEEDBACK    OBSV.nsoaf.xOmg // OBSV.harnefors.omg_elec
            #define PMSM_ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d // OBSV.harnefors.theta_d
        #elif SELECT_ALGORITHM == ALG_ESOAF
            #define PMSM_ELECTRICAL_SPEED_FEEDBACK    (-OBSV.esoaf.xOmg) // 薄片电机实验正iq产生负转速
            #define PMSM_ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d
        #else
            // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    G.omg_elec
            // #define PMSM_ELECTRICAL_POSITION_FEEDBACK G.theta_d

            // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    parksul.xOmg
            // #define PMSM_ELECTRICAL_POSITION_FEEDBACK parksul.theta_d

            // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    qiaoxia.xOmg
            // #define PMSM_ELECTRICAL_POSITION_FEEDBACK qiaoxia.theta_d

            #define PMSM_ELECTRICAL_SPEED_FEEDBACK    (*CTRL).i->omg_elec
            #define PMSM_ELECTRICAL_POSITION_FEEDBACK (*CTRL).i->theta_d_elec
        #endif

    /* Tuning Algorithm 2 */
    #define LOW_SPEED_OPERATION  1
    #define HIGH_SPEED_OPERATION 2
    #define OPERATION_MODE HIGH_SPEED_OPERATION

        /* Park.Sul 2014 FADO in replace of CM */
        #define PARK_SUL_OPT_1 (2*M_PI*60)
        #define PARK_SUL_OPT_2 (2*M_PI*35)
            #define PARK_SUL_T2S_1_KP (PARK_SUL_OPT_1*2)
            #define PARK_SUL_T2S_1_KI (PARK_SUL_OPT_1*PARK_SUL_OPT_1)
            #define PARK_SUL_T2S_2_KP (PARK_SUL_OPT_2*2)
            #define PARK_SUL_T2S_2_KI (PARK_SUL_OPT_2*PARK_SUL_OPT_2)
        #define PARK_SUL_CM_OPT 5 // [rad/s] pole placement
            #define PARK_SUL_CM_KP (PARK_SUL_CM_OPT*2)
            #define PARK_SUL_CM_KI (PARK_SUL_CM_OPT*PARK_SUL_CM_OPT)

    /* Chi.Xu 2009 SMO for EMF of SPMSM (Coupled position estimation via MRAS) */
    #define CHI_XU_USE_CONSTANT_SMO_GAIN TRUE
        #define CHI_XU_SIGMOID_COEFF  500 /*比200大以后，在实验中无感速度稳态误差不会再减小了，但是会影响慢反转*/
    #if OPERATION_MODE == LOW_SPEED_OPERATION
        /* note ell4Zeq is -0.5 */
        #define CHI_XU_USE_CONSTANT_LPF_POLE TRUE
        #if PC_SIMULATION
            #define CHI_XU_SMO_GAIN_SCALE 10.0  //2
            #define CHI_XU_LPF_4_ZEQ    5.0   //10.0
        #else
            #define CHI_XU_SMO_GAIN_SCALE 10 /*取2实验无感稳态不稳，取5慢反转勉强成功，取10慢反转成功*/
            #define CHI_XU_LPF_4_ZEQ    (5.0) /*这项过大（eg=100）会导致角度稳态误差，忘记了你就试试看，取=2，=5，=10，=100分别仿！真！看看。*/
        #endif

        #define CHI_XU_SPEED_PLL_KP (500*2.0) // [rad/s]
        #define CHI_XU_SPEED_PLL_KI (500*500.0)
    #elif OPERATION_MODE == HIGH_SPEED_OPERATION
        /* note ell4Zeq will become 1 */
        #define CHI_XU_SMO_GAIN_SCALE  1.5
        #define CHI_XU_LPF_4_ZEQ       10.0
        #define CHI_XU_USE_CONSTANT_LPF_POLE FALSE
        #define CHI_XU_SPEED_PLL_KP (2*500) // [rad/s] 3000 = 阶跃转速不震荡，8000=阶跃转速很震荡
        #define CHI_XU_SPEED_PLL_KI (10e4) // 从350000减少为150000，可以减少稳态估计转速的波动
    #endif

    /* Qiao.Xia 2013 SMO for EMF of SPMSM */
        #define QIAO_XIA_SIGMOID_COEFF  5000 //200 // 20
        #define QIAO_XIA_SMO_GAIN       1.5 //1.5     // 1.5
        #define QIAO_XIA_MRAS_GAIN      500 //500       // 50
        #define QIAO_XIA_ADAPT_GAIN     500 //2000 // 250 // 100

    /* CHEN 2020 NSO with Active Flux Concept */
        // #define NSOAF_SPMSM // use AP Error
        #define NSOAF_IPMSM // use only OE
        #define TUNING_IGNORE_UQ TRUE
        #define NSOAF_OMEGA_OBSERVER 300 // >150 [rad/s] // cannot be too small (e.g., 145, KP will be negative),
            #define NSOAF_TL_P (1) // 1 for experimental starting // 4 for 1500 rpm // 2 for 800 rpm
            #define NSOAF_TL_I (20)
            #define NSOAF_TL_D (0)

    /* CHEN 2021 ESO with Active Flux Concept */
        // #define ESOAF_OMEGA_OBSERVER 10
        // #define ESOAF_OMEGA_OBSERVER 30 // 30 gives acceptable steady state speed ripple, 200
        // #define ESOAF_OMEGA_OBSERVER 150 // 150 gives acceptable disturbance rejection when sudden 3 A load is applied and keeps the system not stop when Vdc changes from 150 V to 300 V.
        #define ESOAF_OMEGA_OBSERVER 200 // 200 gives acceptable disturbance rejection when load changes between 1.5 A and 3 A.

    /* Farza 2009 for EMMF */
        #define FARZA09_HGO_EEMF_VARTHETA 10
        #define FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE 10

    /* CJH EEMF AO Design */
        #define CJH_EEMF_K1 (100)
        #define CJH_EEMF_K2 (CJH_EEMF_K1*CJH_EEMF_K1*0.25) // see my TCST paper@(18)
        #define CJH_EEMF_GAMMA_OMEGA (5e6)

    /* Harnefors 2006 */








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
// #define OBSV.chixu OBSV.OBSV.chixu
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
// #define OBSV.nsoaf OBSV.OBSV.nsoaf
#endif

/* Extended State Observer with active flux concept Chen2021 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_ESOAF

    struct Chen21_ESO_AF{
        #define NS_CHEN_2021 4
        REAL xPos;
        REAL xOmg;
        REAL xTL;
        REAL xPL; // rotatum
        REAL x[NS_CHEN_2021];
        REAL ell[NS_CHEN_2021];

        int bool_ramp_load_torque; // TRUE for 4th order ESO
        REAL omega_ob; // one parameter tuning
        REAL set_omega_ob;

        REAL output_error_sine; // sin(\tilde\vartheta_d)
        REAL output_error; // \tilde\vartheta_d, need to detect bound jumping 

        REAL xTem;
    } esoaf;
// #define OBSV.esoaf OBSV.OBSV.esoaf
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
// #define OBSV.hgo4eemf OBSV.OBSV.hgo4eemf
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
// #define OBSV.cjheemf OBSV.OBSV.cjheemf
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
// #define harnefors OBSV.harnefors
#endif
};
extern struct ObserverForExperiment OBSV;


void init_QiaoXia2013();
void init_ChiXu2009();
void init_parksul2014();
void init_nsoaf();
void init_hgo4eemf();
void init_cjheemf();
void init_harnefors();


/* Call all pmsm observers at one place here */
void pmsm_observers();
void init_pmsm_observers();

/* Macros for using SVF in Harnefors 2006 */
    // void init_harnefors();
    // void harnefors_scvm();
    #define SVF_POLE_0_VALUE (2000*2*M_PI) /* 定子电阻在高速不准确，就把SVF极点加大！加到3000反而比20000要差。*/
    #define SVF_POLE_0 OBSV.harnefors.svf_p0 
    #define SVF_C(X)   OBSV.harnefors.xSVF_curr[X]
    #define SVF_P(X)   OBSV.harnefors.xSVF_prev[X]
    #define IDQ(X)     OBSV.harnefors.is_dq[X]
    #define IDQ_C(X)   OBSV.harnefors.is_dq_curr[X]
    #define IDQ_P(X)   OBSV.harnefors.is_dq_prev[X]
    #define PIDQ(X)    OBSV.harnefors.pis_dq[X]


#endif
