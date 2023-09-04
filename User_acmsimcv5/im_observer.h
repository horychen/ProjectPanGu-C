#ifndef ADD_IM_OBSERVER_H
#define ADD_IM_OBSERVER_H
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

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

/* EXAMPLE */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_EXAMPLE

    struct Declare_EXAMPLE{
        #define NS_EXAMPLE 5
        REAL x[NS_EXAMPLE];
    } obsv_example;
#endif

};
extern struct ObserverForExperiment OBSV;



struct ObserverControl{

    REAL k_AP_I;
    REAL k_AP_P;
    REAL k_AP_D;
    REAL k_RP_I;
    REAL k_RP_P;
    REAL k_RP_D;

    REAL xIs[2];    // \psi_\sigma
    REAL xPsiMu[2];       // \psi_\mu
    REAL xOmg;
    REAL xTL; 
    REAL xTL_integral_part_AP; 
    REAL xTL_integral_part_RP; 
    REAL xTem;
    // REAL refined_omg;
    REAL actual_iTs;

    REAL xUps_al[6];     // The alpha component of the filtered regressors
    REAL xUps_be[6];     // The beta component of the filtered regressors
    REAL xTheta[2]; 

    REAL mismatch[3];
    REAL error[2];
    REAL varepsilon_AP;
    REAL varepsilon_RP;

    REAL epsilon_AP; // estimate of varepsilon
    REAL varsigma_AP; // estimate of dot varepsilon
    REAL epsilon_RP; // estimate of varepsilon
    REAL varsigma_RP; // estimate of dot varepsilon
    REAL lambda1;
    REAL lambda2;

    REAL taao_alpha; 
    REAL taao_omg_integralPart; // 纯积分的自适应律就用不到这个
    REAL taao_speed; // 机械速度rpm！再强调一遍，不是电气速度rpm，而是机械速度rpm。

    REAL timebase;
    REAL Ts;

    REAL omega_e;
    REAL Tem;

    REAL taao_flux_cmd;
    int taao_flux_cmd_on;

    REAL cosT;
    REAL sinT;
    REAL theta_M;

    REAL actual_flux[2];
    REAL actual_TL;
    REAL actual_z;

    REAL k_Gopinath;
    REAL k_1minusGopinath_inv;
    REAL xXi[2];
};
extern struct ObserverControl ob;

void rk4_init();
void observer_init();
void simulation_only_flux_estimator();
void observer_marino2005();

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
};
extern struct Chen21_ESO_AF esoaf;
void Main_esoaf_chen2021();

/********************************************
 * Collections of VM based Flux Estimators 
 ********************************************/
void flux_observer();

struct Variables_SimulatedVM{
    REAL emf[2];
    REAL emf_DQ[2];

    REAL psi_1[2];
    REAL psi_2[2];
    REAL psi_2_ampl;
    REAL u_offset[2];
    REAL psi_D2_ode1_v2;
    REAL psi_Q2_ode1_v2;

    REAL psi_D1_ode1;
    REAL psi_Q1_ode1;
    REAL psi_D2_ode1;
    REAL psi_Q2_ode1;

    REAL psi_D1_ode4;
    REAL psi_Q1_ode4;
    REAL psi_D2_ode4;
    REAL psi_Q2_ode4;

    REAL psi_D2;
    REAL psi_Q2;
};
extern struct Variables_SimulatedVM simvm;


// #define VM_HoltzQuan2003 VM_Saturated_ExactOffsetCompensation
void VM_Ohtani1992();
void VM_HoltzQuan2002();
void VM_HoltzQuan2003();
void VM_LascuAndreescus2006();
void VM_HuWu1998();
void VM_Stojic2015();
void VM_Harnefors2003_SCVM();
/* A */
void VM_ExactCompensation();
/* B */
void VM_ProposedCmdErrFdkCorInFrameRho();
/* C */
void VM_ClosedLoopFluxEstimator();

void VM_Saturated_ExactOffsetCompensation();
void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
void VM_Saturated_ExactOffsetCompensation_WithParallelNonSaturatedEstimator();
// void stableFME();

// extern struct Variables_Ohtani1992 ohtani;
// extern struct Variables_HuWu1998 huwu;
// extern struct Variables_HoltzQuan2002 holtz02;
// extern struct Variables_Holtz2003 htz;
// extern struct Variables_Harnefors2003_SCVM harnefors;
// extern struct Variables_LascuAndreescus2006 lascu;
// extern struct Variables_Stojic2015 stojic;
// extern struct Variables_fluxModulusEstimator fme;
// /* A */
// extern struct Variables_ExactCompensationMethod exact;
// /* B */
// extern struct Variables_ProposedxRhoFramePICorrectionMethod picorr;
// /* C */
// extern struct Variables_ClosedLoopFluxEstimator clest;

#endif
#endif
