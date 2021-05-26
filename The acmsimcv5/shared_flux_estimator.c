#include "ACMSim.h"
#define AFE_11_OHTANI_1992 0
#define AFE_12_HOLTZ_QUAN_2002 0
#define AFE_13_LASCU_ANDREESCUS_2006 0
#define AFE_14_MY_PROPOSED 0
#define AFE_21_HU_WU_1998 0
#define AFE_22_STOJIC_2015 0
#define AFE_23_SCVM_HARNEFORS_2003 0
#define AFE_24_OE 0
#define AFE_25_VM_CM_FUSION 1 // See ParkSul2014Follower2020
#define AFE_31_HOLT_QUAN_2003_LPF_ORIGINIAL 0
#define AFE_32_HOLT_QUAN_2003_INTEGRATOR 0
#define AFE_33_EXACT_OFFSET_COMPENSATION 0
#define AFE_34_ADAPTIVE_LIMIT 0
#define AFE_35_SATURATION_TIME_DIFFERENCE 0
#define AFE_36_TOP_BUTT_EXACT_COMPENSATION 0

/*****************************/
/* Shared Solver and typedef */
REAL one_over_six = 1.0/6.0;
#define CJH_STYLE_RK4_OBSERVER_RAW_CODE                                  \
    US(0) = US_P(0);                                                     \
    US(1) = US_P(1);                                                     \
    IS(0) = IS_P(0);                                                     \
    IS(1) = IS_P(1);                                                     \
    (*fp)(t, x, fx);                                                     \
    for(i=0;i<NS;++i){                                                   \
        k1[i] = fx[i] * hs;                                              \
        xk[i] = x[i] + k1[i]*0.5;                                        \
    }                                                                    \
                                                                         \
    IS(0) = 0.5*(IS_P(0)+IS_C(0));                                       \
    IS(1) = 0.5*(IS_P(1)+IS_C(1));                                       \
    (*fp)(t, xk, fx);                                                    \
    for(i=0;i<NS;++i){                                                   \
        k2[i] = fx[i] * hs;                                              \
        xk[i] = x[i] + k2[i]*0.5;                                        \
    }                                                                    \
                                                                         \
    (*fp)(t, xk, fx);                                                    \
    for(i=0;i<NS;++i){                                                   \
        k3[i] = fx[i] * hs;                                              \
        xk[i] = x[i] + k3[i];                                            \
    }                                                                    \
                                                                         \
    IS(0) = IS_C(0);                                                     \
    IS(1) = IS_C(1);                                                     \
    (*fp)(t, xk, fx);                                                    \
    for(i=0;i<NS;++i){                                                   \
        k4[i] = fx[i] * hs;                                              \
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;  \
    }
void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 2
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_3states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 3
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 4
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 5
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_6states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 6
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_8states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 8
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
void general_10states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 10
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;
    CJH_STYLE_RK4_OBSERVER_RAW_CODE
    #undef NS
}
#if PC_SIMULATION == TRUE
    #define OFFSET_VOLTAGE_ALPHA (0*0.05*-0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  (0*0.05*+0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
#else
    #define OFFSET_VOLTAGE_ALPHA 0
    #define OFFSET_VOLTAGE_BETA  0
#endif

/***************************/
/* Various Flux Estimators */

#if AFE_11_OHTANI_1992 
#endif
#if AFE_12_HOLTZ_QUAN_2002 
#endif
#if AFE_13_LASCU_ANDREESCUS_2006 
#endif
#if AFE_14_MY_PROPOSED 
#endif
#if AFE_21_HU_WU_1998 
/**/
#endif
#if AFE_22_STOJIC_2015 
#endif
#if AFE_23_SCVM_HARNEFORS_2003 
#endif
#if AFE_24_OE || AFE_25_VM_CM_FUSION // See ParkSul2014Follower2020
    struct ActiveFluxEstimator AFEOE;
    void rhf_ActiveFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2], x[3]: integral action compensating offset voltage

        #define MOTOR (*CTRL.motor)

        /* Calculate psi_2 from psi_1, Lq and i_ab */
        AFEOE.psi_2[0] = x[0] - MOTOR.Lq * IS(0);
        AFEOE.psi_2[1] = x[1] - MOTOR.Lq * IS(1);

        /* Output: Flux amplitude */
        AFEOE.active_flux_ampl = sqrt(AFEOE.psi_2[0]*AFEOE.psi_2[0] + AFEOE.psi_2[1]*AFEOE.psi_2[1]);

        /* State: Cosine and Sine of the flux angle */
        REAL active_flux_ampl_inv=0.0;
        if(AFEOE.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/AFEOE.active_flux_ampl;
        } 
        REAL cosT = AFEOE.psi_2[0] * active_flux_ampl_inv;
        REAL sinT = AFEOE.psi_2[1] * active_flux_ampl_inv;

        /* Input: Current at dq frame and KActive */
        REAL iDQ_at_current_step[2];
        iDQ_at_current_step[0] = AB2M(IS(0), IS(1), cosT, sinT);
        iDQ_at_current_step[1] = AB2T(IS(0), IS(1), cosT, sinT);
        REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_at_current_step[0];

        #if AFE_24_OE
            /* Output: Current error & estimate */
            REAL current_estimate[2];
            current_estimate[0] = MOTOR.Lq_inv * (x[0] - KActive*cosT);
            current_estimate[1] = MOTOR.Lq_inv * (x[1] - KActive*sinT);
            AFEOE.output_error[0] = IS(0) - current_estimate[0];
            AFEOE.output_error[1] = IS(1) - current_estimate[1];
        #endif

        #if AFE_25_VM_CM_FUSION // note this will potentially overwrite results of AFE_24_OE
            /* Output: Flux mismatch to KActive (CM) */
            AFEOE.output_error[0] = KActive*cosT - AFEOE.psi_2[0];
            AFEOE.output_error[1] = KActive*sinT - AFEOE.psi_2[1];
        #endif

        /* State: stator emf as the derivative of staotr flux, psi_1 */
        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA;
        emf[1] = US(1) - MOTOR.R*IS(1) + 1*OFFSET_VOLTAGE_BETA ;
        fx[0] = emf[0] \
            /*P*/+ AFEOE.ActiveFlux_KP * AFEOE.output_error[0] \
            /*I*/+ x[2];
        fx[1] = emf[1] \
            /*P*/+ AFEOE.ActiveFlux_KP * AFEOE.output_error[1] \
            /*I*/+ x[3];

        /* State: disturbance/offset estimated by an integrator */
        fx[2] = AFEOE.ActiveFlux_KI * AFEOE.output_error[0];
        fx[3] = AFEOE.ActiveFlux_KI * AFEOE.output_error[1];
    }
    void Main_the_active_flux_estimator(){
        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_ActiveFluxEstimator_Dynamics, CTRL.timebase, AFE_USED.x, CL_TS);
        // Unpack x
        AFEOE.psi_1[0]    = AFEOE.x[0];
        AFEOE.psi_1[1]    = AFEOE.x[1];
        AFEOE.u_offset[0] = AFEOE.x[2];
        AFEOE.u_offset[1] = AFEOE.x[3];

        // active flux
        AFEOE.psi_2[0] = AFEOE.x[0] - CTRL.motor->Lq * IS(0);
        AFEOE.psi_2[1] = AFEOE.x[1] - CTRL.motor->Lq * IS(1);
        AFEOE.active_flux_ampl = sqrt(AFEOE.psi_2[0]*AFEOE.psi_2[0] + AFEOE.psi_2[1]*AFEOE.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(AFEOE.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/AFEOE.active_flux_ampl;
        } 
        AFEOE.cosT = AFEOE.psi_2[0] * active_flux_ampl_inv;
        AFEOE.sinT = AFEOE.psi_2[1] * active_flux_ampl_inv;

        AFEOE.theta_d = atan2(AFEOE.psi_2[1], AFEOE.psi_2[0]);
    }
#endif
#if AFE_31_HOLT_QUAN_2003_LPF_ORIGINIAL 
#endif
#if AFE_32_HOLT_QUAN_2003_INTEGRATOR 
#endif
#if AFE_33_EXACT_OFFSET_COMPENSATION 
#endif
#if AFE_34_ADAPTIVE_LIMIT 
#endif
#if AFE_35_SATURATION_TIME_DIFFERENCE 
#endif
#if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
#endif







void test_flux_estimators(){
    Main_the_active_flux_estimator();
}




void afe_one_parameter_tuning(REAL omega_est){
    AFEOE.ActiveFlux_KP = 2* omega_est;
    AFEOE.ActiveFlux_KI = omega_est*omega_est;

    #if PC_SIMULATION
    printf("Init AFEOE:\n");
    printf("\tomega_est=%g\n", omega_est);
    printf("\tAFEOE.ActiveFlux_KP=%g\n", AFEOE.ActiveFlux_KP);
    printf("\tAFEOE.ActiveFlux_KI=%g\n", AFEOE.ActiveFlux_KI);
    #endif
}
void init_afe(){
    AFEOE.x[0] = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // TODO：这里假设了初始位置是alpha轴对着d轴的
    AFEOE.x[1] = 0.0;    
    AFEOE.ActiveFlux_KP = AFEOE_KP;
    AFEOE.ActiveFlux_KI = AFEOE_KI;
    AFEOE.set_omega_est = AFEOE_OMEGA_ESTIMATOR;
    AFEOE.omega_est = AFEOE.set_omega_est;
    afe_one_parameter_tuning(AFEOE.omega_est);
}

