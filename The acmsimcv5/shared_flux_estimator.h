
#ifndef SHARED_FLUX_ESTIMATOR_H
#define SHARED_FLUX_ESTIMATOR_H

#define AFE_11_OHTANI_1992 0
#define AFE_12_HOLTZ_QUAN_2002 0
#define AFE_13_LASCU_ANDREESCUS_2006 0
#define AFE_14_MY_PROPOSED 0
#define AFE_21_HU_WU_1998 1
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

#define AFE_40_JO_CHOI_METHOD 0

typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_3states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_6states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_8states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_10states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);

struct SharedFluxEstimatorForExperiment{
#if AFE_11_OHTANI_1992 
#endif
#if AFE_12_HOLTZ_QUAN_2002 
#endif
#if AFE_13_LASCU_ANDREESCUS_2006 
#endif
#if AFE_14_MY_PROPOSED 
#endif
#if AFE_21_HU_WU_1998
    struct Declare_HuWu1998{
        #define NS_HuWu1998 3
        /* State */
        REAL x[NS_HuWu1998];
        REAL psi_1[2]; // Stator Flux Estimate
        REAL psi_comp; // flux amplitude for compensation
        REAL cosT;
        REAL sinT;
        REAL psi_2_limited[2]; // Active flux limited 
        /* Coefficients */
        REAL KP; // alg 3
        REAL KI; // alg 3
        REAL tau_1_inv;
        REAL limiter_KE; // alg 2
        /* Output */
        REAL theta_d;
        REAL inner_product_normalized;
        REAL output_error[2]; // \tilde i_q
        REAL psi_2[2]; // Active Flux Estimate
        REAL stator_flux_ampl;
        REAL stator_flux_ampl_limited;
        REAL active_flux_ampl;
        REAL active_flux_ampl_limited;
    } huwu;
#endif
#if AFE_22_STOJIC_2015 
#endif
#if AFE_23_SCVM_HARNEFORS_2003 
#endif
#if AFE_24_OE || AFE_25_VM_CM_FUSION 
    struct ActiveFluxEstimator{
        /* Coefficients */
        REAL ActiveFlux_KP;
        REAL ActiveFlux_KI;
        REAL omega_est;
        REAL set_omega_est;
        /* Output */
        REAL active_flux_ampl;
        REAL theta_d;
        REAL output_error[2]; // \tilde i_q
        REAL psi_1[2]; // Stator Flux Estimate
        REAL psi_2[2]; // Active Flux Estimate
        REAL u_offset[2];
        /* State */
        REAL x[4];
        REAL cosT;
        REAL sinT;
        /* Ampl Limiter */
        int  limiter_Flag;
        REAL limiter_KE;
        REAL k_af;
        REAL psi_2_limited[2];
    } AFEOE;
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
};
extern struct SharedFluxEstimatorForExperiment FE;


#define AFEOE (FE.AFEOE)
#define huwu (FE.huwu)


void test_flux_estimators();
    void Main_the_active_flux_estimator();
    void MainFE_HuWu_1998();


void init_afe();
void init_FE();

#endif
