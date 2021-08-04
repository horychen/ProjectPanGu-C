
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
#define AFE_35_SATURATION_TIME_DIFFERENCE 1
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
            REAL active_flux_ampl_lpf;
            REAL theta_d;
            REAL output_error[2];    // CM mismatch 
            REAL output_error_dq[2]; // CM mismatch to dq frame
            REAL psi_1[2]; // Stator Flux Estimate
            REAL psi_2[2]; // Active Flux Estimate
            REAL u_offset[2];
            /* State */
            REAL x[4];
            REAL cosT;
            REAL sinT;
            REAL dot_product; // = emf . psi_1
            REAL cross_product;
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
        struct Variables_Holtz2003{
            REAL emf_stator[2];

            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL psi_2_ampl_lpf;
            REAL u_offset[2];
            REAL psi_2_prev[2];

            REAL psi_1_nonSat[2];
            REAL psi_2_nonSat[2];

            REAL psi_1_min[2];
            REAL psi_1_max[2];
            REAL psi_2_min[2];
            REAL psi_2_max[2];

            REAL theta_d;
            REAL cosT;
            REAL sinT;

            REAL rs_est;
            REAL rreq_est;

            REAL Delta_t;
            REAL Delta_t_last;

            REAL u_off_direct_calculated[2];          //

            REAL u_off_original_lpf_input[2];         // holtz03 original (but I uses integrator instead of LPF)
            REAL u_off_saturation_time_correction[2]; // saturation time based correction
            REAL u_off_calculated_increment[2];       // exact offset calculation for compensation
            REAL gain_off;

            long int count_negative;
            long int count_positive;

            int    flag_pos2negLevelA[2];
            int    flag_pos2negLevelB[2];
            REAL time_pos2neg[2];
            REAL time_pos2neg_prev[2];

            int    flag_neg2posLevelA[2];
            int    flag_neg2posLevelB[2];
            REAL time_neg2pos[2];
            REAL time_neg2pos_prev[2];

            REAL psi_aster_max;

            REAL sat_min_time[2];
            REAL sat_max_time[2];
            REAL sat_min_time_reg[2];
            REAL sat_max_time_reg[2];
            REAL extra_limit;
            int flag_limit_too_low;

            // REAL ireq[2];
            REAL field_speed_est;
            REAL omg_est;
            REAL slip_est;
        } htz;
    #endif
    #if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
    #endif
};
extern struct SharedFluxEstimatorForExperiment FE;


#define AFEOE (FE.AFEOE)
#define huwu (FE.huwu)

#define htz (FE.htz)

void simulation_test_flux_estimators();
    void Main_the_active_flux_estimator();
    void MainFE_HuWu_1998();
    void Main_VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();

void init_afe();
void init_FE();
void init_Holtz2003();

#endif
