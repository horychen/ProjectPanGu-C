#ifndef SHARED_FLUX_ESTIMATOR_H
#define SHARED_FLUX_ESTIMATOR_H


/* Macro for External Access Interface */
#define US(X)   OBSV.rk4.us[X]
#define IS(X)   OBSV.rk4.is[X]
#define US_C(X) OBSV.rk4.us_curr[X] // 当前步电压是伪概念，测量的时候，没有电压传感器，所以也测量不到当前电压；就算有电压传感器，由于PWM比较寄存器没有更新，输出电压也是没有变化的。
#define IS_C(X) OBSV.rk4.is_curr[X]
#define US_P(X) OBSV.rk4.us_prev[X]
#define IS_P(X) OBSV.rk4.is_prev[X]

void init_rk4();

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
#define AFE_35_SATURATION_TIME_DIFFERENCE  1
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
            // REAL u_offset_intermediate[2];
            // REAL u_offset_ultra_low_frequency_component[2];
            REAL sat_time_offset[2];
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

            REAL accumulated__u_off_saturation_time_correction[2];
            REAL sign__u_off_saturation_time_correction[2];

            long int count_negative_cycle;
            long int count_positive_cycle;

            long int count_negative_in_one_cycle[2];
            long int count_positive_in_one_cycle[2];
            long int negative_cycle_in_count[2];
            long int positive_cycle_in_count[2];

            int    flag_pos2negLevelA[2];
            int    flag_pos2negLevelB[2];
            REAL time_pos2neg[2];
            REAL time_pos2neg_prev[2];

            int    flag_neg2posLevelA[2];
            int    flag_neg2posLevelB[2];
            REAL time_neg2pos[2];
            REAL time_neg2pos_prev[2];

            REAL psi_aster_max;

            REAL maximum_of_sat_min_time[2];
            REAL maximum_of_sat_max_time[2];
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



    struct Variables_Ohtani1992{
        // in AB frame
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
        // in DQ frame
        REAL psi_DQ1[2];
        REAL psi_DQ2[2];
        REAL psi_DQ2_prev[2];
        REAL deriv_psi_DQ1[2];
    } ohtani;
    // struct Variables_HuWu1998{
    //     REAL psi_1[2];
    //     REAL psi_2[2];
    //     REAL psi_2_ampl;
    //     REAL u_offset[2];
    // } huwu;
    struct Variables_HoltzQuan2002{
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
    } holtz02;
    // struct Variables_Holtz2003{ moved to shared flux estimator.h
    struct Variables_Harnefors2003_SCVM{
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
        REAL lambda;
    } harnefors;
    struct Variables_LascuAndreescus2006{
        REAL x[4];
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
        REAL correction_integral_term[2];
    } lascu;
    struct Variables_Stojic2015{
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
    } stojic;
    struct Variables_fluxModulusEstimator{
        REAL psi_DQ2[2];
        REAL psi_DQ1[2];
        REAL temp;
    } fme;
    struct Variables_ExactCompensationMethod{
        // D-Q
        REAL psi_DQ1[2];
        REAL psi_DQ2[2];
        REAL psi_DQ2_prev[2];

        // alpha-beta
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];

        REAL psi_2_prev[2];
        REAL psi_2_pprev[2];

        // sum_up_method
            // TOP
            int flag_top2top[2];
            REAL last_time_reaching_top[2];
            REAL Delta_time_top2top[2];
            REAL change_in_psi_2_top[2];
            REAL psi_2_last_top[2];
            REAL u_offset_estimated_by_psi_2_top_change[2];

            // BUTT
            int flag_butt2butt[2];
            REAL last_time_reaching_butt[2];
            REAL Delta_time_butt2butt[2];
            REAL change_in_psi_2_butt[2];
            REAL psi_2_last_butt[2];
            REAL u_offset_estimated_by_psi_2_butt_change[2];

            // REAL u_offset[2];
            REAL offset_voltage_compensation[2];

            REAL first_time_enter_top[2];
            REAL first_time_enter_butt[2];

            // 这个没用！
            // REAL top2top_sum[2];
            REAL top2top_count[2];
            // REAL psi_2_offset[2];
            // REAL u_offset_estimated_by_psi_2_sumup[2];

            REAL butt2butt_count[2];

            REAL filtered_compensation[2]; // psi_offset
            REAL filtered_compensation2[2];
            REAL u_offset_estimated_by_top_minus_butt[2];
            REAL u_offset_estimated_by_integration_correction[2];
            REAL u_offset_error[2];

            REAL bool_compensate_psi_offset[2];

            REAL psi_2_real_output[2];

            REAL psi_2_output[2];
            REAL psi_2_output2[2];
            REAL psi_2_output_last_top[2];
            REAL psi_2_output2_last_top[2];
            REAL psi_2_output_last_butt[2];
            REAL psi_2_output2_last_butt[2];

            REAL current_Lissa_move[2];
            REAL last_Lissa_move[2];

        REAL correction_integral_term[2];

        /* zero_crossing_method (below) */
        REAL psi_2_max[2]; // always not equal to zero
        REAL psi_2_min[2]; // always not equal to zero
        REAL psi_2_max_temp[2]; // can be reset to zero
        REAL psi_2_min_temp[2]; // can be reset to zero
        REAL psi_2_maxmin_difference[2];
        REAL psi_2_last_zero_level[2];
        REAL psi_2_last_zero_level_inLoopTheSame[2];
        REAL psi_2_moved_distance_in_LissajousPlot[2];
        // REAL offset_voltage[2];

        REAL flag_Pos_stageA[2];
        REAL flag_Pos_stageB[2];

        REAL flag_Neg_stageA[2];
        REAL flag_Neg_stageB[2];

        REAL Delta_t_PosMinusNeg[2];
        REAL Delta_t_NegMinusPos[2];
        REAL time_Pos[2];
        REAL time_Neg[2];

        REAL offset_voltage_by_zero_crossing[2];
    } exact;
    struct Variables_ProposedxRhoFramePICorrectionMethod{
        REAL x[4];
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
        REAL correction_integral_term[2];
        REAL theta_d;
        REAL cosT;
        REAL sinT;
    } picorr;
    struct Variables_ClosedLoopFluxEstimator{
        REAL x[5];
        REAL psi_1[2];
        REAL psi_2[2];
        REAL psi_2_ampl;
        REAL u_offset[2];
        REAL psi_dmu;
        REAL correction_integral_term[2];
    } clest;

};
extern struct SharedFluxEstimatorForExperiment FE;


// #define AFEOE (FE.AFEOE)
// #define huwu  (FE.huwu)
// #define htz   (FE.htz)

void simulation_test_flux_estimators();
    void Main_the_active_flux_estimator();
    void MainFE_HuWu_1998();
    void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();

void init_afe();
void init_FE();
void init_FE_htz(); // Holtz 2003

#endif
