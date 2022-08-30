#include "ACMSim.h"

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
    US(0) = 0.5*(US_P(0)+US_C(0));                                       \
    US(1) = 0.5*(US_P(1)+US_C(1));                                       \
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
    US(0) = US_C(0);                                                     \
    US(1) = US_C(1);                                                     \
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
    #define OFFSET_VOLTAGE_ALPHA (0.5*-0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  (0.5*+0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
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
    void init_huwu(){
        huwu.tau_1_inv = AFE_21_HUWU_TAU_1_INVERSE;
        huwu.KP = AFE_21_HUWU_KP;
        huwu.KI = AFE_21_HUWU_KI;

        huwu.x[0] = MOTOR.KE;
        huwu.x[1] = 0.0;
        huwu.x[2] = MOTOR.KE;

        huwu.active_flux_ampl = MOTOR.KE;
        huwu.cosT = 1.0;
        huwu.sinT = 0.0;

        huwu.limiter_KE = 1.0*MOTOR.KE; /* HuWu1998: In order to eliminate dc component at the output, the limiting level should be set at a value equal to the actual flux amplitude. */
    }
    void rhf_HuWu_1998_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2], x[3]: compensation flux (integral part)

        #define xPsi1(X)  x[0+X]

        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0);
        emf[1] = US(1) - MOTOR.R*IS(1);

        /* 求定子磁链幅值的倒数 */
        REAL temp = 0.0; 
        huwu.stator_flux_ampl = sqrt(xPsi1(0)*xPsi1(0) + xPsi1(1)*xPsi1(1));
        if(huwu.stator_flux_ampl!=0){
            temp = 1.0 / huwu.stator_flux_ampl;
        }

    #define HU_WU_ALGORITHM_2
    #ifdef HU_WU_ALGORITHM_2
        /* 计算有功磁链分量 */
        huwu.psi_2[0] = xPsi1(0) - MOTOR.Lq * IS(0);
        huwu.psi_2[1] = xPsi1(1) - MOTOR.Lq * IS(1);
        /* 求有功磁链幅值的倒数 */
        huwu.active_flux_ampl = sqrt(huwu.psi_2[0]*huwu.psi_2[0] + huwu.psi_2[1]*huwu.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(huwu.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/huwu.active_flux_ampl;
        }
        /* The amplitude limiter for active flux */
        if(huwu.active_flux_ampl > huwu.limiter_KE){
            huwu.active_flux_ampl_limited = huwu.limiter_KE;
        }else{
            huwu.active_flux_ampl_limited = huwu.active_flux_ampl;
        }

        if(FALSE){
            /* 根据限幅后的有功磁链去求限幅后的定子磁链 */
            REAL stator_flux_limited[2];
            stator_flux_limited[0] = huwu.psi_2[0]*active_flux_ampl_inv * huwu.active_flux_ampl_limited + MOTOR.Lq*IS(0);
            stator_flux_limited[1] = huwu.psi_2[1]*active_flux_ampl_inv * huwu.active_flux_ampl_limited + MOTOR.Lq*IS(1);            
            /* 求限幅后的定子磁链幅值 */
            huwu.stator_flux_ampl_limited = sqrt(stator_flux_limited[0]*stator_flux_limited[0] + stator_flux_limited[1]*stator_flux_limited[1]);

            /* Correct by stator flux */
            fx[0] = emf[0] + huwu.tau_1_inv * (( huwu.stator_flux_ampl_limited ) * xPsi1(0) * temp - xPsi1(0) );
            fx[1] = emf[1] + huwu.tau_1_inv * (( huwu.stator_flux_ampl_limited ) * xPsi1(1) * temp - xPsi1(1) );
            fx[2] = 0.0;
        }else{
            /* Simplified (Correct by rotor flux) */
            fx[0] = emf[0] + huwu.tau_1_inv * (( huwu.active_flux_ampl_limited ) * huwu.psi_2[0] * active_flux_ampl_inv - huwu.psi_2[0] );
            fx[1] = emf[1] + huwu.tau_1_inv * (( huwu.active_flux_ampl_limited ) * huwu.psi_2[1] * active_flux_ampl_inv - huwu.psi_2[1] );
            fx[2] = 0.0;
        }

    #else
        #define xPsi_Comp x[2]
        /* Algorithm 3 */
        huwu.inner_product_normalized = (emf[0] * xPsi1(0) + emf[1] * xPsi1(1)) * temp;

        /* Compensation term is a flux amplitude going through an inverse Park transformation */
        fx[0] = emf[0] + huwu.tau_1_inv * ( (/*P*/+ huwu.KP * huwu.inner_product_normalized \
                                             /*I*/+ xPsi_Comp \
                                            ) * xPsi1(0) * temp - xPsi1(0) );
        fx[1] = emf[1] + huwu.tau_1_inv * ( (/*P*/+ huwu.KP * huwu.inner_product_normalized \
                                             /*I*/+ xPsi_Comp \
                                            ) * xPsi1(1) * temp - xPsi1(1) );

        /* State: disturbance/offset estimated by an integrator */
        fx[2] = huwu.KI * huwu.inner_product_normalized;

        #undef xPsi_Comp
    #endif
    #undef xPsi1
    }
    void MainFE_HuWu_1998(){
        // stator flux and integral states update
        general_3states_rk4_solver(&rhf_HuWu_1998_Dynamics, CTRL.timebase, huwu.x, CL_TS);
        // Unpack x
        huwu.psi_1[0] = huwu.x[0];
        huwu.psi_1[1] = huwu.x[1];
        huwu.psi_comp = huwu.x[2];

        // active flux and position
        huwu.psi_2[0] = huwu.x[0] - CTRL.motor->Lq * IS(0);
        huwu.psi_2[1] = huwu.x[1] - CTRL.motor->Lq * IS(1);
        huwu.active_flux_ampl = sqrt(huwu.psi_2[0]*huwu.psi_2[0] + huwu.psi_2[1]*huwu.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(huwu.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/huwu.active_flux_ampl;
            huwu.cosT = huwu.psi_2[0] * active_flux_ampl_inv;
            huwu.sinT = huwu.psi_2[1] * active_flux_ampl_inv;
        }else{
            huwu.cosT = 1.0;
            huwu.sinT = 0.0;
        }

        huwu.theta_d = atan2(huwu.psi_2[1], huwu.psi_2[0]);
    }
#endif
#if AFE_22_STOJIC_2015 
#endif
#if AFE_23_SCVM_HARNEFORS_2003 
#endif
#if AFE_24_OE || AFE_25_VM_CM_FUSION // See ParkSul2014Follower2020
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
        AFEOE.active_flux_ampl     = MOTOR.KE;
        AFEOE.active_flux_ampl_lpf = AFEOE.active_flux_ampl;
        AFEOE.cosT = 1.0;
        AFEOE.sinT = 0.0;

        AFEOE.x[0] = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // TODO：这里假设了初始位置是alpha轴对着d轴的
        AFEOE.x[1] = 0.0;
        AFEOE.ActiveFlux_KP = AFEOE_KP;
        AFEOE.ActiveFlux_KI = AFEOE_KI;
        AFEOE.set_omega_est = AFEOE_OMEGA_ESTIMATOR;
        AFEOE.omega_est = AFEOE.set_omega_est;
        #if AFE_25_VM_CM_FUSION
            // afe_one_parameter_tuning(AFEOE.omega_est);
        #endif

        AFEOE.limiter_KE = 1.15 * MOTOR.KE;
        AFEOE.k_af = 0.0 * 100 * 2 * M_PI;
    }
    void rhf_ActiveFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2], x[3]: integral action compensating offset voltage

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

        // #define DEBUG_USING_ACTUAL_CM
        #ifdef DEBUG_USING_ACTUAL_CM
            cosT = CTRL.S->cosT;
            sinT = CTRL.S->sinT;
        #endif

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

        #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED
            /* The amplitude limiter */
            if(AFEOE.active_flux_ampl > AFEOE.limiter_KE){
                AFEOE.psi_2_limited[0] = AFEOE.limiter_KE * AFEOE.psi_2[0]*active_flux_ampl_inv;
                AFEOE.psi_2_limited[1] = AFEOE.limiter_KE * AFEOE.psi_2[1]*active_flux_ampl_inv;
            }else{
                AFEOE.psi_2_limited[0] = AFEOE.psi_2[0];
                AFEOE.psi_2_limited[1] = AFEOE.psi_2[1];
            }
        #else
            AFEOE.psi_2_limited[0] = AFEOE.psi_2[0];
            AFEOE.psi_2_limited[1] = AFEOE.psi_2[1];
        #endif

        /* State: stator emf as the derivative of staotr flux, psi_1 */
        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA;
        emf[1] = US(1) - MOTOR.R*IS(1) + 1*OFFSET_VOLTAGE_BETA ;
        fx[0] = emf[0] \
            /*k_af*/- AFEOE.k_af*(AFEOE.psi_2[0] - AFEOE.psi_2_limited[0]) \
            /*P*/+ AFEOE.ActiveFlux_KP * AFEOE.output_error[0] \
            /*I*/+ x[2];
        fx[1] = emf[1] \
            /*k_af*/- AFEOE.k_af*(AFEOE.psi_2[1] - AFEOE.psi_2_limited[1]) \
            /*P*/+ AFEOE.ActiveFlux_KP * AFEOE.output_error[1] \
            /*I*/+ x[3];

        /* State: disturbance/offset estimated by an integrator */
        fx[2] = AFEOE.ActiveFlux_KI * AFEOE.output_error[0];
        fx[3] = AFEOE.ActiveFlux_KI * AFEOE.output_error[1];
    }
    REAL k_af_speed_Hz = 1.5;
    void Main_the_active_flux_estimator(){

        /* OPT for AFE */
        if(AFEOE.omega_est != AFEOE.set_omega_est){
            AFEOE.omega_est = AFEOE.set_omega_est;
            afe_one_parameter_tuning(AFEOE.omega_est);
        }

        /* Time-varying gains */
        if(fabs(CTRL.I->cmd_omg_elec)*MOTOR.npp_inv*ONE_OVER_2PI<k_af_speed_Hz){ // [Hz]
            AFEOE.k_af = 2*M_PI*100;
            AFEOE.limiter_Flag = TRUE;
        }else{
            AFEOE.k_af = 0.0;
            AFEOE.limiter_Flag = FALSE;
        }

        #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED == FALSE
            /* NO Need */
            /* NO Need */
            /* NO Need */
            AFEOE.k_af = 0.0;
        #endif

        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_ActiveFluxEstimator_Dynamics, CTRL.timebase, AFEOE.x, CL_TS);
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
            AFEOE.cosT = AFEOE.psi_2[0] * active_flux_ampl_inv;
            AFEOE.sinT = AFEOE.psi_2[1] * active_flux_ampl_inv;
        }else{
            AFEOE.cosT = 1.0;
            AFEOE.sinT = 0.0;
        }

        AFEOE.theta_d = atan2(AFEOE.psi_2[1], AFEOE.psi_2[0]);

        // Convert output error to dq frame
        // REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * CTRL.I->idq[0];
        /* TODO: 思考这个dq角度怎么选最好，要不要换成电压向量的角度而不是转子角度？ */
        AFEOE.active_flux_ampl_lpf = _lpf(AFEOE.active_flux_ampl, AFEOE.active_flux_ampl_lpf, 5);
        AFEOE.output_error_dq[0] = MOTOR.KActive - AFEOE.active_flux_ampl_lpf;
        AFEOE.output_error[0] = MOTOR.KActive*AFEOE.cosT - AFEOE.psi_2[0];
        AFEOE.output_error[1] = MOTOR.KActive*AFEOE.sinT - AFEOE.psi_2[1];
        AFEOE.output_error_dq[1] = sqrt(AFEOE.output_error[0]*AFEOE.output_error[0] + AFEOE.output_error[1]*AFEOE.output_error[1]);
        // if(CTRL.I->cmd_speed_rpm>0){
        //     AFEOE.output_error_dq[0] = AB2M(AFEOE.output_error[0], AFEOE.output_error[1], AFEOE.cosT, AFEOE.sinT);
        //     AFEOE.output_error_dq[1] = AB2T(AFEOE.output_error[0], AFEOE.output_error[1], AFEOE.cosT, AFEOE.sinT);
        // }else{
        //     AFEOE.output_error_dq[0] = AB2M(AFEOE.output_error[0], AFEOE.output_error[1], AFEOE.cosT, AFEOE.sinT);
        //     AFEOE.output_error_dq[1] = AB2T(AFEOE.output_error[0], AFEOE.output_error[1], AFEOE.cosT, AFEOE.sinT);
        // }

        // Research
        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0) + AFEOE.ActiveFlux_KP * AFEOE.output_error[0];
        emf[1] = US(1) - MOTOR.R*IS(1) + AFEOE.ActiveFlux_KP * AFEOE.output_error[1];
        AFEOE.dot_product = emf[0]*AFEOE.psi_1[0] + emf[1]*AFEOE.psi_1[1];
        AFEOE.cross_product = emf[0]*-AFEOE.psi_1[1] + emf[1]*AFEOE.psi_1[0];
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
    void init_Holtz2003(){

        htz.rs_est   = U_MOTOR_R;
        htz.rreq_est = U_MOTOR_RREQ;
        htz.psi_aster_max = U_MOTOR_KE + IM_FLUX_COMMAND_SINE_PART;

        int ind;
        for(ind=0;ind<2;++ind){
            htz.emf_stator[ind] = 0;

            htz.psi_1[ind] = 0;
            htz.psi_2[ind] = 0;
            htz.psi_2_prev[ind] = 0;
            htz.psi_2_ampl = U_MOTOR_KE;
            htz.psi_2_ampl_lpf = htz.psi_2_ampl;

            htz.psi_1_nonSat[ind] = 0;
            htz.psi_2_nonSat[ind] = 0;

            htz.psi_1_min[ind] = 0;
            htz.psi_1_max[ind] = 0;
            htz.psi_2_min[ind] = 0;
            htz.psi_2_max[ind] = 0;

            htz.Delta_t = 1;
            htz.u_offset[ind] = 0;
            htz.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
            htz.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
            htz.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
            htz.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

            htz.flag_pos2negLevelA[ind] = 0;
            htz.flag_pos2negLevelB[ind] = 0;
            htz.time_pos2neg[ind] = 0;
            htz.time_pos2neg_prev[ind] = 0;

            htz.flag_neg2posLevelA[ind] = 0;
            htz.flag_neg2posLevelB[ind] = 0;
            htz.time_neg2pos[ind] = 0;
            htz.time_neg2pos_prev[ind] = 0;    

            htz.sat_min_time[ind] = 0.0;
            htz.sat_max_time[ind] = 0.0;
            htz.sat_min_time_reg[ind] = 0.0;
            htz.sat_max_time_reg[ind] = 0.0;
            htz.extra_limit = 0.0;
            htz.flag_limit_too_low = FALSE;
        }
    }
    void rhf_Holtz2003_Dynamics(REAL t, REAL *x, REAL *fx){
        htz.emf_stator[0] = US(0) - htz.rs_est*IS(0) - htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
        htz.emf_stator[1] = US(1) - htz.rs_est*IS(1) - htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
        fx[0] = (htz.emf_stator[0]);
        fx[1] = (htz.emf_stator[1]);
    }
    void Main_VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit(){
        #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
        #define PSI_MU_ASTER_MAX htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
        #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT FALSE

        #define BOOL_USE_METHOD_LPF_INPUT        TRUE
        #define BOOL_USE_METHOD_INTEGRATOR_INPUT FALSE

        // Euler's method is shit at higher speeds
        htz.emf_stator[0] = US_C(0) - htz.rs_est*IS_C(0) - htz.u_offset[0];
        htz.emf_stator[1] = US_C(1) - htz.rs_est*IS_C(1) - htz.u_offset[1];
        // htz.psi_1[0] += CL_TS*(htz.emf_stator[0]);
        // htz.psi_1[1] += CL_TS*(htz.emf_stator[1]);

        // rk4 
        general_2states_rk4_solver(&rhf_Holtz2003_Dynamics, CTRL.timebase, htz.psi_1, CL_TS);
        htz.psi_2[0] = htz.psi_1[0] - CTRL.motor->Lq*IS_C(0);
        htz.psi_2[1] = htz.psi_1[1] - CTRL.motor->Lq*IS_C(1);
        htz.psi_2_ampl = sqrt(htz.psi_2[0]*htz.psi_2[0]+htz.psi_2[1]*htz.psi_2[1]);
        htz.psi_2_ampl_lpf = _lpf(htz.psi_2_ampl, htz.psi_2_ampl_lpf, 5);
        if(htz.psi_2_ampl!=0){
            REAL active_flux_ampl_inv = 0.0;
            active_flux_ampl_inv = 1.0/htz.psi_2_ampl;
            htz.cosT = htz.psi_2[0] * active_flux_ampl_inv;
            htz.sinT = htz.psi_2[1] * active_flux_ampl_inv;
        }else{
            htz.cosT = 1;
            htz.sinT = 0;
        }
        htz.theta_d = atan2(htz.psi_2[1], htz.psi_2[0]);

        htz.psi_1_nonSat[0] += CL_TS*(htz.emf_stator[0]);
        htz.psi_1_nonSat[1] += CL_TS*(htz.emf_stator[1]);
        htz.psi_2_nonSat[0] = htz.psi_1_nonSat[0] - CTRL.motor->Lq*IS_C(0);
        htz.psi_2_nonSat[1] = htz.psi_1_nonSat[1] - CTRL.motor->Lq*IS_C(1);

        if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
            // htz.psi_aster_max = CTRL.taao_flux_cmd + 0.05;
            htz.psi_aster_max = CTRL.I->cmd_psi + htz.extra_limit;
            // htz.psi_aster_max = CTRL.taao_flux_cmd;            
        }

        // 限幅是针对转子磁链限幅的
        if(htz.psi_2[0]    > PSI_MU_ASTER_MAX){
            htz.psi_2[0]   = PSI_MU_ASTER_MAX;
            htz.sat_max_time[0] += CL_TS;
        }else if(htz.psi_2[0] < -PSI_MU_ASTER_MAX){
            htz.psi_2[0]   = -PSI_MU_ASTER_MAX;
            htz.sat_min_time[0] += CL_TS;
        }
        if(htz.psi_2[1]    > PSI_MU_ASTER_MAX){
            htz.psi_2[1]   = PSI_MU_ASTER_MAX;
            htz.sat_max_time[1] += CL_TS;
        }else if(htz.psi_2[1] < -PSI_MU_ASTER_MAX){
            htz.psi_2[1]   = -PSI_MU_ASTER_MAX;
            htz.sat_min_time[1] += CL_TS;
        }
        // 限幅后的转子磁链，再求取限幅后的定子磁链
        htz.psi_1[0] = htz.psi_2[0] + CTRL.motor->Lq*IS_C(0);
        htz.psi_1[1] = htz.psi_2[1] + CTRL.motor->Lq*IS_C(1);

        // Speed Estimation
        if(TRUE){
            // htz.ireq[0] = CTRL.Lmu_inv*htz.psi_2[0] - IS_C(0);
            // htz.ireq[1] = CTRL.Lmu_inv*htz.psi_2[1] - IS_C(1);
            REAL temp;
            temp = (htz.psi_1[0]*htz.psi_1[0]+htz.psi_1[1]*htz.psi_1[1]);
            if(temp>0.001){
                htz.field_speed_est = - (htz.psi_1[0]*-htz.emf_stator[1] + htz.psi_1[1]*htz.emf_stator[0]) / temp;
            }
            temp = (htz.psi_2[0]*htz.psi_2[0]+htz.psi_2[1]*htz.psi_2[1]);
            if(temp>0.001){
                htz.slip_est = CTRL.motor->Rreq*(IS_C(0)*-htz.psi_2[1]+IS_C(1)*htz.psi_2[0]) / temp;
            }
            htz.omg_est = htz.field_speed_est - htz.slip_est;
        }


        // TODO My proposed saturation time based correction method NOTE VERY COOL
        #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
            htz.u_off_original_lpf_input[ind]         = 0.5*(htz.psi_2_min[ind] + htz.psi_2_max[ind]) /  (htz.Delta_t+htz.Delta_t_last); \
            htz.u_off_calculated_increment[ind]       = 0.5*(htz.psi_2_min[ind] + htz.psi_2_max[ind]) / ((htz.Delta_t+htz.Delta_t_last) - (htz.sat_max_time[ind]+htz.sat_min_time[ind])); \
            htz.u_off_saturation_time_correction[ind] = htz.sat_max_time[ind] - htz.sat_min_time[ind]; \
            htz.u_off_direct_calculated[ind] += (htz.count_negative+htz.count_positive>4) * htz.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
            // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

        int ind;
        for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

            // if( tmin!=0 && tmax!=0 ){
            //     // The sat func's limit is too small.
            //     limit += TS * min(tmin, tmax);
            // }


            /* 必须先检查是否进入levelA */
            if(htz.flag_pos2negLevelA[ind] == TRUE){ 
                if(htz.psi_2_prev[ind]<0 && htz.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                    if(htz.flag_pos2negLevelB[ind] == FALSE){
                        htz.count_negative+=1;
                        // printf("POS2NEG: %g, %d\n", CTRL.timebase, ind);
                        // printf("%g, %g\n", htz.psi_2_prev[ind], htz.psi_2[ind]);
                        // getch();
                        // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                        htz.psi_1_max[ind] = htz.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                        htz.Delta_t_last = htz.Delta_t;
                        htz.Delta_t = htz.time_pos2neg[ind] - htz.time_pos2neg_prev[ind];
                        htz.time_pos2neg_prev[ind] = htz.time_pos2neg[ind]; // 备份作为下次耗时参考点
                        // 初始化
                        htz.flag_neg2posLevelA[ind] = FALSE;
                        htz.flag_neg2posLevelB[ind] = FALSE;

                        CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS

                        htz.psi_1_min[ind] = 0.0;
                        htz.psi_2_min[ind] = 0.0;
                        if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                            htz.sat_min_time_reg[ind] = htz.sat_min_time[ind];
                            if(htz.sat_max_time_reg[ind]>CL_TS && htz.sat_min_time_reg[ind]>CL_TS){
                                htz.flag_limit_too_low = TRUE;
                                htz.extra_limit += 1e-2 * (htz.sat_max_time_reg[ind] + htz.sat_min_time_reg[ind]) / htz.Delta_t; 
                            }else{
                                htz.flag_limit_too_low = FALSE;
                                htz.extra_limit -= 2e-4 * htz.Delta_t;
                                if(htz.extra_limit<0.0){
                                    htz.extra_limit = 0.0;
                                }
                            }
                            htz.sat_max_time_reg[ind] = 0.0;
                        }
                        htz.sat_min_time[ind] = 0.0;
                    }
                    htz.flag_pos2negLevelB[ind] = TRUE; 
                    if(htz.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                        if(htz.psi_2[ind] < htz.psi_2_min[ind]){
                            htz.psi_2_min[ind] = htz.psi_2[ind];
                        }
                    }
                }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，htz.psi_2[0]>0
                    htz.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
                }
            }
            if(htz.psi_2_prev[ind]>0 && htz.psi_2[ind]<0){ // 发现磁链由正变负的时刻
                htz.flag_pos2negLevelA[ind] = TRUE;
                htz.time_pos2neg[ind] = CTRL.timebase;
            }


            if(htz.flag_neg2posLevelA[ind] == TRUE){ 
                if(htz.psi_2_prev[ind]>0 && htz.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
                    if(htz.flag_neg2posLevelB[ind] == FALSE){
                        htz.count_positive+=1;
                        // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                        htz.psi_1_min[ind] = htz.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                        htz.Delta_t_last = htz.Delta_t;
                        htz.Delta_t = htz.time_neg2pos[ind] - htz.time_neg2pos_prev[ind];
                        htz.time_neg2pos_prev[ind] = htz.time_neg2pos[ind]; // 备份作为下次耗时参考点
                        // 初始化
                        htz.flag_pos2negLevelA[ind] = FALSE;
                        htz.flag_pos2negLevelB[ind] = FALSE;

                        CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS

                        htz.psi_1_max[ind] = 0.0;
                        htz.psi_2_max[ind] = 0.0;
                        if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                            htz.sat_max_time_reg[ind] = htz.sat_max_time[ind];
                            if(htz.sat_min_time_reg[ind]>CL_TS && htz.sat_max_time_reg[ind]>CL_TS){
                                htz.flag_limit_too_low = TRUE;
                                htz.extra_limit += 1e-2 * (htz.sat_min_time_reg[ind] + htz.sat_max_time_reg[ind]) / htz.Delta_t; 
                            }else{
                                htz.flag_limit_too_low = FALSE;
                                htz.extra_limit -= 2e-4 * htz.Delta_t;
                                if(htz.extra_limit<0.0){
                                    htz.extra_limit = 0.0;
                                }
                            }
                            htz.sat_min_time_reg[ind] = 0.0;
                        }
                        htz.sat_max_time[ind] = 0.0;
                    }
                    htz.flag_neg2posLevelB[ind] = TRUE; 
                    if(htz.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                        if(htz.psi_2[ind] > htz.psi_2_max[ind]){
                            htz.psi_2_max[ind] = htz.psi_2[ind];
                        }
                    }
                }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，htz.psi_2[0]<0
                    htz.flag_neg2posLevelA[ind] = FALSE;
                }
            }
            if(htz.psi_2_prev[ind]<0 && htz.psi_2[ind]>0){ // 发现磁链由负变正的时刻
                htz.flag_neg2posLevelA[ind] = TRUE;
                htz.time_neg2pos[ind] = CTRL.timebase;
            }
        }

        /*这里一共有四种方案，积分两种，LPF两种：
        1. Holtz03原版是用u_off_original_lpf_input过LPF，
        2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
        3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
        4. 我还提出了用饱和时间去做校正的方法*/

        // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
        #if BOOL_USE_METHOD_INTEGRATOR_INPUT
            #define INTEGRATOR_INPUT(X)   htz.u_off_saturation_time_correction[X] // exact offset calculation for compensation
            // #define INTEGRATOR_INPUT(X)   htz.u_off_original_lpf_input[X]
            htz.u_offset[0] += htz.gain_off * CL_TS * INTEGRATOR_INPUT(0);
            htz.u_offset[1] += htz.gain_off * CL_TS * INTEGRATOR_INPUT(1);
        #endif

        // 低通
        #if BOOL_USE_METHOD_LPF_INPUT
            #define LPF_INPUT(X) htz.u_off_direct_calculated[X]
            // #define LPF_INPUT(X) htz.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
            htz.u_offset[0] = _lpf( LPF_INPUT(0), htz.u_offset[0], TAU_OFF_INVERSE);
            htz.u_offset[1] = _lpf( LPF_INPUT(1), htz.u_offset[1], TAU_OFF_INVERSE);
        #endif

        // 差分
        // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
        // htz.u_offset[0] += DIFFERENCE_INPUT(0);
        // htz.u_offset[1] += DIFFERENCE_INPUT(1);

        // 直通
        // htz.u_offset[0] = htz.u_off_direct_calculated[0];
        // htz.u_offset[1] = htz.u_off_direct_calculated[1];

        // htz.psi_1_nonSat[0] = htz.psi_1[0];
        // htz.psi_1_nonSat[1] = htz.psi_1[1];
        // htz.psi_2_nonSat[0] = htz.psi_2[0];
        // htz.psi_2_nonSat[1] = htz.psi_2[1];

        htz.psi_2_prev[0] = htz.psi_2[0];
        htz.psi_2_prev[1] = htz.psi_2[1];
    }
#endif
#if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
#endif

#if AFE_40_JO_CHOI_METHOD
    void init_joChoi(){
    }
#endif


void simulation_test_flux_estimators(){
    MainFE_HuWu_1998();
    Main_the_active_flux_estimator();
    Main_VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
}

void init_FE(){
    init_huwu();
    init_afe();

    init_Holtz2003();
}

