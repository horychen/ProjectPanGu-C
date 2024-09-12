#include "ACMSim.h"
/*****************************/
/* Shared Solver and typedef */
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
    #define OFFSET_VOLTAGE_ALPHA 0//(0.09*-0.01 *((*CTRL).timebase>0.3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  0//(0.09*+0.01 *((*CTRL).timebase>0.3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
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
    void init_FE_huwu(){
        FE.huwu.tau_1_inv = AFE_21_HUWU_TAU_1_INVERSE;
        FE.huwu.KP = AFE_21_HUWU_KP;
        FE.huwu.KI = AFE_21_HUWU_KI;

        FE.huwu.x[0] = MOTOR.KE;
        FE.huwu.x[1] = 0.0;
        FE.huwu.x[2] = MOTOR.KE;

        FE.huwu.active_flux_ampl = MOTOR.KE;
        FE.huwu.cosT = 1.0;
        FE.huwu.sinT = 0.0;

        FE.huwu.limiter_KE = 1.0*MOTOR.KE; /* FE.huwu1998: In order to eliminate dc component at the output, the limiting level should be set at a value equal to the actual flux amplitude. */
    }
    void rhf_HUWU_1998_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2], x[3]: compensation flux (integral part)

        #define xPsi1(X)  x[0+X]

        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0);
        emf[1] = US(1) - MOTOR.R*IS(1);

        /* 求定子磁链幅值的倒数 */
        REAL temp = 0.0; 
        FE.huwu.stator_flux_ampl = sqrt(xPsi1(0)*xPsi1(0) + xPsi1(1)*xPsi1(1));
        if(FE.huwu.stator_flux_ampl!=0){
            temp = 1.0 / FE.huwu.stator_flux_ampl;
        }

    #define HU_WU_ALGORITHM_2
    #ifdef HU_WU_ALGORITHM_2
        /* 计算有功磁链分量 */
        FE.huwu.psi_2[0] = xPsi1(0) - MOTOR.Lq * IS(0);
        FE.huwu.psi_2[1] = xPsi1(1) - MOTOR.Lq * IS(1);
        /* 求有功磁链幅值的倒数 */
        FE.huwu.active_flux_ampl = sqrt(FE.huwu.psi_2[0]*FE.huwu.psi_2[0] + FE.huwu.psi_2[1]*FE.huwu.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(FE.huwu.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/FE.huwu.active_flux_ampl;
        }
        /* The amplitude limiter for active flux */
        if(FE.huwu.active_flux_ampl > FE.huwu.limiter_KE){
            FE.huwu.active_flux_ampl_limited = FE.huwu.limiter_KE;
        }else{
            FE.huwu.active_flux_ampl_limited = FE.huwu.active_flux_ampl;
        }

        if(FALSE){
            /* 根据限幅后的有功磁链去求限幅后的定子磁链 */
            REAL stator_flux_limited[2];
            stator_flux_limited[0] = FE.huwu.psi_2[0]*active_flux_ampl_inv * FE.huwu.active_flux_ampl_limited + MOTOR.Lq*IS(0);
            stator_flux_limited[1] = FE.huwu.psi_2[1]*active_flux_ampl_inv * FE.huwu.active_flux_ampl_limited + MOTOR.Lq*IS(1);            
            /* 求限幅后的定子磁链幅值 */
            FE.huwu.stator_flux_ampl_limited = sqrt(stator_flux_limited[0]*stator_flux_limited[0] + stator_flux_limited[1]*stator_flux_limited[1]);

            /* Correct by stator flux */
            fx[0] = emf[0] + FE.huwu.tau_1_inv * (( FE.huwu.stator_flux_ampl_limited ) * xPsi1(0) * temp - xPsi1(0) );
            fx[1] = emf[1] + FE.huwu.tau_1_inv * (( FE.huwu.stator_flux_ampl_limited ) * xPsi1(1) * temp - xPsi1(1) );
            fx[2] = 0.0;
        }else{
            /* Simplified (Correct by rotor flux) */
            fx[0] = emf[0] + FE.huwu.tau_1_inv * (( FE.huwu.active_flux_ampl_limited ) * FE.huwu.psi_2[0] * active_flux_ampl_inv - FE.huwu.psi_2[0] );
            fx[1] = emf[1] + FE.huwu.tau_1_inv * (( FE.huwu.active_flux_ampl_limited ) * FE.huwu.psi_2[1] * active_flux_ampl_inv - FE.huwu.psi_2[1] );
            fx[2] = 0.0;
        }

    #else
        #define xPsi_Comp x[2]
        /* Algorithm 3 */
        FE.huwu.inner_product_normalized = (emf[0] * xPsi1(0) + emf[1] * xPsi1(1)) * temp;

        /* Compensation term is a flux amplitude going through an inverse Park transformation */
        fx[0] = emf[0] + FE.huwu.tau_1_inv * ( (/*P*/+ FE.huwu.KP * FE.huwu.inner_product_normalized \
                                             /*I*/+ xPsi_Comp \
                                            ) * xPsi1(0) * temp - xPsi1(0) );
        fx[1] = emf[1] + FE.huwu.tau_1_inv * ( (/*P*/+ FE.huwu.KP * FE.huwu.inner_product_normalized \
                                             /*I*/+ xPsi_Comp \
                                            ) * xPsi1(1) * temp - xPsi1(1) );

        /* State: disturbance/offset estimated by an integrator */
        fx[2] = FE.huwu.KI * FE.huwu.inner_product_normalized;

        #undef xPsi_Comp
    #endif
    #undef xPsi1
    }
    void MainFE_HUWU_1998(){
        // stator flux and integral states update
        general_3states_rk4_solver(&rhf_HUWU_1998_Dynamics, (*CTRL).timebase, FE.huwu.x, CL_TS);
        // Unpack x
        FE.huwu.psi_1[0] = FE.huwu.x[0];
        FE.huwu.psi_1[1] = FE.huwu.x[1];
        FE.huwu.psi_comp = FE.huwu.x[2];

        // active flux and position
        FE.huwu.psi_2[0] = FE.huwu.x[0] - (*CTRL).motor->Lq * IS(0);
        FE.huwu.psi_2[1] = FE.huwu.x[1] - (*CTRL).motor->Lq * IS(1);
        FE.huwu.active_flux_ampl = sqrt(FE.huwu.psi_2[0]*FE.huwu.psi_2[0] + FE.huwu.psi_2[1]*FE.huwu.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(FE.huwu.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/FE.huwu.active_flux_ampl;
            FE.huwu.cosT = FE.huwu.psi_2[0] * active_flux_ampl_inv;
            FE.huwu.sinT = FE.huwu.psi_2[1] * active_flux_ampl_inv;
        }else{
            FE.huwu.cosT = 1.0;
            FE.huwu.sinT = 0.0;
        }

        FE.huwu.theta_d = atan2(FE.huwu.psi_2[1], FE.huwu.psi_2[0]);
    }
#endif
#if AFE_22_STOJIC_2015 
#endif
#if AFE_23_SCVM_HARNEFORS_2003 
#endif
#if AFE_24_OE || AFE_25_VM_CM_FUSION // See ParkSul2014Follower2020
    void afe_one_parameter_tuning(REAL omega_est){
        FE.AFEOE.ActiveFlux_KP = 2* omega_est;
        FE.AFEOE.ActiveFlux_KI = omega_est*omega_est;

        #if PC_SIMULATION
        printf("Init AFEOE:\n");
        printf("\tomega_est=%g\n", omega_est);
        printf("\tAFEOE.ActiveFlux_KP=%g\n", FE.AFEOE.ActiveFlux_KP);
        printf("\tAFEOE.ActiveFlux_KI=%g\n", FE.AFEOE.ActiveFlux_KI);
        #endif
    }
    void init_afe(){
        FE.AFEOE.active_flux_ampl     = MOTOR.KE;
        FE.AFEOE.active_flux_ampl_lpf = FE.AFEOE.active_flux_ampl;
        FE.AFEOE.cosT = 1.0;
        FE.AFEOE.sinT = 0.0;

        FE.AFEOE.x[0] = CORRECTION_4_SHARED_FLUX_EST; // TODO：这里假设了初始位置是alpha轴对着d轴的
        FE.AFEOE.x[1] = 0.0;
        FE.AFEOE.ActiveFlux_KP = AFEOE_KP;
        FE.AFEOE.ActiveFlux_KI = AFEOE_KI;
        FE.AFEOE.set_omega_est = AFEOE_OMEGA_ESTIMATOR;
        FE.AFEOE.omega_est = FE.AFEOE.set_omega_est;
        #if AFE_25_VM_CM_FUSION
            // afe_one_parameter_tuning(FE.AFEOE.omega_est);
        #endif

        FE.AFEOE.limiter_KE = 1.15 * MOTOR.KE;
        FE.AFEOE.k_af = 0.0 * 100 * 2 * M_PI;
    }
    void rhf_ActiveFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2], x[3]: integral action compensating offset voltage

        /* Calculate psi_2 from psi_1, Lq and i_ab */
        FE.AFEOE.psi_2[0] = x[0] - MOTOR.Lq * IS(0);
        FE.AFEOE.psi_2[1] = x[1] - MOTOR.Lq * IS(1);

        /* Output: Flux amplitude */
        FE.AFEOE.active_flux_ampl = sqrt(FE.AFEOE.psi_2[0]*FE.AFEOE.psi_2[0] + FE.AFEOE.psi_2[1]*FE.AFEOE.psi_2[1]);

        /* State: Cosine and Sine of the flux angle */
        REAL active_flux_ampl_inv=0.0;
        if(FE.AFEOE.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/FE.AFEOE.active_flux_ampl;
        } 
        REAL cosT = FE.AFEOE.psi_2[0] * active_flux_ampl_inv;
        REAL sinT = FE.AFEOE.psi_2[1] * active_flux_ampl_inv;

        // #define DEBUG_USING_ACTUAL_CM
        #ifdef DEBUG_USING_ACTUAL_CM
            cosT = (*CTRL).s->cosT;
            sinT = (*CTRL).s->sinT;
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
            FE.AFEOE.output_error[0] = IS(0) - current_estimate[0];
            FE.AFEOE.output_error[1] = IS(1) - current_estimate[1];
        #endif

        #if AFE_25_VM_CM_FUSION // note this will potentially overwrite results of AFE_24_OE
            /* Output: Flux mismatch to KActive (CM) */
            FE.AFEOE.output_error[0] = KActive*cosT - FE.AFEOE.psi_2[0];
            FE.AFEOE.output_error[1] = KActive*sinT - FE.AFEOE.psi_2[1];
        #endif

        #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED
            /* The amplitude limiter */
            if(FE.AFEOE.active_flux_ampl > FE.AFEOE.limiter_KE){
                FE.AFEOE.psi_2_limited[0] = FE.AFEOE.limiter_KE * FE.AFEOE.psi_2[0]*active_flux_ampl_inv;
                FE.AFEOE.psi_2_limited[1] = FE.AFEOE.limiter_KE * FE.AFEOE.psi_2[1]*active_flux_ampl_inv;
            }else{
                FE.AFEOE.psi_2_limited[0] = FE.AFEOE.psi_2[0];
                FE.AFEOE.psi_2_limited[1] = FE.AFEOE.psi_2[1];
            }
        #else
            FE.AFEOE.psi_2_limited[0] = FE.AFEOE.psi_2[0];
            FE.AFEOE.psi_2_limited[1] = FE.AFEOE.psi_2[1];
        #endif

        /* State: stator emf as the derivative of staotr flux, psi_1 */
        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA;
        emf[1] = US(1) - MOTOR.R*IS(1) + 1*OFFSET_VOLTAGE_BETA ;
        fx[0] = emf[0] \
            /*k_af*/- FE.AFEOE.k_af*(FE.AFEOE.psi_2[0] - FE.AFEOE.psi_2_limited[0]) \
            /*P*/+ FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[0] \
            /*I*/+ x[2];
        fx[1] = emf[1] \
            /*k_af*/- FE.AFEOE.k_af*(FE.AFEOE.psi_2[1] - FE.AFEOE.psi_2_limited[1]) \
            /*P*/+ FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[1] \
            /*I*/+ x[3];

        /* State: disturbance/offset estimated by an integrator */
        fx[2] = FE.AFEOE.ActiveFlux_KI * FE.AFEOE.output_error[0];
        fx[3] = FE.AFEOE.ActiveFlux_KI * FE.AFEOE.output_error[1];
    }
    REAL k_af_speed_Hz = 1.5;
    void Main_the_active_flux_estimator(){

        /* OPT for AFE */
        if(FE.AFEOE.omega_est != FE.AFEOE.set_omega_est){
            FE.AFEOE.omega_est = FE.AFEOE.set_omega_est;
            afe_one_parameter_tuning(FE.AFEOE.omega_est);
        }

        /* Time-varying gains */
        if(fabs((*CTRL).i->cmd_varTheta)*MOTOR.npp_inv*ONE_OVER_2PI<k_af_speed_Hz){ // [Hz]
            FE.AFEOE.k_af = 2*M_PI*100;
            FE.AFEOE.limiter_Flag = TRUE;
        }else{
            FE.AFEOE.k_af = 0.0;
            FE.AFEOE.limiter_Flag = FALSE;
        }

        #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED == FALSE
            /* NO Need */
            /* NO Need */
            /* NO Need */
            FE.AFEOE.k_af = 0.0;
        #endif

        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_ActiveFluxEstimator_Dynamics, (*CTRL).timebase, FE.AFEOE.x, CL_TS);
        // Unpack x
        FE.AFEOE.psi_1[0]    = FE.AFEOE.x[0];
        FE.AFEOE.psi_1[1]    = FE.AFEOE.x[1];
        FE.AFEOE.u_offset[0] = FE.AFEOE.x[2];
        FE.AFEOE.u_offset[1] = FE.AFEOE.x[3];

        // active flux
        FE.AFEOE.psi_2[0] = FE.AFEOE.x[0] - (*CTRL).motor->Lq * IS(0);
        FE.AFEOE.psi_2[1] = FE.AFEOE.x[1] - (*CTRL).motor->Lq * IS(1);
        FE.AFEOE.active_flux_ampl = sqrt(FE.AFEOE.psi_2[0]*FE.AFEOE.psi_2[0] + FE.AFEOE.psi_2[1]*FE.AFEOE.psi_2[1]);
        REAL active_flux_ampl_inv=0.0;
        if(FE.AFEOE.active_flux_ampl!=0){
            active_flux_ampl_inv = 1.0/FE.AFEOE.active_flux_ampl;
            FE.AFEOE.cosT = FE.AFEOE.psi_2[0] * active_flux_ampl_inv;
            FE.AFEOE.sinT = FE.AFEOE.psi_2[1] * active_flux_ampl_inv;
        }else{
            FE.AFEOE.cosT = 1.0;
            FE.AFEOE.sinT = 0.0;
        }

        FE.AFEOE.theta_d = atan2(FE.AFEOE.psi_2[1], FE.AFEOE.psi_2[0]);
        FE.AFEOE.theta_e = angle_diff(FE.AFEOE.theta_d, CTRL_1.i->theta_d_elec);
        // Convert output error to dq frame
        // REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
        /* TODO: 思考这个dq角度怎么选最好，要不要换成电压向量的角度而不是转子角度？ */
        FE.AFEOE.active_flux_ampl_lpf = _lpf(FE.AFEOE.active_flux_ampl, FE.AFEOE.active_flux_ampl_lpf, 5);
        FE.AFEOE.output_error_dq[0] = MOTOR.KActive - FE.AFEOE.active_flux_ampl_lpf;
        FE.AFEOE.output_error[0] = MOTOR.KActive*FE.AFEOE.cosT - FE.AFEOE.psi_2[0];
        FE.AFEOE.output_error[1] = MOTOR.KActive*FE.AFEOE.sinT - FE.AFEOE.psi_2[1];
        FE.AFEOE.output_error_dq[1] = sqrt(FE.AFEOE.output_error[0]*FE.AFEOE.output_error[0] + FE.AFEOE.output_error[1]*FE.AFEOE.output_error[1]);
        // if((*CTRL).i->cmd_speed_rpm>0){
        //     FE.AFEOE.output_error_dq[0] = AB2M(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
        //     FE.AFEOE.output_error_dq[1] = AB2T(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
        // }else{
        //     FE.AFEOE.output_error_dq[0] = AB2M(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
        //     FE.AFEOE.output_error_dq[1] = AB2T(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
        // }

        // Research
        REAL emf[2];
        emf[0] = US(0) - MOTOR.R*IS(0) + FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[0];
        emf[1] = US(1) - MOTOR.R*IS(1) + FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[1];
        FE.AFEOE.dot_product = emf[0]*FE.AFEOE.psi_1[0] + emf[1]*FE.AFEOE.psi_1[1];
        FE.AFEOE.cross_product = emf[0]*-FE.AFEOE.psi_1[1] + emf[1]*FE.AFEOE.psi_1[0];
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
    // void init_FE_htz(){

    //     FE.htz.rs_est   = U_MOTOR_R;
    //     FE.htz.rreq_est = U_MOTOR_RREQ;
    //     // FE.htz.psi_aster_max = U_MOTOR_KE + IM_FLUX_COMMAND_SINE_PART;
    //     FE.htz.psi_aster_max = CORRECTION_4_SHARED_FLUX_EST;

    //     int ind;
    //     for(ind=0;ind<2;++ind){
    //         FE.htz.emf_stator[ind] = 0;

    //         FE.htz.psi_1[ind] = 0;
    //         FE.htz.psi_2[ind] = 0;
    //         FE.htz.psi_2_prev[ind] = 0;
    //         FE.htz.psi_2_ampl = CORRECTION_4_SHARED_FLUX_EST;
    //         FE.htz.psi_2_ampl_lpf = FE.htz.psi_2_ampl;

    //         FE.htz.psi_1_nonSat[ind] = 0;
    //         FE.htz.psi_2_nonSat[ind] = 0;

    //         FE.htz.psi_1_min[ind] = 0;
    //         FE.htz.psi_1_max[ind] = 0;
    //         FE.htz.psi_2_min[ind] = 0;
    //         FE.htz.psi_2_max[ind] = 0;

    //         FE.htz.Delta_t = 1;
    //         FE.htz.u_offset[ind] = 0;
    //         FE.htz.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
    //         FE.htz.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
    //         FE.htz.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
    //         FE.htz.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

    //         FE.htz.flag_pos2negLevelA[ind] = 0;
    //         FE.htz.flag_pos2negLevelB[ind] = 0;
    //         FE.htz.time_pos2neg[ind] = 0;
    //         FE.htz.time_pos2neg_prev[ind] = 0;

    //         FE.htz.flag_neg2posLevelA[ind] = 0;
    //         FE.htz.flag_neg2posLevelB[ind] = 0;
    //         FE.htz.time_neg2pos[ind] = 0;
    //         FE.htz.time_neg2pos_prev[ind] = 0;    

    //         FE.htz.sat_min_time[ind] = 0.0;
    //         FE.htz.sat_max_time[ind] = 0.0;
    //         FE.htz.sat_min_time_reg[ind] = 0.0;
    //         FE.htz.sat_max_time_reg[ind] = 0.0;
    //         FE.htz.extra_limit = 0.0;
    //         FE.htz.flag_limit_too_low = FALSE;
    //     }
    // }
    // void rhf_Holtz2003_Dynamics(REAL t, REAL *x, REAL *fx){
    //     FE.htz.emf_stator[0] = US(0) - FE.htz.rs_est*IS(0) - FE.htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
    //     FE.htz.emf_stator[1] = US(1) - FE.htz.rs_est*IS(1) - FE.htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
    //     fx[0] = (FE.htz.emf_stator[0]);
    //     fx[1] = (FE.htz.emf_stator[1]);
    // }
    // void Main_VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit(){
    //     #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
    //     #define PSI_MU_ASTER_MAX FE.htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
    //     #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT FALSE

    //     #define BOOL_USE_METHOD_LPF_INPUT        TRUE
    //     #define BOOL_USE_METHOD_INTEGRATOR_INPUT FALSE

    //     // Euler's method is shit at higher speeds
    //     FE.htz.emf_stator[0] = US_C(0) - FE.htz.rs_est*IS_C(0) - FE.htz.u_offset[0];
    //     FE.htz.emf_stator[1] = US_C(1) - FE.htz.rs_est*IS_C(1) - FE.htz.u_offset[1];
    //     // FE.htz.psi_1[0] += CL_TS*(FE.htz.emf_stator[0]);
    //     // FE.htz.psi_1[1] += CL_TS*(FE.htz.emf_stator[1]);

    //     // rk4 
    //     general_2states_rk4_solver(&rhf_Holtz2003_Dynamics, (*CTRL).timebase, FE.htz.psi_1, CL_TS);
    //     FE.htz.psi_2[0] = FE.htz.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
    //     FE.htz.psi_2[1] = FE.htz.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);
    //     FE.htz.psi_2_ampl = sqrt(FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
    //     FE.htz.psi_2_ampl_lpf = _lpf(FE.htz.psi_2_ampl, FE.htz.psi_2_ampl_lpf, 5);
    //     if(FE.htz.psi_2_ampl!=0){
    //         REAL active_flux_ampl_inv = 0.0;
    //         active_flux_ampl_inv = 1.0/FE.htz.psi_2_ampl;
    //         FE.htz.cosT = FE.htz.psi_2[0] * active_flux_ampl_inv;
    //         FE.htz.sinT = FE.htz.psi_2[1] * active_flux_ampl_inv;
    //     }else{
    //         FE.htz.cosT = 1;
    //         FE.htz.sinT = 0;
    //     }
    //     FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);

    //     FE.htz.psi_1_nonSat[0] += CL_TS*(FE.htz.emf_stator[0]);
    //     FE.htz.psi_1_nonSat[1] += CL_TS*(FE.htz.emf_stator[1]);
    //     FE.htz.psi_2_nonSat[0] = FE.htz.psi_1_nonSat[0] - (*CTRL).motor->Lq*IS_C(0);
    //     FE.htz.psi_2_nonSat[1] = FE.htz.psi_1_nonSat[1] - (*CTRL).motor->Lq*IS_C(1);

    //     if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
    //         // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd + 0.05;
    //         FE.htz.psi_aster_max = (*CTRL).i->cmd_psi + FE.htz.extra_limit;
    //         // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd;            
    //     }

    //     // 限幅是针对转子磁链限幅的
    //     if(FE.htz.psi_2[0]    > PSI_MU_ASTER_MAX){
    //         FE.htz.psi_2[0]   = PSI_MU_ASTER_MAX;
    //         FE.htz.sat_max_time[0] += CL_TS;
    //     }else if(FE.htz.psi_2[0] < -PSI_MU_ASTER_MAX){
    //         FE.htz.psi_2[0]   = -PSI_MU_ASTER_MAX;
    //         FE.htz.sat_min_time[0] += CL_TS;
    //     }
    //     if(FE.htz.psi_2[1]    > PSI_MU_ASTER_MAX){
    //         FE.htz.psi_2[1]   = PSI_MU_ASTER_MAX;
    //         FE.htz.sat_max_time[1] += CL_TS;
    //     }else if(FE.htz.psi_2[1] < -PSI_MU_ASTER_MAX){
    //         FE.htz.psi_2[1]   = -PSI_MU_ASTER_MAX;
    //         FE.htz.sat_min_time[1] += CL_TS;
    //     }
    //     // 限幅后的转子磁链，再求取限幅后的定子磁链
    //     FE.htz.psi_1[0] = FE.htz.psi_2[0] + (*CTRL).motor->Lq*IS_C(0);
    //     FE.htz.psi_1[1] = FE.htz.psi_2[1] + (*CTRL).motor->Lq*IS_C(1);

    //     // Speed Estimation
    //     if(TRUE){
    //         // FE.htz.ireq[0] = (*CTRL).Lmu_inv*FE.htz.psi_2[0] - IS_C(0);
    //         // FE.htz.ireq[1] = (*CTRL).Lmu_inv*FE.htz.psi_2[1] - IS_C(1);
    //         REAL temp;
    //         temp = (FE.htz.psi_1[0]*FE.htz.psi_1[0]+FE.htz.psi_1[1]*FE.htz.psi_1[1]);
    //         if(temp>0.001){
    //             FE.htz.field_speed_est = - (FE.htz.psi_1[0]*-FE.htz.emf_stator[1] + FE.htz.psi_1[1]*FE.htz.emf_stator[0]) / temp;
    //         }
    //         temp = (FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
    //         if(temp>0.001){
    //             FE.htz.slip_est = (*CTRL).motor->Rreq*(IS_C(0)*-FE.htz.psi_2[1]+IS_C(1)*FE.htz.psi_2[0]) / temp;
    //         }
    //         FE.htz.omg_est = FE.htz.field_speed_est - FE.htz.slip_est;
    //     }


    //     // TODO My proposed saturation time based correction method NOTE VERY COOL
    //     #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
    //         FE.htz.u_off_original_lpf_input[ind]         = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) /  (FE.htz.Delta_t+FE.htz.Delta_t_last); \
    //         FE.htz.u_off_calculated_increment[ind]       = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) / ((FE.htz.Delta_t+FE.htz.Delta_t_last) - (FE.htz.sat_max_time[ind]+FE.htz.sat_min_time[ind])); \
    //         FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind]; \
    //         FE.htz.u_off_direct_calculated[ind] += (FE.htz.count_negative+FE.htz.count_positive>4) * FE.htz.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
    //         // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

    //     int ind;
    //     for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

    //         // if( tmin!=0 && tmax!=0 ){
    //         //     // The sat func's limit is too small.
    //         //     limit += TS * min(tmin, tmax);
    //         // }


    //         /* 必须先检查是否进入levelA */
    //         if(FE.htz.flag_pos2negLevelA[ind] == TRUE){ 
    //             if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
    //                 if(FE.htz.flag_pos2negLevelB[ind] == FALSE){
    //                     FE.htz.count_negative+=1;
    //                     // printf("POS2NEG: %g, %d\n", (*CTRL).timebase, ind);
    //                     // printf("%g, %g\n", FE.htz.psi_2_prev[ind], FE.htz.psi_2[ind]);
    //                     // getch();
    //                     // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
    //                     FE.htz.psi_1_max[ind] = FE.htz.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
    //                     FE.htz.Delta_t_last = FE.htz.Delta_t;
    //                     FE.htz.Delta_t = FE.htz.time_pos2neg[ind] - FE.htz.time_pos2neg_prev[ind];
    //                     FE.htz.time_pos2neg_prev[ind] = FE.htz.time_pos2neg[ind]; // 备份作为下次耗时参考点
    //                     // 初始化
    //                     FE.htz.flag_neg2posLevelA[ind] = FALSE;
    //                     FE.htz.flag_neg2posLevelB[ind] = FALSE;

    //                     CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS

    //                     FE.htz.psi_1_min[ind] = 0.0;
    //                     FE.htz.psi_2_min[ind] = 0.0;
    //                     if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
    //                         FE.htz.sat_min_time_reg[ind] = FE.htz.sat_min_time[ind];
    //                         if(FE.htz.sat_max_time_reg[ind]>CL_TS && FE.htz.sat_min_time_reg[ind]>CL_TS){
    //                             FE.htz.flag_limit_too_low = TRUE;
    //                             FE.htz.extra_limit += 1e-2 * (FE.htz.sat_max_time_reg[ind] + FE.htz.sat_min_time_reg[ind]) / FE.htz.Delta_t; 
    //                         }else{
    //                             FE.htz.flag_limit_too_low = FALSE;
    //                             FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
    //                             if(FE.htz.extra_limit<0.0){
    //                                 FE.htz.extra_limit = 0.0;
    //                             }
    //                         }
    //                         FE.htz.sat_max_time_reg[ind] = 0.0;
    //                     }
    //                     FE.htz.sat_min_time[ind] = 0.0;
    //                 }
    //                 FE.htz.flag_pos2negLevelB[ind] = TRUE; 
    //                 if(FE.htz.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
    //                     if(FE.htz.psi_2[ind] < FE.htz.psi_2_min[ind]){
    //                         FE.htz.psi_2_min[ind] = FE.htz.psi_2[ind];
    //                     }
    //                 }
    //             }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]>0
    //                 FE.htz.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
    //             }
    //         }
    //         if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]<0){ // 发现磁链由正变负的时刻
    //             FE.htz.flag_pos2negLevelA[ind] = TRUE;
    //             FE.htz.time_pos2neg[ind] = (*CTRL).timebase;
    //         }


    //         if(FE.htz.flag_neg2posLevelA[ind] == TRUE){ 
    //             if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
    //                 if(FE.htz.flag_neg2posLevelB[ind] == FALSE){
    //                     FE.htz.count_positive+=1;
    //                     // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
    //                     FE.htz.psi_1_min[ind] = FE.htz.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
    //                     FE.htz.Delta_t_last = FE.htz.Delta_t;
    //                     FE.htz.Delta_t = FE.htz.time_neg2pos[ind] - FE.htz.time_neg2pos_prev[ind];
    //                     FE.htz.time_neg2pos_prev[ind] = FE.htz.time_neg2pos[ind]; // 备份作为下次耗时参考点
    //                     // 初始化
    //                     FE.htz.flag_pos2negLevelA[ind] = FALSE;
    //                     FE.htz.flag_pos2negLevelB[ind] = FALSE;

    //                     CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS

    //                     FE.htz.psi_1_max[ind] = 0.0;
    //                     FE.htz.psi_2_max[ind] = 0.0;
    //                     if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
    //                         FE.htz.sat_max_time_reg[ind] = FE.htz.sat_max_time[ind];
    //                         if(FE.htz.sat_min_time_reg[ind]>CL_TS && FE.htz.sat_max_time_reg[ind]>CL_TS){
    //                             FE.htz.flag_limit_too_low = TRUE;
    //                             FE.htz.extra_limit += 1e-2 * (FE.htz.sat_min_time_reg[ind] + FE.htz.sat_max_time_reg[ind]) / FE.htz.Delta_t; 
    //                         }else{
    //                             FE.htz.flag_limit_too_low = FALSE;
    //                             FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
    //                             if(FE.htz.extra_limit<0.0){
    //                                 FE.htz.extra_limit = 0.0;
    //                             }
    //                         }
    //                         FE.htz.sat_min_time_reg[ind] = 0.0;
    //                     }
    //                     FE.htz.sat_max_time[ind] = 0.0;
    //                 }
    //                 FE.htz.flag_neg2posLevelB[ind] = TRUE; 
    //                 if(FE.htz.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
    //                     if(FE.htz.psi_2[ind] > FE.htz.psi_2_max[ind]){
    //                         FE.htz.psi_2_max[ind] = FE.htz.psi_2[ind];
    //                     }
    //                 }
    //             }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]<0
    //                 FE.htz.flag_neg2posLevelA[ind] = FALSE;
    //             }
    //         }
    //         if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]>0){ // 发现磁链由负变正的时刻
    //             FE.htz.flag_neg2posLevelA[ind] = TRUE;
    //             FE.htz.time_neg2pos[ind] = (*CTRL).timebase;
    //         }
    //     }

    //     /*这里一共有四种方案，积分两种，LPF两种：
    //     1. Holtz03原版是用u_off_original_lpf_input过LPF，
    //     2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
    //     3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
    //     4. 我还提出了用饱和时间去做校正的方法*/

    //     // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
    //     #if BOOL_USE_METHOD_INTEGRATOR_INPUT
    //         #define INTEGRATOR_INPUT(X)   FE.htz.u_off_saturation_time_correction[X] // exact offset calculation for compensation
    //         // #define INTEGRATOR_INPUT(X)   FE.htz.u_off_original_lpf_input[X]
    //         FE.htz.u_offset[0] += FE.htz.gain_off * CL_TS * INTEGRATOR_INPUT(0);
    //         FE.htz.u_offset[1] += FE.htz.gain_off * CL_TS * INTEGRATOR_INPUT(1);
    //     #endif

    //     // 低通
    //     #if BOOL_USE_METHOD_LPF_INPUT
    //         #define LPF_INPUT(X) FE.htz.u_off_direct_calculated[X]
    //         // #define LPF_INPUT(X) FE.htz.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
    //         FE.htz.u_offset[0] = _lpf( LPF_INPUT(0), FE.htz.u_offset[0], TAU_OFF_INVERSE);
    //         FE.htz.u_offset[1] = _lpf( LPF_INPUT(1), FE.htz.u_offset[1], TAU_OFF_INVERSE);
    //     #endif

    //     // 差分
    //     // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
    //     // FE.htz.u_offset[0] += DIFFERENCE_INPUT(0);
    //     // FE.htz.u_offset[1] += DIFFERENCE_INPUT(1);

    //     // 直通
    //     // FE.htz.u_offset[0] = FE.htz.u_off_direct_calculated[0];
    //     // FE.htz.u_offset[1] = FE.htz.u_off_direct_calculated[1];

    //     // FE.htz.psi_1_nonSat[0] = FE.htz.psi_1[0];
    //     // FE.htz.psi_1_nonSat[1] = FE.htz.psi_1[1];
    //     // FE.htz.psi_2_nonSat[0] = FE.htz.psi_2[0];
    //     // FE.htz.psi_2_nonSat[1] = FE.htz.psi_2[1];

    //     FE.htz.psi_2_prev[0] = FE.htz.psi_2[0];
    //     FE.htz.psi_2_prev[1] = FE.htz.psi_2[1];
    // }
#endif
#if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
#endif
#if AFE_37_NO_SATURATION_BASED
void init_No_Saturation_Based(){
    int ind;
    for(ind=0;ind<2;++ind){
        FE.no_sat.emf_stator[ind] = 0;

        FE.no_sat.psi_1[0] = d_sim.init.KE;
        FE.no_sat.psi_1[1] = 0;
        FE.no_sat.psi_2[0] = d_sim.init.KE;
        FE.no_sat.psi_2[1] = 0;
        FE.no_sat.psi_2_prev[ind] = 0;
        FE.no_sat.psi_com[ind] = 0; 
        FE.no_sat.psi_2_min[ind] = 0;
        FE.no_sat.psi_2_max[ind] = 0;

        FE.no_sat.rs_est = d_sim.init.R;
        FE.no_sat.u_offset[ind] = 0;
        // FE.no_sat.u_offset_intermediate[ind] = 0;
        FE.no_sat.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
        FE.no_sat.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
        FE.no_sat.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
        FE.no_sat.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

        FE.no_sat.flag_pos2negLevelA[ind] = 0;
        FE.no_sat.flag_pos2negLevelB[ind] = 0;
        FE.no_sat.time_pos2neg[ind] = 0;
        FE.no_sat.time_pos2neg_prev[ind] = 0;

        FE.no_sat.flag_neg2posLevelA[ind] = 0;
        FE.no_sat.flag_neg2posLevelB[ind] = 0;
        FE.no_sat.time_neg2pos[ind] = 0;
        FE.no_sat.time_neg2pos_prev[ind] = 0;
        FE.no_sat.u_offset[ind] = 0;

        FE.no_sat.extra_limit = 0.0;
        FE.no_sat.flag_limit_too_low = FALSE;
        FE.no_sat.theta_e = 0;
    }
    FE.no_sat.ell_1 = d_sim.init.KE;
    FE.no_sat.ell_2 = d_sim.init.KE;
    FE.no_sat.psi_1_min[0] = -d_sim.init.KE;
    FE.no_sat.psi_1_min[1] = -d_sim.init.KE;
    FE.no_sat.psi_1_max[0] = d_sim.init.KE;
    FE.no_sat.psi_1_max[1] = d_sim.init.KE;

}

void rhf_No_Saturation_Based_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL emf[2];
        emf[0] = US(0) - FE.no_sat.rs_est*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/- VM_NOSAT_PI_CORRECTION_GAIN_P * FE.no_sat.psi_com[0] \
            /*I*/- x[2];
        emf[1] = US(1) - FE.no_sat.rs_est*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/- VM_NOSAT_PI_CORRECTION_GAIN_P * FE.no_sat.psi_com[1] \
            /*I*/- x[3];
        FE.no_sat.u_offset[0] = x[2];
        FE.no_sat.u_offset[1] = x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_NOSAT_PI_CORRECTION_GAIN_I * FE.no_sat.psi_com[0];
        fx[3] = VM_NOSAT_PI_CORRECTION_GAIN_I * FE.no_sat.psi_com[1];
}

void Main_No_Saturation_Based(){
    // rk4 
    general_4states_rk4_solver(&rhf_No_Saturation_Based_Dynamics, (*CTRL).timebase, FE.no_sat.psi_1, CL_TS);
    FE.no_sat.psi_2[0] = FE.no_sat.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
    FE.no_sat.psi_2[1] = FE.no_sat.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);
    int ind;
    //psi1 
    for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 
        /* 必须先检查是否进入levelA */
        if(FE.no_sat.flag_pos2negLevelA[ind] == TRUE){ 
            if(FE.no_sat.psi_1_prev[ind]<0 && FE.no_sat.psi_1[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                if(FE.no_sat.flag_pos2negLevelB[ind] == FALSE){
                    FE.no_sat.count_negative_cycle+=1; // FE.no_sat.count_positive_cycle = 0;
                    // printf("POS2NEG: %g, %d\n", (*CTRL).timebase, ind);
                    // printf("%g, %g\n", FE.no_sat.psi_2_prev[ind], FE.no_sat.psi_2[ind]);
                    // getch();
                    // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                    FE.no_sat.psi_2_max[ind] = FE.no_sat.psi_1_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    // 初始化
                    FE.no_sat.flag_neg2posLevelA[ind] = FALSE;
                    FE.no_sat.flag_neg2posLevelB[ind] = FALSE;

                    // 注意这里是正半周到负半周切换的时候才执行一次的哦！
                    // FE.no_sat.accumulated__u_off_saturation_time_correction[ind] += FE.no_sat.u_off_saturation_time_correction[ind];
                    // 饱和时间的正弦包络线的正负半周的频率比磁链频率低多啦！需要再额外加一个低频u_offset校正
                    FE.no_sat.psi_com[ind]= 0.5 * (FE.no_sat.psi_1_max[ind] + FE.no_sat.psi_1_min[ind]);
                    FE.no_sat.ell_1 = (FE.no_sat.psi_2_max[ind] - FE.no_sat.psi_2_min[ind]) * 0.5;
                    FE.no_sat.psi_1_min[ind] = 0.0;
                    FE.no_sat.psi_2_min[ind] = 0.0;

                }
                FE.no_sat.flag_pos2negLevelB[ind] = TRUE;
                if(FE.no_sat.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                    if(FE.no_sat.psi_1[ind] < FE.no_sat.psi_1_min[ind]){
                        FE.no_sat.psi_1_min[ind] = FE.no_sat.psi_1[ind];
                    }
                }
            }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.no_sat.psi_2[0]>0
                FE.no_sat.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
            }
        }
        if(FE.no_sat.psi_1_prev[ind]>0 && FE.no_sat.psi_1[ind]<0){ // 发现磁链由正变负的时刻
            FE.no_sat.flag_pos2negLevelA[ind] = TRUE;
        }


        if(FE.no_sat.flag_neg2posLevelA[ind] == TRUE){ 
            if(FE.no_sat.psi_1_prev[ind]>0 && FE.no_sat.psi_1[ind]>0){ // 二次检查，磁链已经是正的了
                if(FE.no_sat.flag_neg2posLevelB[ind] == FALSE){
                    FE.no_sat.count_positive_cycle+=1; // FE.no_sat.count_negative_cycle = 0;
                    // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                    FE.no_sat.psi_2_min[ind] = FE.no_sat.psi_1_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    // 初始化
                    FE.no_sat.flag_pos2negLevelA[ind] = FALSE;
                    FE.no_sat.flag_pos2negLevelB[ind] = FALSE;

                    // FE.no_sat.accumulated__u_off_saturation_time_correction[ind] += FE.no_sat.u_off_saturation_time_correction[ind];
                    FE.no_sat.psi_com[ind]= 0.5 * (FE.no_sat.psi_1_max[ind] + FE.no_sat.psi_1_min[ind]);
                    FE.no_sat.ell_1 = (FE.no_sat.psi_2_max[ind] - FE.no_sat.psi_2_min[ind]) * 0.5;

                    FE.no_sat.psi_1_max[ind] = 0.0;
                    FE.no_sat.psi_2_max[ind] = 0.0;

                }
                FE.no_sat.flag_neg2posLevelB[ind] = TRUE; 
                if(FE.no_sat.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                    if(FE.no_sat.psi_1[ind] > FE.no_sat.psi_1_max[ind]){
                        FE.no_sat.psi_1_max[ind] = FE.no_sat.psi_1[ind];
                    }
                }
            }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.no_sat.psi_2[0]<0
                FE.no_sat.flag_neg2posLevelA[ind] = FALSE;
            }
        }
        if(FE.no_sat.psi_1_prev[ind]<0 && FE.no_sat.psi_1[ind]>0){ // 发现磁链由负变正的时刻
            FE.no_sat.flag_neg2posLevelA[ind] = TRUE;
        }
    }   
    //psi2
    

    // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
        // FE.no_sat.u_offset[0] += VM_PROPOSED_PI_CORRECTION_GAIN_I * CL_TS * FE.no_sat.psi_com[0];
        // FE.no_sat.u_offset[1] += VM_PROPOSED_PI_CORRECTION_GAIN_I * CL_TS * FE.no_sat.psi_com[0];
    FE.no_sat.psi_1_prev[0] = FE.no_sat.psi_1[0];
    FE.no_sat.psi_1_prev[1] = FE.no_sat.psi_1[1];
    FE.no_sat.psi_2_prev[0] = FE.no_sat.psi_2[0];
    FE.no_sat.psi_2_prev[1] = FE.no_sat.psi_2[1];
    FE.no_sat.psi_2_ampl = sqrtf(FE.no_sat.psi_2[0]*FE.no_sat.psi_2[0]+FE.no_sat.psi_2[1]*FE.no_sat.psi_2[1]);
    REAL psi_2_ampl_inv = 1 / FE.no_sat.psi_2_ampl;
    FE.no_sat.cosT = FE.no_sat.psi_2[0] * psi_2_ampl_inv;
    FE.no_sat.sinT = FE.no_sat.psi_2[1] * psi_2_ampl_inv; 
    FE.no_sat.theta_d = atan2(FE.no_sat.psi_2[1], FE.no_sat.psi_2[0]);
    //FE.no_sat.theta_e = angle_diff(FE.no_sat.theta_d, ACM.theta_d) * ONE_OVER_2PI * 360;
}

#endif
#if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
    void init_ClosedLoopFluxEstimatorForPMSM() {
    int ind;
    // 初始化 psi_1 和 psi_2 数组的所有元素
    for (ind=0;ind<2;++ind) {
        FE.clfe4PMSM.psi_1[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.clfe4PMSM.psi_2[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.clfe4PMSM.correction_integral_term[ind] = 0;
        FE.clfe4PMSM.u_offset[ind] = 0;
        FE.clfe4PMSM.current_error[ind] = 0.0;
        FE.clfe4PMSM.current_estimate[ind] = 0.0;
        FE.clfe4PMSM.hat_psi_d2 = 0.0;
    }
    // 初始化 psi_2_ampl
    FE.clfe4PMSM.psi_2_ampl = 0;

    // 初始化 x 数组的所有元素
    FE.clfe4PMSM.x[0] = d_sim.init.KE;
    FE.clfe4PMSM.x[1] = 0;
    FE.clfe4PMSM.x[2] = 0;
    FE.clfe4PMSM.x[3] = 0;
    FE.clfe4PMSM.theta_e = 0;
    FE.clfe4PMSM.theta_d =0;
    }
/* 8. C. Output Error Closed-loop Flux Estimator */
    void rhf_ClosedLoopFluxEstimatorForPMSM_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2]: \psi_{d\mu}
        // x[3], x[4]: integral action compensating offset voltage

        FE.clfe4PMSM.psi_2[0] = x[0] - MOTOR.Lq * IS(0);
        FE.clfe4PMSM.psi_2[1] = x[1] - MOTOR.Lq * IS(1);
        FE.clfe4PMSM.ampl_psi_2 = sqrt(FE.clfe4PMSM.psi_2[0]*FE.clfe4PMSM.psi_2[0] + FE.clfe4PMSM.psi_2[1]*FE.clfe4PMSM.psi_2[1]);
        REAL ampl_psi_2_inv=0.0;
        REAL cos_rho = 1.0;
        
        REAL sin_rho = 0.0;
        if(FE.clfe4PMSM.ampl_psi_2!=0){
            ampl_psi_2_inv = 1.0/FE.clfe4PMSM.ampl_psi_2;
            cos_rho = FE.clfe4PMSM.psi_2[0] * ampl_psi_2_inv;
            sin_rho = FE.clfe4PMSM.psi_2[1] * ampl_psi_2_inv;
        }

        
        /* Variant with command rho */
        // REAL cos_rho = (*CTRL).s->cosT;
        // REAL sin_rho = (*CTRL).s->sinT;
        /* Lascu CLE */
        // REAL psi_ds = x[0]*cos_rho + x[1]*sin_rho;
        // fx[2] = (*CTRL).motor->Rreq*(*CTRL).motor->Lsigma_inv*psi_ds - (*CTRL).motor->alpha*((*CTRL).motor->Lmu+(*CTRL).motor->Lsigma)*(*CTRL).motor->Lsigma_inv*x[2]\
        //       + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
        /* MRAS */
        REAL ids = 0.0;
        ids = IS(0)*cos_rho + IS(1)*sin_rho;

        
        FE.clfe4PMSM.hat_psi_d2 = (MOTOR.Ld - MOTOR.Lq) * ids + (*CTRL).motor->KE;
        // hat_psi_d2 = (*CTRL).motor->KE;
        FE.clfe4PMSM.current_estimate[0] = MOTOR.Lq_inv * (x[0] - FE.clfe4PMSM.hat_psi_d2*cos_rho);
        FE.clfe4PMSM.current_estimate[1] = MOTOR.Lq_inv * (x[1] - FE.clfe4PMSM.hat_psi_d2*sin_rho);

        
        FE.clfe4PMSM.current_error[0] = IS(0) - FE.clfe4PMSM.current_estimate[0];
        FE.clfe4PMSM.current_error[1] = IS(1) - FE.clfe4PMSM.current_estimate[1];

        REAL emf[2] = {0.0, 0.0};
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/+ OUTPUT_ERROR_CLEF4PMSM_GAIN_KP * FE.clfe4PMSM.current_error[0] \
            /*I*/+ x[2];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/+ OUTPUT_ERROR_CLEF4PMSM_GAIN_KP * FE.clfe4PMSM.current_error[1] \
            /*I*/+ x[3];
        
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = OUTPUT_ERROR_CLEF4PMSM_GAIN_KI * FE.clfe4PMSM.current_error[0];
        fx[3] = OUTPUT_ERROR_CLEF4PMSM_GAIN_KI * FE.clfe4PMSM.current_error[1];
    }
    void Main_VM_ClosedLoopFluxEstimatorForPMSM(){
        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_ClosedLoopFluxEstimatorForPMSM_Dynamics, (*CTRL).timebase, FE.clfe4PMSM.x, CL_TS);
        // Unpack x
        FE.clfe4PMSM.psi_1[0]                    = FE.clfe4PMSM.x[0];
        FE.clfe4PMSM.psi_1[1]                    = FE.clfe4PMSM.x[1];
        FE.clfe4PMSM.correction_integral_term[0] = FE.clfe4PMSM.x[2];
        FE.clfe4PMSM.correction_integral_term[1] = FE.clfe4PMSM.x[3];
        FE.clfe4PMSM.u_offset[0] = FE.clfe4PMSM.correction_integral_term[0];
        FE.clfe4PMSM.u_offset[1] = FE.clfe4PMSM.correction_integral_term[1];
        // rotor flux updates
        FE.clfe4PMSM.psi_2[0] = FE.clfe4PMSM.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
        FE.clfe4PMSM.psi_2[1] = FE.clfe4PMSM.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);    
        FE.clfe4PMSM.psi_2_ampl = sqrt(FE.clfe4PMSM.psi_2[0]*FE.clfe4PMSM.psi_2[0]+FE.clfe4PMSM.psi_2[1]*FE.clfe4PMSM.psi_2[1]);
        FE.clfe4PMSM.theta_d = atan2(FE.clfe4PMSM.psi_2[1], FE.clfe4PMSM.psi_2[0]);
        //FE.clfe4PMSM.theta_e = angle_diff(FE.clfe4PMSM.theta_d, ACM.theta_d) * ONE_OVER_2PI * 360;
    }
#endif
#if AFE_39_SATURATION_TIME_WITHOUT_LIMITING
void init_Saturation_time_Without_Limiting(){
    int ind;
    for(ind=0;ind<2;++ind){
        FE.stwl.emf_stator[ind] = 0;

        FE.stwl.psi_1[0] = d_sim.init.KE;
        FE.stwl.psi_1[1] = 0;
        FE.stwl.psi_2[ind] = 0;
        FE.stwl.psi_2_prev[ind] = 0;

        FE.stwl.psi_1_nonSat[ind] = 0;
        FE.stwl.psi_2_nonSat[ind] = 0;

        FE.stwl.psi_1_min[ind] = 0;
        FE.stwl.psi_1_max[ind] = 0;
        FE.stwl.psi_2_min[ind] = 0;
        FE.stwl.psi_2_max[ind] = 0;

        FE.stwl.rs_est = d_sim.init.R;
        FE.stwl.rreq_est = d_sim.init.Rreq;

        FE.stwl.Delta_t = 1;
        FE.stwl.u_offset[ind] = 0;
        // FE.stwl.u_offset_intermediate[ind] = 0;
        FE.stwl.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
        FE.stwl.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
        FE.stwl.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
        FE.stwl.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

        FE.stwl.flag_pos2negLevelA[ind] = 0;
        FE.stwl.flag_pos2negLevelB[ind] = 0;
        FE.stwl.time_pos2neg[ind] = 0;
        FE.stwl.time_pos2neg_prev[ind] = 0;

        FE.stwl.flag_neg2posLevelA[ind] = 0;
        FE.stwl.flag_neg2posLevelB[ind] = 0;
        FE.stwl.time_neg2pos[ind] = 0;
        FE.stwl.time_neg2pos_prev[ind] = 0;

        FE.stwl.psi_aster_max = d_sim.init.KE;

        FE.stwl.maximum_of_sat_min_time[ind] = 0.0;
        FE.stwl.maximum_of_sat_max_time[ind] = 0.0;
        FE.stwl.sat_min_time[ind] = 0.0;
        FE.stwl.sat_max_time[ind] = 0.0;
        FE.stwl.sat_min_time_reg[ind] = 0.0;
        FE.stwl.sat_max_time_reg[ind] = 0.0;
        FE.stwl.extra_limit = 0.0;
        FE.stwl.flag_limit_too_low = FALSE;
    };
}
int bool_positive_extra_limit_2 =TRUE;
REAL stwl_realtime_gain_off =  1.0;
void rhf_Saturation_time_Without_Limiting_Dynamics(REAL t, REAL *x, REAL *fx){
    FE.stwl.emf_stator[0] = US(0) - (*CTRL).motor->R * IS(0) - FE.stwl.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
    FE.stwl.emf_stator[1] = US(1) - (*CTRL).motor->R * IS(1) - FE.stwl.u_offset[1] + OFFSET_VOLTAGE_BETA ;
    fx[0] = (FE.stwl.emf_stator[0]);
    fx[1] = (FE.stwl.emf_stator[1]);
}
void Main_Saturation_time_Without_Limiting(){
    #define PSI_MU_ASTER_MAX FE.htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
    #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT FALSE // The limit is extended when both upper and lower limit are reached in a cycle.

    #define BOOL_USE_METHOD_LPF_INPUT      FALSE
    #define BOOL_USE_METHOD_INTEGRAL_INPUT TRUE

    // Euler's method is shit at higher speeds
    FE.stwl.emf_stator[0] = US_C(0) - (*CTRL).motor->R*IS_C(0) - FE.stwl.u_offset[0];
    FE.stwl.emf_stator[1] = US_C(1) - (*CTRL).motor->R*IS_C(1) - FE.stwl.u_offset[1];
    // FE.stwl.psi_1[0] += CL_TS*(FE.stwl.emf_stator[0]);
    // FE.stwl.psi_1[1] += CL_TS*(FE.stwl.emf_stator[1]);

    // rk4 
    general_2states_rk4_solver(&rhf_Saturation_time_Without_Limiting_Dynamics, (*CTRL).timebase, FE.stwl.psi_1, CL_TS);
    FE.stwl.psi_2[0] = FE.stwl.psi_1[0] - (*CTRL).motor->Lsigma*IS_C(0);
    FE.stwl.psi_2[1] = FE.stwl.psi_1[1] - (*CTRL).motor->Lsigma*IS_C(1);
    FE.stwl.psi_2_ampl = sqrt(FE.stwl.psi_2[0]*FE.stwl.psi_2[0]+FE.stwl.psi_2[1]*FE.stwl.psi_2[1]);

    // 限幅前求角度还是应该限幅后？
    FE.stwl.theta_d = atan2(FE.stwl.psi_2[1], FE.stwl.psi_2[0]);
    FE.stwl.cosT = cos(FE.stwl.theta_d);
    FE.stwl.sinT = sin(FE.stwl.theta_d);

    FE.stwl.psi_1_nonSat[0] += CL_TS*(FE.stwl.emf_stator[0]);
    FE.stwl.psi_1_nonSat[1] += CL_TS*(FE.stwl.emf_stator[1]);
    FE.stwl.psi_2_nonSat[0] = FE.stwl.psi_1_nonSat[0] - (*CTRL).motor->Lsigma*IS_C(0);
    FE.stwl.psi_2_nonSat[1] = FE.stwl.psi_1_nonSat[1] - (*CTRL).motor->Lsigma*IS_C(1);

    // FE.stwl.psi_aster_max = (*CTRL).taao_flux_cmd + 0.05;
    FE.stwl.psi_aster_max = (*CTRL).i->cmd_psi + FE.stwl.extra_limit;
    // FE.stwl.psi_aster_max = (*CTRL).taao_flux_cmd;

    // 限幅是针对转子磁链限幅的
    int ind;
    for(ind=0;ind<2;++ind){
        if((*CTRL).i->cmd_varOmega != 0.0){
            if(FE.stwl.psi_2[ind]    > PSI_MU_ASTER_MAX){ // TODO BUG呀！这里怎么可以是>应该是大于等于啊！
                FE.stwl.psi_2[ind]   = PSI_MU_ASTER_MAX;
                FE.stwl.sat_max_time[ind] += CL_TS;
                // marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                // marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                // marino_sat_d_axis_flux_control = 0.0;
                // marino_sat_q_axis_flux_control = 0.0;
            }else if(FE.stwl.psi_2[ind] < -PSI_MU_ASTER_MAX){
                FE.stwl.psi_2[ind]   = -PSI_MU_ASTER_MAX;
                FE.stwl.sat_min_time[ind] += CL_TS;
            //     marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
            //     marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
            //     marino_sat_d_axis_flux_control = 0.0;
            //     marino_sat_q_axis_flux_control = 0.0;
            }else{
                // 这样可以及时清零饱和时间
                if(FE.stwl.sat_max_time[ind]>0){FE.stwl.sat_max_time[ind] -= CL_TS;}
                if(FE.stwl.sat_min_time[ind]>0){FE.stwl.sat_min_time[ind] -= CL_TS;}
                // marino.lambda_inv = LAMBDA_INV_xOmg;
                // marino.gamma_inv  = GAMMA_INV_xTL;
                // marino_sat_d_axis_flux_control = 1.0;
                // marino_sat_q_axis_flux_control = 1.0;
            }
        }
        // 上限饱和减去下限饱和作为误差，主要为了消除实际磁链幅值大于给定的情况，实际上这种现象在常见工况下出现次数不多。
        FE.stwl.u_off_saturation_time_correction[ind] = FE.stwl.sat_max_time[ind] - FE.stwl.sat_min_time[ind];
        // u_offset波动会导致sat_min_time和sat_max_time的波动，这个时候最有效的办法是减少gain_off。
        // 但是同时，观察饱和时间sat_min_time等的波形，可以发现它里面也会出现一个正弦波包络线。
        if(FE.stwl.sat_min_time[ind] > FE.stwl.maximum_of_sat_min_time[ind]){FE.stwl.maximum_of_sat_min_time[ind] = FE.stwl.sat_min_time[ind];}
        if(FE.stwl.sat_max_time[ind] > FE.stwl.maximum_of_sat_max_time[ind]){FE.stwl.maximum_of_sat_max_time[ind] = FE.stwl.sat_max_time[ind];}
    }

    // 数数，算磁链周期
    if(FE.stwl.psi_2[0]    > 0.0){
        FE.stwl.count_positive_in_one_cycle[0] += 1;
        if(FE.stwl.count_negative_in_one_cycle[0]!=0){ FE.stwl.negative_cycle_in_count[0] = FE.stwl.count_negative_in_one_cycle[0]; FE.stwl.count_negative_in_one_cycle[0] = 0;}
    }else if(FE.stwl.psi_2[0] < -0.0){
        FE.stwl.count_negative_in_one_cycle[0] += 1;
        if(FE.stwl.count_positive_in_one_cycle[0]!=0){ FE.stwl.positive_cycle_in_count[0] = FE.stwl.count_positive_in_one_cycle[0]; FE.stwl.count_positive_in_one_cycle[0] = 0;}
    }
    if(FE.stwl.psi_2[1]    > 0.0){
        FE.stwl.count_positive_in_one_cycle[1] += 1;
        if(FE.stwl.count_negative_in_one_cycle[1]!=0){ FE.stwl.negative_cycle_in_count[1] = FE.stwl.count_negative_in_one_cycle[1]; FE.stwl.count_negative_in_one_cycle[1] = 0;}
    }else if(FE.stwl.psi_2[1] < -0.0){
        FE.stwl.count_negative_in_one_cycle[1] += 1;
        if(FE.stwl.count_positive_in_one_cycle[1]!=0){ FE.stwl.positive_cycle_in_count[1] = FE.stwl.count_positive_in_one_cycle[1]; FE.stwl.count_positive_in_one_cycle[1] = 0;}
    }

    // 限幅后的转子磁链，再求取限幅后的定子磁链
    FE.stwl.psi_1[0] = FE.stwl.psi_2[0] + (*CTRL).motor->Lsigma*IS_C(0);
    FE.stwl.psi_1[1] = FE.stwl.psi_2[1] + (*CTRL).motor->Lsigma*IS_C(1);

    // Speed Estimation
    if(TRUE){
        // FE.stwl.ireq[0] = (*CTRL).Lmu_inv*FE.stwl.psi_2[0] - IS_C(0);
        // FE.stwl.ireq[1] = (*CTRL).Lmu_inv*FE.stwl.psi_2[1] - IS_C(1);
        REAL temp;
        temp = (FE.stwl.psi_1[0]*FE.stwl.psi_1[0]+FE.stwl.psi_1[1]*FE.stwl.psi_1[1]);
        if(temp>0.001){
            FE.stwl.field_speed_est = - (FE.stwl.psi_1[0]*-FE.stwl.emf_stator[1] + FE.stwl.psi_1[1]*FE.stwl.emf_stator[0]) / temp;
        }
        temp = (FE.stwl.psi_2[0]*FE.stwl.psi_2[0]+FE.stwl.psi_2[1]*FE.stwl.psi_2[1]);
        if(temp>0.001){
            FE.stwl.slip_est = (*CTRL).motor->Rreq*(IS_C(0)*-FE.stwl.psi_2[1]+IS_C(1)*FE.stwl.psi_2[0]) / temp;
        }
        FE.stwl.omg_est = FE.stwl.field_speed_est - FE.stwl.slip_est;
    }


    // TODO My proposed saturation time based correction method NOTE VERY COOL
    #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
        FE.stwl.u_off_original_lpf_input[ind]         = 0.5*(FE.stwl.psi_2_min[ind] + FE.stwl.psi_2_max[ind]) /  (FE.stwl.Delta_t+FE.stwl.Delta_t_last); \
        FE.stwl.u_off_calculated_increment[ind]       = 0.5*(FE.stwl.psi_2_min[ind] + FE.stwl.psi_2_max[ind]) / ((FE.stwl.Delta_t+FE.stwl.Delta_t_last) - (FE.stwl.sat_max_time[ind]+FE.stwl.sat_min_time[ind])); \
        FE.stwl.u_off_saturation_time_correction[ind] = FE.stwl.sat_max_time[ind] - FE.stwl.sat_min_time[ind]; \
        FE.stwl.u_off_direct_calculated[ind] += (FE.stwl.count_negative_cycle+FE.stwl.count_positive_cycle>4) * FE.stwl.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
        // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

    for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

        // if( tmin!=0 && tmax!=0 ){
        //     // The sat func's limit is too small.
        //     limit += TS * min(tmin, tmax);
        // }


        /* 必须先检查是否进入levelA */
        if(FE.stwl.flag_pos2negLevelA[ind] == TRUE){ 
            if(FE.stwl.psi_2_prev[ind]<0 && FE.stwl.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                if(FE.stwl.flag_pos2negLevelB[ind] == FALSE){
                    FE.stwl.count_negative_cycle+=1; // FE.stwl.count_positive_cycle = 0;
                    // printf("POS2NEG: %g, %d\n", (*CTRL).timebase, ind);
                    // printf("%g, %g\n", FE.stwl.psi_2_prev[ind], FE.stwl.psi_2[ind]);
                    // getch();
                    // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                    FE.stwl.psi_1_max[ind] = FE.stwl.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.stwl.Delta_t_last = FE.stwl.Delta_t;
                    FE.stwl.Delta_t = FE.stwl.time_pos2neg[ind] - FE.stwl.time_pos2neg_prev[ind];
                    FE.stwl.time_pos2neg_prev[ind] = FE.stwl.time_pos2neg[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.stwl.flag_neg2posLevelA[ind] = FALSE;
                    FE.stwl.flag_neg2posLevelB[ind] = FALSE;

                    // 注意这里是正半周到负半周切换的时候才执行一次的哦！
                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.stwl.accumulated__u_off_saturation_time_correction[ind] += FE.stwl.u_off_saturation_time_correction[ind];
                    FE.stwl.sign__u_off_saturation_time_correction[ind] = -1.0;
                    // 饱和时间的正弦包络线的正负半周的频率比磁链频率低多啦！需要再额外加一个低频u_offset校正
                    FE.stwl.sat_time_offset[ind] = FE.stwl.maximum_of_sat_max_time[ind] - FE.stwl.maximum_of_sat_min_time[ind];
                    FE.stwl.maximum_of_sat_max_time[ind] = 0.0;
                    FE.stwl.maximum_of_sat_min_time[ind] = 0.0;

                    FE.stwl.psi_1_min[ind] = 0.0;
                    FE.stwl.psi_2_min[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.stwl.sat_min_time_reg[ind] = FE.stwl.sat_min_time[ind];
                        if(FE.stwl.sat_max_time_reg[ind]>CL_TS && FE.stwl.sat_min_time_reg[ind]>CL_TS){
                            FE.stwl.flag_limit_too_low = TRUE;
                            FE.stwl.extra_limit += 1e-2 * (FE.stwl.sat_max_time_reg[ind] + FE.stwl.sat_min_time_reg[ind]) / FE.stwl.Delta_t; 
                        }else{
                            FE.stwl.flag_limit_too_low = FALSE;
                            FE.stwl.extra_limit -= 2e-4 * FE.stwl.Delta_t;
                            if(bool_positive_extra_limit_2){
                                if(FE.stwl.extra_limit<0.0){
                                    FE.stwl.extra_limit = 0.0;
                                }
                            }
                        }
                        FE.stwl.sat_max_time_reg[ind] = 0.0;
                    }
                    FE.stwl.sat_min_time[ind] = 0.0;
                }
                FE.stwl.flag_pos2negLevelB[ind] = TRUE;
                if(FE.stwl.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                    if(FE.stwl.psi_2[ind] < FE.stwl.psi_2_min[ind]){
                        FE.stwl.psi_2_min[ind] = FE.stwl.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.stwl.psi_2[0]>0
                FE.stwl.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
            }
        }
        if(FE.stwl.psi_2_prev[ind]>0 && FE.stwl.psi_2[ind]<0){ // 发现磁链由正变负的时刻
            FE.stwl.flag_pos2negLevelA[ind] = TRUE;
            FE.stwl.time_pos2neg[ind] = (*CTRL).timebase;
        }


        if(FE.stwl.flag_neg2posLevelA[ind] == TRUE){ 
            if(FE.stwl.psi_2_prev[ind]>0 && FE.stwl.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
                if(FE.stwl.flag_neg2posLevelB[ind] == FALSE){
                    FE.stwl.count_positive_cycle+=1; // FE.stwl.count_negative_cycle = 0;
                    // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                    FE.stwl.psi_1_min[ind] = FE.stwl.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.stwl.Delta_t_last = FE.stwl.Delta_t;
                    FE.stwl.Delta_t = FE.stwl.time_neg2pos[ind] - FE.stwl.time_neg2pos_prev[ind];
                    FE.stwl.time_neg2pos_prev[ind] = FE.stwl.time_neg2pos[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.stwl.flag_pos2negLevelA[ind] = FALSE;
                    FE.stwl.flag_pos2negLevelB[ind] = FALSE;

                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.stwl.accumulated__u_off_saturation_time_correction[ind] += FE.stwl.u_off_saturation_time_correction[ind];
                    FE.stwl.sign__u_off_saturation_time_correction[ind] = 1.0;

                    FE.stwl.psi_1_max[ind] = 0.0;
                    FE.stwl.psi_2_max[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.stwl.sat_max_time_reg[ind] = FE.stwl.sat_max_time[ind];
                        if(FE.stwl.sat_min_time_reg[ind]>CL_TS && FE.stwl.sat_max_time_reg[ind]>CL_TS){
                            FE.stwl.flag_limit_too_low = TRUE;
                            FE.stwl.extra_limit += 1e-2 * (FE.stwl.sat_min_time_reg[ind] + FE.stwl.sat_max_time_reg[ind]) / FE.stwl.Delta_t; 
                        }else{
                            FE.stwl.flag_limit_too_low = FALSE;
                            FE.stwl.extra_limit -= 2e-4 * FE.stwl.Delta_t;
                            if(FE.stwl.extra_limit<0.0){
                                FE.stwl.extra_limit = 0.0;
                            }
                        }
                        FE.stwl.sat_min_time_reg[ind] = 0.0;
                    }
                    FE.stwl.sat_max_time[ind] = 0.0;
                }
                FE.stwl.flag_neg2posLevelB[ind] = TRUE; 
                if(FE.stwl.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                    if(FE.stwl.psi_2[ind] > FE.stwl.psi_2_max[ind]){
                        FE.stwl.psi_2_max[ind] = FE.stwl.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.stwl.psi_2[0]<0
                FE.stwl.flag_neg2posLevelA[ind] = FALSE;
            }
        }
        if(FE.stwl.psi_2_prev[ind]<0 && FE.stwl.psi_2[ind]>0){ // 发现磁链由负变正的时刻
            FE.stwl.flag_neg2posLevelA[ind] = TRUE;
            FE.stwl.time_neg2pos[ind] = (*CTRL).timebase;
        }
    }

    /*这里一共有四种方案，积分两种，LPF两种：
    1. Holtz03原版是用u_off_original_lpf_input过LPF，
    2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
    3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
    4. 我还提出了用饱和时间去做校正的方法*/

    // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
    #if BOOL_USE_METHOD_INTEGRAL_INPUT
        #define INTEGRAL_INPUT(X)   FE.stwl.u_off_saturation_time_correction[X] // exact offset calculation for compensation
        // FE.stwl.sat_time_offset[X]
        // #define INTEGRAL_INPUT(X)   FE.stwl.accumulated__u_off_saturation_time_correction[X]
        // #define INTEGRAL_INPUT(X)   FE.stwl.u_off_original_lpf_input[X]

        long int local_sum = FE.stwl.negative_cycle_in_count[0] + FE.stwl.positive_cycle_in_count[0] + FE.stwl.negative_cycle_in_count[1] + FE.stwl.positive_cycle_in_count[1];
        if(local_sum>0){
            FE.stwl.gain_off = stwl_realtime_gain_off * HOLTZ_2002_GAIN_OFFSET / ((REAL)local_sum*CL_TS);
        }
        FE.stwl.u_offset[0] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(0);
        FE.stwl.u_offset[1] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(1);

        // 想法是好的，但是没有用，因为咱们要的不是磁链正负半周的饱和时间最大和最小相互抵消；实际上观察到正负半周是饱和时间的正弦包络线的正负半周，频率比磁链频率低多啦！
        // FE.stwl.accumulated__u_off_saturation_time_correction[0] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.stwl.accumulated__u_off_saturation_time_correction[1] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // if(FE.stwl.sign__u_off_saturation_time_correction[0]>0){
        //     FE.stwl.u_offset[0] = FE.stwl.accumulated__u_off_saturation_time_correction[0];
        //     FE.stwl.sign__u_off_saturation_time_correction[0] = 0;
        // }
        // if(FE.stwl.sign__u_off_saturation_time_correction[1]>0){
        //     FE.stwl.u_offset[1] = FE.stwl.accumulated__u_off_saturation_time_correction[1];
        //     FE.stwl.sign__u_off_saturation_time_correction[1] = 0;
        // }

        // 本来想二重积分消除交流波动，但是我发现饱和时间误差波动的原因是上下饱和时间清零不及时导致的。但是哦……好像也没有有效地及时清零的方法，所以还是试试双重积分吧：
        // FE.stwl.u_offset_intermediate[0] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.stwl.u_offset_intermediate[1] += FE.stwl.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // FE.stwl.u_offset[0] += CL_TS * FE.stwl.u_offset_intermediate[0];
        // FE.stwl.u_offset[1] += CL_TS * FE.stwl.u_offset_intermediate[1];
    #endif

    // 低通
    #if BOOL_USE_METHOD_LPF_INPUT
        #define LPF_INPUT(X) FE.stwl.u_off_direct_calculated[X]
        // #define LPF_INPUT(X) FE.stwl.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
        #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
        FE.stwl.u_offset[0] = _lpf( LPF_INPUT(0), FE.stwl.u_offset[0], TAU_OFF_INVERSE);
        FE.stwl.u_offset[1] = _lpf( LPF_INPUT(1), FE.stwl.u_offset[1], TAU_OFF_INVERSE);
    #endif

    // 差分
    // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
    // FE.stwl.u_offset[0] += DIFFERENCE_INPUT(0);
    // FE.stwl.u_offset[1] += DIFFERENCE_INPUT(1);

    // 直通
    // FE.stwl.u_offset[0] = FE.stwl.u_off_direct_calculated[0];
    // FE.stwl.u_offset[1] = FE.stwl.u_off_direct_calculated[1];

    // FE.stwl.psi_1_nonSat[0] = FE.stwl.psi_1[0];
    // FE.stwl.psi_1_nonSat[1] = FE.stwl.psi_1[1];
    // FE.stwl.psi_2_nonSat[0] = FE.stwl.psi_2[0];
    // FE.stwl.psi_2_nonSat[1] = FE.stwl.psi_2[1];
    FE.stwl.theta_d = atan2(FE.stwl.psi_2[1], FE.stwl.psi_2[0]);
    //FE.stwl.theta_e = angle_diff(FE.stwl.theta_d, ACM.theta_d) * ONE_OVER_2PI * 360;
    FE.stwl.psi_2_prev[0] = FE.stwl.psi_2[0];
    FE.stwl.psi_2_prev[1] = FE.stwl.psi_2[1];
};
#endif

#if AFE_40_JO_CHOI_METHOD
    void init_joChoi(){
    }
#endif


void simulation_test_flux_estimators(){
    // MainFE_HUWU_1998();
    // Main_No_Saturation_Based();
    // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
    // Main_VM_ClosedLoopFluxEstimatorForPMSM();
    Main_the_active_flux_estimator();
    // VM_LascuAndreescus2006();
    // Main_Saturation_time_Without_Limiting();
    // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
}

void init_FE(){
    // init_FE_huwu();
    // init_ClosedLoopFluxEstimatorForPMSM();
    init_afe();
    // init_No_Saturation_Based();
    // init_FE_htz();
    // init_LascuAndreescus2006();
    // init_Saturation_time_Without_Limiting();
}
