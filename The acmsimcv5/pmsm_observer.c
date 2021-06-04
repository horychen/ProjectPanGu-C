#include "ACMSim.h"

#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

/********************************************/
/* Natural Speed Observer for IPMSM with Active Flux Concept (Chen 2020)
 ********************************************/
/* The 3rd-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_NSOAF
void rhf_NSOAF_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xIq  = x[0];
    REAL xOmg = x[1];
    REAL xTL  = x[2];

    /* Know Signals */
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS(0), IS(1), AFE_USED.cosT, AFE_USED.sinT);
    iDQ_now[1]  = AB2T(IS(0), IS(1), AFE_USED.cosT, AFE_USED.sinT);
    REAL uQ_now = AB2T(US(0), US(1), AFE_USED.cosT, AFE_USED.sinT);
    nsoaf.q_axis_voltage = uQ_now;

    /* Output Error = \tilde i_q (scalar) */
    nsoaf.output_error = iDQ_now[1] - xIq;
    #ifdef NSOAF_SPMSM

        /* Filter (TODO: 不能把欧拉法放到龙贝格法里面来啊！) */
        static REAL uQ_now_filtered = 0.0;
        uQ_now_filtered = _lpf(uQ_now, uQ_now_filtered, 30); // 越大滤越狠
        // uQ_now_filtered = uQ_now;

        nsoaf.active_power_real  = + fabs(uQ_now_filtered) * iDQ_now[1];
        nsoaf.active_power_est   = + fabs(uQ_now_filtered) * xIq;
        nsoaf.active_power_error = + fabs(uQ_now_filtered) * nsoaf.output_error; 
    #endif
    #ifdef NSOAF_IPMSM
        nsoaf.active_power_real  = + iDQ_now[1];
        nsoaf.active_power_est   = + xIq;
        nsoaf.active_power_error = + nsoaf.output_error; /*妈耶……结果不要把uQ拿进来就完美了……*/
    #endif

    /* State Observer */
    // xIq
    fx[0] = MOTOR.Lq_inv * (uQ_now - MOTOR.R * xIq - xOmg*(MOTOR.KE + MOTOR.Ld*iDQ_now[0])) - nsoaf.KD*nsoaf.active_power_error;
    // xOmg
    REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * KActive * xIq;
    // xTL = ACM.TLoad; // DEBUG
    fx[1] = MOTOR.Js_inv * MOTOR.npp * (nsoaf.xTem - xTL - nsoaf.KP*nsoaf.active_power_error);
    /* Parameter Adaptation */
    // xTL
    fx[2] = nsoaf.KI * nsoaf.active_power_error;
}
void nso_one_parameter_tuning(REAL omega_ob){
    if(omega_ob<170){
        nsoaf.set_omega_ob = 170;
        return;
    }

    #define LQ_INV MOTOR.Lq_inv
    REAL one_over__npp_divided_by_Js__times__Lq_id_plus_KActive = 1.0 \
        / ( MOTOR.npp 
            * MOTOR.Js_inv * LQ_INV
            * (MOTOR.Lq*CTRL.I->idq_cmd[0] + MOTOR.KActive)
          );
    REAL uq_inv = 1.0 / CTRL.O->udq_cmd[1];
    #if TUNING_IGNORE_UQ
        uq_inv = 1.0; // this is 1.#INF when init
    #endif

    // nsoaf.KD = 3*omega_ob                                     * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;
    // nsoaf.KP = ( (3*omega_ob*omega_ob - MOTOR.R*LQ_INV) * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive - 1.5*MOTOR.npp*MOTOR.KActive ) * uq_inv;
    // nsoaf.KI = omega_ob*omega_ob*omega_ob                     * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;

    nsoaf.KD = (  3*omega_ob - MOTOR.R*LQ_INV) * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;
    nsoaf.KP = ( (3*omega_ob*omega_ob)         * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive - 1.5*MOTOR.npp*MOTOR.KActive ) * uq_inv;
    nsoaf.KI = omega_ob*omega_ob*omega_ob      * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;

    #if PC_SIMULATION
    #if TUNING_IGNORE_UQ
    printf("Init NSOAF:\n");
    printf("\tIgnore uq.\n");
    #endif
    printf("\tomega_ob=%g\n", omega_ob);
    printf("\t1.5*MOTOR.npp*MOTOR.KActive=%g\n", 1.5*MOTOR.npp*MOTOR.KActive);
    printf("\tone_over__npp_divided_by_Js__times__Lq_id_plus_KActive=%g\n", one_over__npp_divided_by_Js__times__Lq_id_plus_KActive);
    printf("\tKD=%g\n\tKP=%g\n\tKI=%g\n", nsoaf.KD, nsoaf.KP, nsoaf.KI);
    #endif
}
void Main_nsoaf_chen2020(){

    /* OBSERVATION */

    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * MOTOR.Lq*CTRL.I->idq_cmd[0];

    // TODO: when id changes, the gain should also change.
    if(nsoaf.set_omega_ob != nsoaf.omega_ob){
        nsoaf.omega_ob = nsoaf.set_omega_ob;
        nso_one_parameter_tuning(nsoaf.omega_ob);
        // afe_one_parameter_tuning(nsoaf.omega_ob);
    }

    /* Unpack States */
    nsoaf.xIq      = nsoaf.xBest[0];
    nsoaf.xOmg     = nsoaf.xBest[1];
    nsoaf.xTL      = nsoaf.xBest[2];

    /* Know Signals */
    #define MOTOR (*CTRL.motor)
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS_C(0), IS_C(1), AFE_USED.cosT, AFE_USED.sinT);
    iDQ_now[1]  = AB2T(IS_C(0), IS_C(1), AFE_USED.cosT, AFE_USED.sinT);
    REAL uQ_now = AB2T(US_C(0), US_C(1), AFE_USED.cosT, AFE_USED.sinT);
    nsoaf.q_axis_voltage = uQ_now;

    /* Output Error = \tilde i_q (scalar) */
    nsoaf.output_error = iDQ_now[1] - nsoaf.xIq;
    nsoaf.active_power_real  = + fabs(uQ_now) * iDQ_now[1];
    nsoaf.active_power_est   = + fabs(uQ_now) * nsoaf.xIq;
    nsoaf.active_power_error = + fabs(uQ_now) * nsoaf.output_error;

    REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * KActive * nsoaf.xIq;



    /*  xTL的方程只是提供了粒子的xTL的取值建议；
        转速的方程，会根据你的粒子的xTL的值去计算出“下一步的转速”；
        然后你才根据下一步的电流来判断上面得到的“下一步的转速”到底准不准。
    */
    /* 放弃 MPO 或者说 DBO 的原因是目标函数（输出误差varepsilon）为零并不对应最优点 */
        // REAL fx[NS];
        // REAL iQ_C = AB2T(IS_C(0), IS_C(1), nsoaf.cosT, nsoaf.sinT);

        // REAL best_xTL;

        // 直接解算令输出误差为零的转速值（欧拉法ode1）
        // nsoaf.solved_xOmg = ((uQ_now - MOTOR.R * xIq       ) - (iQ_C - xIq)*MOTOR.Lq*CL_TS_INVERSE) / (MOTOR.KE + MOTOR.Ld*iDQ_now[0]);
        // nsoaf.solved_xOmg = ((uQ_now - MOTOR.R * iDQ_now[1]) - (iQ_C - xIq)*MOTOR.Lq*CL_TS_INVERSE) / (MOTOR.KE + MOTOR.Ld*iDQ_now[0]);

        // REAL iQ_C = AB2T(IS_C(0), IS_C(1), nsoaf.cosT, nsoaf.sinT);
        // REAL best_output_error = 99999;
        // int  best_index = 0;
        // int j;
        // for(j=0; j<10; ++j){
        //     nsoaf.KP = (j+1) * 0.2 * NSOAF_TL_P*0;
        //     nsoaf.KI = (j+1) * 0.2 * NSOAF_TL_I;
        //     nsoaf.KD = (j+1) * 0.2 * NSOAF_TL_D;
        //     general_3_states_rk4_solver(&rhf_NSOAF_Dynamics, CTRL.timebase, (nsoaf.x+j*NS), CL_TS);
        //     nsoaf.output_error = iQ_C - *(nsoaf.x+j*NS);
        //     if(fabs(nsoaf.output_error) < fabs(best_output_error)){
        //         best_output_error = nsoaf.output_error;
        //         best_index = j;
        //     }
        // }
        // nsoaf.xBest[0] = nsoaf.x[best_index*NS+0];
        // nsoaf.xBest[1] = nsoaf.x[best_index*NS+1];
        // nsoaf.xBest[2] = nsoaf.x[best_index*NS+2];

        // DEBUG
        // nsoaf.xBest[1] = ACM.omg_elec;

    general_3states_rk4_solver(&rhf_NSOAF_Dynamics, CTRL.timebase, nsoaf.xBest, CL_TS);


    /* Unpack x (for the best) */
    // nsoaf.xIq      = nsoaf.xBest[0];
    // nsoaf.xOmg     = nsoaf.xBest[1];
    // nsoaf.xTL      = nsoaf.xBest[2];
    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */
    nsoaf.LoadTorquePI = nsoaf.xTL + nsoaf.KP*nsoaf.active_power_error;
}
void init_nsoaf(){

    nsoaf.KP = NSOAF_TL_P;
    nsoaf.KI = NSOAF_TL_I;
    nsoaf.KD = NSOAF_TL_D;
    nsoaf.set_omega_ob = NSOAF_OMEGA_OBSERVER;

    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * MOTOR.Lq*CTRL.I->idq[0];
    nsoaf.omega_ob = nsoaf.set_omega_ob;
    nso_one_parameter_tuning(nsoaf.omega_ob);
}
#endif

/********************************************/
/* EEMF-Speed-Adaptive-High-Gain-Observer-Farza-2009
 ********************************************/
/* The 10th-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Farza_2009
#define NS 10
void rhf_func_hgoeemf(REAL *increment_n, REAL *xPsi, REAL *xEemf, 
                      REAL xOmg, REAL xGammaOmg, 
                      REAL *xPhi, REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    // f = \dot x = the time derivative
    REAL f[NS];

    // DEBUG
    // xOmg = CTRL.I->omg_elec;

    // dependent parameters
    // MOTOR.Ld
    // MOTOR.DeltaL
    // MOTOR.R
    #define MOTOR (*CTRL.motor)

    /* Output Error of xPsi */
    hgo4eemf.output_error[0] = MOTOR.Ld*IS(0) - xPsi[0];
    hgo4eemf.output_error[1] = MOTOR.Ld*IS(1) - xPsi[1];

    /* Parameter Adaptation */
    // xOmg (recall xPhi is the filtered regreesor for speed)
    f[4] = hgo4eemf.vartheta*hgo4eemf.xGammaOmg * (xPhi[0]*hgo4eemf.output_error[0] + xPhi[1]*hgo4eemf.output_error[1]);
    // xGammaOmg
    f[5] = - hgo4eemf.vartheta * xGammaOmg * (xPhi[0]*xPhi[0]+xPhi[1]*xPhi[1]) * xGammaOmg + hgo4eemf.vartheta*xGammaOmg;

    /* Auxiliary States */
    // xPhi
    f[6] = hgo4eemf.vartheta * (-2*xPhi[0] + xPhi[2]) + MOTOR.DeltaL*-IS(1);
    f[7] = hgo4eemf.vartheta * (-2*xPhi[1] + xPhi[3]) + MOTOR.DeltaL* IS(0);
    f[8] = hgo4eemf.vartheta * (  -xPhi[0] ) + hgo4eemf.vartheta_inv*-xEemf[1];
    f[9] = hgo4eemf.vartheta * (  -xPhi[1] ) + hgo4eemf.vartheta_inv* xEemf[0];

    /* State Observer */
    // xPsi = Ld * i
    f[0] = xEemf[0] + US(0) - MOTOR.R*IS(0) + MOTOR.DeltaL*xOmg*-IS(1) + 2*hgo4eemf.vartheta*hgo4eemf.output_error[0] + xPhi[0]*f[4];
    f[1] = xEemf[1] + US(1) - MOTOR.R*IS(1) + MOTOR.DeltaL*xOmg* IS(0) + 2*hgo4eemf.vartheta*hgo4eemf.output_error[1] + xPhi[1]*f[4];
    // xEemf
    f[2] = xOmg*-xEemf[1] + hgo4eemf.vartheta * (hgo4eemf.vartheta * hgo4eemf.output_error[0]                         + xPhi[2]*f[4]);
    f[3] = xOmg* xEemf[0] + hgo4eemf.vartheta * (hgo4eemf.vartheta * hgo4eemf.output_error[1]                         + xPhi[3]*f[4]);

    increment_n[ 0] = ( f[ 0] )*hs;
    increment_n[ 1] = ( f[ 1] )*hs;
    increment_n[ 2] = ( f[ 2] )*hs;
    increment_n[ 3] = ( f[ 3] )*hs;
    increment_n[ 4] = ( f[ 4] )*hs;
    increment_n[ 5] = ( f[ 5] )*hs;
    increment_n[ 6] = ( f[ 6] )*hs;
    increment_n[ 7] = ( f[ 7] )*hs;
    increment_n[ 8] = ( f[ 8] )*hs;
    increment_n[ 9] = ( f[ 9] )*hs;
}
void hgo4eemf_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[NS];
    static REAL increment_2[NS];
    static REAL increment_3[NS];
    static REAL increment_4[NS];
    static REAL x_temp[NS];
    static REAL *p_x_temp=x_temp;

    /* Theoritically speaking, rhf_func should be time-varing like rhf_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhf_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhf_func_hgoeemf( increment_1, hgo4eemf.xPsi, hgo4eemf.xEemf, 
                      hgo4eemf.xOmg, hgo4eemf.xGammaOmg,
                      hgo4eemf.xPhi, hs); 
    x_temp[0]  = hgo4eemf.xPsi[0]        + increment_1[0]*0.5;
    x_temp[1]  = hgo4eemf.xPsi[1]        + increment_1[1]*0.5;
    x_temp[2]  = hgo4eemf.xEemf[0]       + increment_1[2]*0.5;
    x_temp[3]  = hgo4eemf.xEemf[1]       + increment_1[3]*0.5;
    x_temp[4]  = hgo4eemf.xOmg           + increment_1[4]*0.5;
    x_temp[5]  = hgo4eemf.xGammaOmg      + increment_1[5]*0.5;
    x_temp[6]  = hgo4eemf.xPhi[0]        + increment_1[6]*0.5;
    x_temp[7]  = hgo4eemf.xPhi[1]        + increment_1[7]*0.5;
    x_temp[8]  = hgo4eemf.xPhi[2]        + increment_1[8]*0.5;
    x_temp[9]  = hgo4eemf.xPhi[3]        + increment_1[9]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhf_func_hgoeemf( increment_2, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    x_temp[0]  = hgo4eemf.xPsi[0]        + increment_2[0]*0.5;
    x_temp[1]  = hgo4eemf.xPsi[1]        + increment_2[1]*0.5;
    x_temp[2]  = hgo4eemf.xEemf[0]       + increment_2[2]*0.5;
    x_temp[3]  = hgo4eemf.xEemf[1]       + increment_2[3]*0.5;
    x_temp[4]  = hgo4eemf.xOmg           + increment_2[4]*0.5;
    x_temp[5]  = hgo4eemf.xGammaOmg      + increment_2[5]*0.5;
    x_temp[6]  = hgo4eemf.xPhi[0]        + increment_2[6]*0.5;
    x_temp[7]  = hgo4eemf.xPhi[1]        + increment_2[7]*0.5;
    x_temp[8]  = hgo4eemf.xPhi[2]        + increment_2[8]*0.5;
    x_temp[9]  = hgo4eemf.xPhi[3]        + increment_2[9]*0.5;

    // time instant t+hs/2
    rhf_func_hgoeemf( increment_3, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    x_temp[0]  = hgo4eemf.xPsi[0]        + increment_3[0];
    x_temp[1]  = hgo4eemf.xPsi[1]        + increment_3[1];
    x_temp[2]  = hgo4eemf.xEemf[0]       + increment_3[2];
    x_temp[3]  = hgo4eemf.xEemf[1]       + increment_3[3];
    x_temp[4]  = hgo4eemf.xOmg           + increment_3[4];
    x_temp[5]  = hgo4eemf.xGammaOmg      + increment_3[5];
    x_temp[6]  = hgo4eemf.xPhi[0]        + increment_3[6];
    x_temp[7]  = hgo4eemf.xPhi[1]        + increment_3[7];
    x_temp[8]  = hgo4eemf.xPhi[2]        + increment_3[8];
    x_temp[9]  = hgo4eemf.xPhi[3]        + increment_3[9];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhf_func_hgoeemf( increment_4, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    hgo4eemf.xPsi[0]         +=   (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    hgo4eemf.xPsi[1]         +=   (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    hgo4eemf.xEemf[0]        +=   (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    hgo4eemf.xEemf[1]        +=   (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3
    hgo4eemf.xOmg            +=   (increment_1[4] + 2*(increment_2[4] + increment_3[4]) + increment_4[4])*0.166666666666667; // 4
    hgo4eemf.xGammaOmg       +=   (increment_1[5] + 2*(increment_2[5] + increment_3[5]) + increment_4[5])*0.166666666666667; // 5
    hgo4eemf.xPhi[0]         +=   (increment_1[6] + 2*(increment_2[6] + increment_3[6]) + increment_4[6])*0.166666666666667; // 6
    hgo4eemf.xPhi[1]         +=   (increment_1[7] + 2*(increment_2[7] + increment_3[7]) + increment_4[7])*0.166666666666667; // 7
    hgo4eemf.xPhi[2]         +=   (increment_1[8] + 2*(increment_2[8] + increment_3[8]) + increment_4[8])*0.166666666666667; // 8
    hgo4eemf.xPhi[3]         +=   (increment_1[9] + 2*(increment_2[9] + increment_3[9]) + increment_4[9])*0.166666666666667; // 9

    /* Post-Observer */
    hgo4eemf.theta_d = atan2( 
                             - hgo4eemf.xEemf[0]*(sign(hgo4eemf.xOmg)), 
                               hgo4eemf.xEemf[1]*(sign(hgo4eemf.xOmg))
                           );
}
/* Main Observer */
void Main_cjh_eemfhgo_farza09(){

    /* OBSERVATION */
    hgo4eemf_dedicated_rk4_solver(1*CL_TS);
}
void init_hgo4eemf(){
    hgo4eemf.vartheta  = FARZA09_HGO_EEMF_VARTHETA;
    if(hgo4eemf.vartheta==0.0){
        #if PC_SIMULATION
            printf("varthteta cannot be zero\n");
        #endif
    }else{
        hgo4eemf.vartheta_inv = 1.0 / hgo4eemf.vartheta;
    }
    hgo4eemf.xGammaOmg = FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE;
}
#undef NS
#endif


/********************************************/
/* EEMF-Error-Dynamics-based-AO-Design
 ********************************************/
/* The 13th-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_CJH_EEMF
#define NS 13
void rhf_func_eemfaod(REAL *increment_n, REAL *xPsi, REAL *xChi, REAL xOmg, 
                      REAL *xEta,
                      REAL *xVarsigma,
                      REAL *xUpsilon,
                      REAL *xZeta,    
                      REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    // DEBUG
    // xOmg = CTRL.I->omg_elec;

    // dependent parameters
    #define MOTOR (*CTRL.motor)
    // MOTOR.Ld
    // MOTOR.DeltaL
    // MOTOR.R
    // cjheemf.k1
    // cjheemf.k2
    // cjheemf.gamma_omg

    cjheemf.output_error[0] = MOTOR.Ld*IS(0) - xPsi[0];
    cjheemf.output_error[1] = MOTOR.Ld*IS(1) - xPsi[1];
    cjheemf.effective_output_error[0] = cjheemf.output_error[0] + xEta[0] - xUpsilon[0]*xOmg;
    cjheemf.effective_output_error[1] = cjheemf.output_error[1] + xEta[1] - xUpsilon[1]*xOmg;

    // f = \dot x = the time derivative
    REAL f[NS];
    REAL TwoLdMinusLq = MOTOR.DeltaL + MOTOR.Ld;
    // xPsi
    f[0] = US(0) - MOTOR.R* IS(0) + xChi[0] + TwoLdMinusLq*xOmg*-IS(1) + cjheemf.k1*cjheemf.output_error[0];
    f[1] = US(1) - MOTOR.R* IS(1) + xChi[1] + TwoLdMinusLq*xOmg* IS(0) + cjheemf.k1*cjheemf.output_error[1];
    // xChi
    f[2] = xOmg*xOmg * MOTOR.DeltaL*IS(0) - xOmg*-US(1) + xOmg*MOTOR.R*-IS(1) + cjheemf.k2*cjheemf.output_error[0];
    f[3] = xOmg*xOmg * MOTOR.DeltaL*IS(1) - xOmg* US(0) + xOmg*MOTOR.R* IS(0) + cjheemf.k2*cjheemf.output_error[1];
    // xOmg (recall xUpsilon is the filtered regreesor for speed)
    f[4] = cjheemf.gamma_omg * (xUpsilon[0]*cjheemf.effective_output_error[0] + xUpsilon[1]*cjheemf.effective_output_error[1]);
    // f[4] = cjheemf.gamma_omg * (xUpsilon[0]*-cjheemf.effective_output_error[1] + xUpsilon[1]*cjheemf.effective_output_error[0]);

    /* Auxiliary States*/
    // xEta 
    f[5] = - cjheemf.k1*xEta[0] + xVarsigma[0] + xOmg*TwoLdMinusLq*-IS(1);
    f[6] = - cjheemf.k1*xEta[1] + xVarsigma[1] + xOmg*TwoLdMinusLq* IS(0);
    // varsigma
    f[7] = - cjheemf.k2*xVarsigma[0] + xOmg*((xOmg+xOmg)*MOTOR.DeltaL*IS(0) +US(1) + MOTOR.R*-IS(1));
    f[8] = - cjheemf.k2*xVarsigma[1] + xOmg*((xOmg+xOmg)*MOTOR.DeltaL*IS(1) -US(0) + MOTOR.R* IS(0));
    //  xUpsilon
    f[9]  = - cjheemf.k1*xUpsilon[0] + xZeta[0] + TwoLdMinusLq*-IS(1);
    f[10] = - cjheemf.k1*xUpsilon[1] + xZeta[1] + TwoLdMinusLq* IS(0);
    // zeta
    f[11] = - cjheemf.k2*xZeta[0] + ((xOmg+xOmg)*MOTOR.DeltaL*IS(0) +US(1) + MOTOR.R*-IS(1));
    f[12] = - cjheemf.k2*xZeta[1] + ((xOmg+xOmg)*MOTOR.DeltaL*IS(1) -US(0) + MOTOR.R* IS(0));

    increment_n[ 0] = ( f[ 0] )*hs;
    increment_n[ 1] = ( f[ 1] )*hs;
    increment_n[ 2] = ( f[ 2] )*hs;
    increment_n[ 3] = ( f[ 3] )*hs;
    increment_n[ 4] = ( f[ 4] )*hs;
    increment_n[ 5] = ( f[ 5] )*hs;
    increment_n[ 6] = ( f[ 6] )*hs;
    increment_n[ 7] = ( f[ 7] )*hs;
    increment_n[ 8] = ( f[ 8] )*hs;
    increment_n[ 9] = ( f[ 9] )*hs;
    increment_n[10] = ( f[10] )*hs;
    increment_n[11] = ( f[11] )*hs;
    increment_n[12] = ( f[12] )*hs;
}
void eemf_ao_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[NS];
    static REAL increment_2[NS];
    static REAL increment_3[NS];
    static REAL increment_4[NS];
    static REAL x_temp[NS];
    static REAL *p_x_temp=x_temp;

    /* Theoritically speaking, rhf_func should be time-varing like rhf_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhf_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhf_func_eemfaod( increment_1, cjheemf.xPsi, cjheemf.xChi, cjheemf.xOmg,
                      cjheemf.xEta,
                      cjheemf.xVarsigma,
                      cjheemf.xUpsilon,
                      cjheemf.xZeta,
                      hs); 
    x_temp[ 0]  = cjheemf.xPsi[0]        + increment_1[ 0]*0.5;
    x_temp[ 1]  = cjheemf.xPsi[1]        + increment_1[ 1]*0.5;
    x_temp[ 2]  = cjheemf.xChi[0]        + increment_1[ 2]*0.5;
    x_temp[ 3]  = cjheemf.xChi[1]        + increment_1[ 3]*0.5;
    x_temp[ 4]  = cjheemf.xOmg           + increment_1[ 4]*0.5;
    x_temp[ 5]  = cjheemf.xEta[0]        + increment_1[ 5]*0.5;
    x_temp[ 6]  = cjheemf.xEta[1]        + increment_1[ 6]*0.5;
    x_temp[ 7]  = cjheemf.xVarsigma[0]   + increment_1[ 7]*0.5;
    x_temp[ 8]  = cjheemf.xVarsigma[1]   + increment_1[ 8]*0.5;
    x_temp[ 9]  = cjheemf.xUpsilon[0]    + increment_1[ 9]*0.5;
    x_temp[10]  = cjheemf.xUpsilon[1]    + increment_1[10]*0.5;
    x_temp[11]  = cjheemf.xZeta[0]       + increment_1[11]*0.5;
    x_temp[12]  = cjheemf.xZeta[1]       + increment_1[12]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhf_func_eemfaod( increment_2, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    x_temp[ 0]  = cjheemf.xPsi[0]        + increment_2[ 0]*0.5;
    x_temp[ 1]  = cjheemf.xPsi[1]        + increment_2[ 1]*0.5;
    x_temp[ 2]  = cjheemf.xChi[0]        + increment_2[ 2]*0.5;
    x_temp[ 3]  = cjheemf.xChi[1]        + increment_2[ 3]*0.5;
    x_temp[ 4]  = cjheemf.xOmg           + increment_2[ 4]*0.5;
    x_temp[ 5]  = cjheemf.xEta[0]        + increment_2[ 5]*0.5;
    x_temp[ 6]  = cjheemf.xEta[1]        + increment_2[ 6]*0.5;
    x_temp[ 7]  = cjheemf.xVarsigma[0]   + increment_2[ 7]*0.5;
    x_temp[ 8]  = cjheemf.xVarsigma[1]   + increment_2[ 8]*0.5;
    x_temp[ 9]  = cjheemf.xUpsilon[0]    + increment_2[ 9]*0.5;
    x_temp[10]  = cjheemf.xUpsilon[1]    + increment_2[10]*0.5;
    x_temp[11]  = cjheemf.xZeta[0]       + increment_2[11]*0.5;
    x_temp[12]  = cjheemf.xZeta[1]       + increment_2[12]*0.5;

    // time instant t+hs/2
    rhf_func_eemfaod( increment_3, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    x_temp[ 0]  = cjheemf.xPsi[0]        + increment_3[ 0];
    x_temp[ 1]  = cjheemf.xPsi[1]        + increment_3[ 1];
    x_temp[ 2]  = cjheemf.xChi[0]        + increment_3[ 2];
    x_temp[ 3]  = cjheemf.xChi[1]        + increment_3[ 3];
    x_temp[ 4]  = cjheemf.xOmg           + increment_3[ 4];
    x_temp[ 5]  = cjheemf.xEta[0]        + increment_3[ 5];
    x_temp[ 6]  = cjheemf.xEta[1]        + increment_3[ 6];
    x_temp[ 7]  = cjheemf.xVarsigma[0]   + increment_3[ 7];
    x_temp[ 8]  = cjheemf.xVarsigma[1]   + increment_3[ 8];
    x_temp[ 9]  = cjheemf.xUpsilon[0]    + increment_3[ 9];
    x_temp[10]  = cjheemf.xUpsilon[1]    + increment_3[10];
    x_temp[11]  = cjheemf.xZeta[0]       + increment_3[11];
    x_temp[12]  = cjheemf.xZeta[1]       + increment_3[12];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhf_func_eemfaod( increment_4, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    cjheemf.xPsi[0]        +=   (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    cjheemf.xPsi[1]        +=   (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    cjheemf.xChi[0]        +=   (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    cjheemf.xChi[1]        +=   (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3
    cjheemf.xOmg           +=   (increment_1[4] + 2*(increment_2[4] + increment_3[4]) + increment_4[4])*0.166666666666667; // 4
    // if(CTRL.timebase>1.6){
    //     printf("%g\n", cjheemf.xOmg);;
    // }
    cjheemf.xEta[0]        +=   (increment_1[5] + 2*(increment_2[5] + increment_3[5]) + increment_4[5])*0.166666666666667; // 5
    cjheemf.xEta[1]        +=   (increment_1[6] + 2*(increment_2[6] + increment_3[6]) + increment_4[6])*0.166666666666667; // 6
    cjheemf.xVarsigma[0]   +=   (increment_1[7] + 2*(increment_2[7] + increment_3[7]) + increment_4[7])*0.166666666666667; // 7
    cjheemf.xVarsigma[1]   +=   (increment_1[8] + 2*(increment_2[8] + increment_3[8]) + increment_4[8])*0.166666666666667; // 8
    cjheemf.xUpsilon[0]    +=   (increment_1[9] + 2*(increment_2[9] + increment_3[9]) + increment_4[9])*0.166666666666667; // 9
    cjheemf.xUpsilon[1]    +=   (increment_1[10] + 2*(increment_2[10] + increment_3[10]) + increment_4[10])*0.166666666666667; // 10
    cjheemf.xZeta[0]       +=   (increment_1[11] + 2*(increment_2[11] + increment_3[11]) + increment_4[11])*0.166666666666667; // 11
    cjheemf.xZeta[1]       +=   (increment_1[12] + 2*(increment_2[12] + increment_3[12]) + increment_4[12])*0.166666666666667; // 12


    cjheemf.xEemf[0] = - CTRL.motor->Ld * cjheemf.xOmg *-IS_C(1) - cjheemf.xChi[0]; 
    cjheemf.xEemf[1] = - CTRL.motor->Ld * cjheemf.xOmg * IS_C(0) - cjheemf.xChi[1]; 
    cjheemf.theta_d = atan2( 
                             - cjheemf.xEemf[0]*(sign(cjheemf.xOmg)), 
                               cjheemf.xEemf[1]*(sign(cjheemf.xOmg))
                           );
}
/* Main Observer */
void Main_cjh_eemfao(){

    /* OBSERVATION */
    eemf_ao_dedicated_rk4_solver(1*CL_TS);
}
void init_cjheemf(){
    cjheemf.k1        = CJH_EEMF_K1;
    cjheemf.k2        = CJH_EEMF_K2;
    cjheemf.gamma_omg = CJH_EEMF_GAMMA_OMEGA;
}
#undef NS
#endif


/********************************************/
/* Harnefor 2006
 ********************************************/
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Harnefors_2006
void init_harnefors(){
    // harnefors.theta_d = 0.0;
    // harnefors.omg_elec = 0.0;

    harnefors.svf_p0 = SVF_POLE_0_VALUE;
    harnefors.xSVF_curr[0] = 0.0;
    harnefors.xSVF_curr[1] = 0.0;
    harnefors.xSVF_prev[0] = 0.0;
    harnefors.xSVF_prev[1] = 0.0;
    harnefors.is_dq[0] = 0.0;
    harnefors.is_dq[1] = 0.0;
    harnefors.is_dq_curr[0] = 0.0;
    harnefors.is_dq_curr[1] = 0.0;
    harnefors.is_dq_prev[0] = 0.0;
    harnefors.is_dq_prev[1] = 0.0;
    harnefors.pis_dq[0] = 0.0;
    harnefors.pis_dq[1] = 0.0;    
}
void state_variable_filter(REAL hs){
    // output += SVF_POLE_0 * (input - last_output) * hs;
    // last_output = output;
    // if input == output:
    //     derivative of input = SVF_POLE_0 * (input - last_output)

    /* State Variable Filter for ob.pis[.] Computed by Standalone RK4 */
    REAL xxsvf1[2];
    REAL xxsvf2[2];
    REAL xxsvf3[2];
    REAL xxsvf4[2];
    REAL xsvf_temp[2];
    // step 1 @ t
    xxsvf1[0] = SVF_POLE_0 * (IDQ_P(0) - SVF_P(0)) * hs;
    xxsvf1[1] = SVF_POLE_0 * (IDQ_P(1) - SVF_P(1)) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（上一步的状态的值，其他时变的已知量在t时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf1[0]*0.5;
    xsvf_temp[1] = SVF_P(1) + xxsvf1[1]*0.5;
    // 临时的状态变量 = 上一步的状态变量 + step1增量*0.5；

    // step 2 @ t+hs/2
    IDQ(0) = 0.5*(IDQ_P(0)+IDQ_C(0));
    IDQ(1) = 0.5*(IDQ_P(1)+IDQ_C(1));
    xxsvf2[0] = SVF_POLE_0 * (IDQ(0) - xsvf_temp[0]) * hs;
    xxsvf2[1] = SVF_POLE_0 * (IDQ(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t+hs/2时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf2[0]*0.5;
    xsvf_temp[1] = SVF_P(1) + xxsvf2[1]*0.5;
    // 临时的状态变量 = 上一步的状态变量 + step2增量*0.5；

    // step 3 @ t+hs/2
    xxsvf3[0] = SVF_POLE_0 * (IDQ(0) - xsvf_temp[0]) * hs;
    xxsvf3[1] = SVF_POLE_0 * (IDQ(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t+hs/2时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf3[0];
    xsvf_temp[1] = SVF_P(1) + xxsvf3[1];
    // 临时的状态变量 = 上一步的状态变量 + step3增量*1.0；

    // step 4 @ t+hs
    xxsvf4[0] = SVF_POLE_0 * (IDQ_C(0) - xsvf_temp[0]) * hs;
    xxsvf4[1] = SVF_POLE_0 * (IDQ_C(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t时刻的值） * 数值积分的步长；
    SVF_C(0) += (xxsvf1[0] + 2*(xxsvf2[0] + xxsvf3[0]) + xxsvf4[0])*0.166666666666667; // 0
    SVF_C(1) += (xxsvf1[1] + 2*(xxsvf2[1] + xxsvf3[1]) + xxsvf4[1])*0.166666666666667; // 1    
    // 数值积分的结果 = （step1增量+2*step2增量+2*step3增量+step4的增量）/6

    PIDQ(0) = SVF_POLE_0 * (IDQ_C(0) - SVF_C(0));
    PIDQ(1) = SVF_POLE_0 * (IDQ_C(1) - SVF_C(1));
}
// Harnefors 2003: mu=-1, lambda=sqrt(2)
// Harnefors 2006
#define LAMBDA 2
void Main_harnefors_scvm(){

    // 这一组参数很适合伺尔沃的400W，也可以用于100W和200W（delta=6.5，VLBW=50Hz）
        // #define CJH_TUNING_A  25 // low voltage servo motor (<48 Vdc)
        // #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
        // #define CJH_TUNING_C 1
    // 为NTU的伺服调试的参数（delta=3，VLBW=40Hz）
        #define CJH_TUNING_A  1 // low voltage servo motor (<48 Vdc)
        #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
        #define CJH_TUNING_C 0.2 // [0.2， 0.5]
    // 可调参数壹
    REAL lambda_s = CJH_TUNING_C * LAMBDA * sign(harnefors.omg_elec);
    // 可调参数贰
    REAL alpha_bw_lpf = CJH_TUNING_A*0.1*(1500*RPM_2_ELEC_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabs(harnefors.omg_elec);


    // 一阶差分计算DQ电流的导数
    static REAL last_id = 0.0;
    static REAL last_iq = 0.0;
    // #define D_AXIS_CURRENT CTRL.id_cmd
    // #define Q_AXIS_CURRENT CTRL.iq_cmd
    // harnefors.deriv_id = (CTRL.id_cmd - last_id) * CL_TS_INVERSE;
    // harnefors.deriv_iq = (CTRL.iq_cmd - last_iq) * CL_TS_INVERSE;
    // last_id = CTRL.id_cmd;
    // last_iq = CTRL.iq_cmd;
    #define D_AXIS_CURRENT CTRL.I->idq[0]
    #define Q_AXIS_CURRENT CTRL.I->idq[1]
    harnefors.deriv_id = (D_AXIS_CURRENT - last_id) * CL_TS_INVERSE;
    harnefors.deriv_iq = (Q_AXIS_CURRENT - last_iq) * CL_TS_INVERSE;
    last_id = D_AXIS_CURRENT;
    last_iq = Q_AXIS_CURRENT;

    // 用SVF计算DQ电流的导数
    IDQ_C(0) = D_AXIS_CURRENT;
    IDQ_C(1) = Q_AXIS_CURRENT;
    state_variable_filter(CL_TS);
    IDQ_P(0) = IDQ_C(0); // used in SVF 
    IDQ_P(1) = IDQ_C(1); // used in SVF 
    SVF_P(0) = SVF_C(0);
    SVF_P(1) = SVF_C(1);
    #define DERIV_ID PIDQ(0)
    #define DERIV_IQ PIDQ(1)

    #define UD_CMD CTRL.O->udq_cmd[0]
    #define UQ_CMD CTRL.O->udq_cmd[1]
    #define MOTOR  (*CTRL.motor)

    // // 计算反电势（考虑dq电流导数）
    // #define BOOL_COMPENSATE_PIDQ 1
    // REAL d_axis_emf = UD_CMD - MOTOR.R*D_AXIS_CURRENT + harnefors.omg_elec*MOTOR.Lq*Q_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Ld*DERIV_ID; // eemf
    // REAL q_axis_emf = UQ_CMD - MOTOR.R*Q_AXIS_CURRENT - harnefors.omg_elec*MOTOR.Ld*D_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Lq*DERIV_IQ; // eemf

    // 计算反电势（忽略dq电流导数）
    #define BOOL_COMPENSATE_PIDQ 0
    REAL d_axis_emf = UD_CMD - MOTOR.R*D_AXIS_CURRENT + harnefors.omg_elec*MOTOR.Lq*Q_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Ld*DERIV_ID; // eemf
    REAL q_axis_emf = UQ_CMD - MOTOR.R*Q_AXIS_CURRENT - harnefors.omg_elec*MOTOR.Ld*D_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Lq*DERIV_IQ; // eemf

    // 数值积分获得转速和转子位置
        // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
    #define KE_MISMATCH 1.0 // 0.7
    harnefors.theta_d  += CL_TS * harnefors.omg_elec;
    harnefors.omg_elec += CL_TS * alpha_bw_lpf * ( (q_axis_emf - lambda_s*d_axis_emf)/(MOTOR.KE*KE_MISMATCH+(MOTOR.Ld-MOTOR.Lq)*D_AXIS_CURRENT) - harnefors.omg_elec );

    // 转子位置周期限幅
    while(harnefors.theta_d>M_PI) harnefors.theta_d-=2*M_PI;
    while(harnefors.theta_d<-M_PI) harnefors.theta_d+=2*M_PI;
}
#endif


/********************************************/
/* Qiao.Xia 2013 emfSMO with Sigmoid (Also Hongryel Kim 2011) as Nonlinear
 ********************************************/
REAL sm_sigmoid(REAL x, REAL a){
    return 2.0 / (1.0 + exp(-a*x)) - 1.0;
}
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Qiao_Xia
void init_QiaoXia2013(){
    qiaoxia.smo_gain      = 0.0; // time varying gain
    qiaoxia.sigmoid_coeff = QIAO_XIA_SIGMOID_COEFF;   // 17
    qiaoxia.adapt_gain    = QIAO_XIA_ADAPT_GAIN; // this is fixed to be 1 in the paper
    qiaoxia.mras_gain     = QIAO_XIA_MRAS_GAIN;
}
void rhf_QiaoXia2013_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xIa   = x[0];
    REAL xIb   = x[1];
    REAL xEmfa = x[2];
    REAL xEmfb = x[3];
    REAL xOmg  = x[4];

    /* Know Signals */
    // #define MOTOR (*CTRL.motor)

    /* Output Error = \tilde i_q (scalar) */
    qiaoxia.output_error[0] = IS(0) - xIa;
    qiaoxia.output_error[1] = IS(1) - xIb;

    /* Sliding Mode Switching Term*/
    qiaoxia.xEmf_raw[0] = qiaoxia.smo_gain * sm_sigmoid(qiaoxia.output_error[0], qiaoxia.sigmoid_coeff);
    qiaoxia.xEmf_raw[1] = qiaoxia.smo_gain * sm_sigmoid(qiaoxia.output_error[1], qiaoxia.sigmoid_coeff);
    // qiaoxia.xEmf_raw[0] = qiaoxia.smo_gain * sign(qiaoxia.output_error[0]);
    // qiaoxia.xEmf_raw[1] = qiaoxia.smo_gain * sign(qiaoxia.output_error[1]);

    /* State Observer */
    // xIab
    fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * xIa + qiaoxia.xEmf_raw[0]);
    fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * xIb + qiaoxia.xEmf_raw[1]);
    // xEmf
    #define OMG_USED xOmg //CTRL.I->omg_elec // 
    fx[2] = -OMG_USED*xEmfb + qiaoxia.mras_gain * (qiaoxia.xEmf_raw[0] - xEmfa);
    fx[3] =  OMG_USED*xEmfa + qiaoxia.mras_gain * (qiaoxia.xEmf_raw[1] - xEmfb);
    // xOmg
    fx[4] = qiaoxia.adapt_gain * (qiaoxia.xEmf_raw[0] - xEmfa)*-xEmfb + (qiaoxia.xEmf_raw[1] - xEmfb)*xEmfa;
}
void Main_QiaoXia2013_emfSMO(){

    /* Time-varying gains */
    qiaoxia.smo_gain = QIAO_XIA_SMO_GAIN * fabs(CTRL.I->cmd_omg_elec) * MOTOR.KE;
    // qiaoxia.smo_gain = QIAO_XIA_SMO_GAIN * 70 * MOTOR.KE;

    general_5states_rk4_solver(&rhf_QiaoXia2013_Dynamics, CTRL.timebase, qiaoxia.x, CL_TS);

    /* Unpack x (for the best) */
    qiaoxia.xIab[0] = qiaoxia.x[0];
    qiaoxia.xIab[1] = qiaoxia.x[1];
    qiaoxia.xEmf[0] = qiaoxia.x[2];
    qiaoxia.xEmf[1] = qiaoxia.x[3];
    qiaoxia.xOmg    = qiaoxia.x[4];
    // REAL temp = -atan2( qiaoxia.xEmf[0]*-sign(CTRL.I->cmd_speed_rpm),   // qiaoxia.xOmg
    //                     qiaoxia.xEmf[1]*-sign(CTRL.I->cmd_speed_rpm) ); // qiaoxia.xOmg
    REAL temp = -atan2( qiaoxia.xEmf[0]*sign(-qiaoxia.xOmg),   // qiaoxia.xOmg
                        qiaoxia.xEmf[1]*sign(-qiaoxia.xOmg) ); // qiaoxia.xOmg
    // REAL temp = -atan2( qiaoxia.xEmf[0],   // qiaoxia.xOmg
                        // qiaoxia.xEmf[1] ); // qiaoxia.xOmg
    qiaoxia.theta_d = temp;
    // if(temp>0){
    //     qiaoxia.theta_d = temp - M_PI;
    // }else{
    //     qiaoxia.theta_d = temp + M_PI;
    // }
    // if(qiaoxia.theta_d>M_PI){
    //     qiaoxia.theta_d -= M_PI;
    // }else if(qiaoxia.theta_d<-M_PI){
    //     qiaoxia.theta_d += M_PI;
    // }

    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */    
}
#endif



/********************************************/
/* SongChi.LongyaXu 2009 emfSMO 
 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Chi_Xu
void init_ChiXu2009(){
    /*实验中Chi.Xu的状态x在调参过程中经常变成+inf，所以这里初始化一下。*/
    chixu.x[0] = 0.0;
    chixu.x[1] = 0.0;
    chixu.x[2] = 0.0;
    chixu.x[3] = 0.0;

    chixu.smo_gain      = 400.0; // CHI_XU_USE_CONSTANT_SMO_GAIN
    chixu.sigmoid_coeff = CHI_XU_SIGMOID_COEFF;   // 17
    chixu.PLL_KP        = CHI_XU_SPEED_PLL_KP;
    chixu.PLL_KI        = CHI_XU_SPEED_PLL_KI;
    chixu.ell4xZeq      = -0.5; // -0.5 for low speed and 1.0 for high speed
    chixu.omega_lpf_4_xZeq_const_part = CHI_XU_LPF_4_ZEQ;
    chixu.omega_lpf_4_xZeq = chixu.omega_lpf_4_xZeq_const_part;

    chixu.smo_gain_scale = CHI_XU_SMO_GAIN_SCALE;
}
REAL get_theta_d_from_xZeq(REAL xZeq_a, REAL xZeq_b){
    REAL temp = -atan2( xZeq_a, xZeq_b) + M_PI*0.5;
    if(temp>0){
        temp = temp - M_PI;
    }else{
        temp = temp + M_PI;
    }
    return temp;
}
void rhf_ChiXu2009_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    #define xIab(X) x[0+X]
    #define xZeq(X) x[2+X]
    #define xOmg     x[4]
    #define xTheta_d x[5]

    /* Know Signals */
    // #define MOTOR (*CTRL.motor)

    /* Output Error = \tilde i_q (scalar) */
    chixu.output_error[0] = IS(0) - xIab(0);
    chixu.output_error[1] = IS(1) - xIab(1);

    /* Sliding Mode Switching Term*/
    // TODO: need to change sigmoid to saturation function 
    chixu.xEmf_raw[0] = chixu.smo_gain * sm_sigmoid(chixu.output_error[0], chixu.sigmoid_coeff);
    chixu.xEmf_raw[1] = chixu.smo_gain * sm_sigmoid(chixu.output_error[1], chixu.sigmoid_coeff);

    // xZeq
    fx[2] = chixu.omega_lpf_4_xZeq * (chixu.xEmf_raw[0] - xZeq(0));
    fx[3] = chixu.omega_lpf_4_xZeq * (chixu.xEmf_raw[1] - xZeq(1));

    //
    chixu.theta_d = get_theta_d_from_xZeq(chixu.xZeq[0], chixu.xZeq[1]);

    /* State Observer */
    // xIab
    fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * xIab(0) + chixu.ell4xZeq * xZeq(0) + chixu.xEmf_raw[0]);
    fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * xIab(1) + chixu.ell4xZeq * xZeq(1) + chixu.xEmf_raw[1]);

    // PLL for xOmg
    if(TRUE){
        // xOmg
        fx[4] = (chixu.PLL_KI        * sin(chixu.theta_d - xTheta_d));
        // xTheta_d
        fx[5] = (xOmg + chixu.PLL_KP * sin(chixu.theta_d - xTheta_d));
    }else{
        // xOmg
        fx[4] = (chixu.PLL_KI        * difference_between_two_angles(chixu.theta_d, xTheta_d));
        // xTheta_d
        fx[5] = (xOmg + chixu.PLL_KP * difference_between_two_angles(chixu.theta_d, xTheta_d));
    }


    #undef xIab
    #undef xZeq
    #undef xOmg
    #undef xTheta_d
}
void Main_ChiXu2009_emfSMO(){

    // #define OMEGA_SYNC_USED CTRL.I->cmd_omg_elec
    // #define OMEGA_SYNC_USED CTRL.I->omg_elec
    #define OMEGA_SYNC_USED chixu.xOmg

    #if CHI_XU_USE_CONSTANT_SMO_GAIN == FALSE
        /* Time-varying gains */
        chixu.smo_gain = chixu.smo_gain_scale * fabs(OMEGA_SYNC_USED) * MOTOR.KE; // 根据Qiao.Xia 2013的稳定性证明增益要比反电势大。
        if(fabs(OMEGA_SYNC_USED)<5){ // [rad/s]
            // 转速太低以后，就不要再减小滑模增益了
            chixu.smo_gain = chixu.smo_gain_scale * 5 * MOTOR.KE;
        }
    #endif

    /* ell for xZeq */
    if(fabs(CTRL.I->cmd_speed_rpm)>180){
        chixu.ell4xZeq = 1.0;
    }else{
        chixu.ell4xZeq = -0.5;
    }

    /* For the sake of simplicity, a first-order LPF is used in (3).
        It is noticed that the time constant τc of LPF should be designed
        properly according to the fundamental frequency of phase
        currents of PMSM */
    #if CHI_XU_USE_CONSTANT_LPF_POLE == FALSE
        chixu.omega_lpf_4_xZeq = 0.05*fabs(OMEGA_SYNC_USED) + chixu.omega_lpf_4_xZeq_const_part;
    #endif

    general_6states_rk4_solver(&rhf_ChiXu2009_Dynamics, CTRL.timebase, chixu.x, CL_TS);

    /* Unpack x (for the best) */
    chixu.xIab[0]  = chixu.x[0];
    chixu.xIab[1]  = chixu.x[1];
    chixu.xZeq[0]  = chixu.x[2];
    chixu.xZeq[1]  = chixu.x[3];
    chixu.xOmg     = chixu.x[4];
    if(chixu.x[5]>M_PI){
        chixu.x[5] -= 2*M_PI;
    }else if(chixu.x[5]<-M_PI){
        chixu.x[5] += 2*M_PI;
    }
    chixu.xTheta_d = chixu.x[5];
    if(FALSE){
        // use xZeq
        chixu.theta_d = get_theta_d_from_xZeq(chixu.xZeq[0], chixu.xZeq[1]);
    }else{
        // use PLL output
        chixu.theta_d = chixu.xTheta_d; 
    }

    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */
}
#endif


/********************************************/
/* Park.Sul 2014 FADO in replace of CM 
 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Park_Sul
void init_parksul2014(){
    parksul.T2S_1_KP = PARK_SUL_T2S_1_KP;
    parksul.T2S_1_KI = PARK_SUL_T2S_1_KI;
    parksul.T2S_2_KP = PARK_SUL_T2S_2_KP;
    parksul.T2S_2_KI = PARK_SUL_T2S_2_KI;

    parksul.CM_KP = PARK_SUL_CM_KP;
    parksul.CM_KI = PARK_SUL_CM_KI;

    parksul.limiter_KE = 1.15 * MOTOR.KE;

    parksul.x[0] = MOTOR.KE;
    parksul.x[2] = MOTOR.KE;
    parksul.xPsi2[0] = MOTOR.KE;
}
void rhf_parksul2014_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    #define xPsi1(X)    x[0+X]
    #define xHatPsi1(X) x[2+X]
    #define xD(X)       x[4+X]
    #define xT2S_1(X)   x[6+X]
    #define xT2S_2(X)   x[8+X]
    // #define xTheta_d    x[7]
    // #define xOmg        x[8]

    /* Know Signals */
    parksul.omega_f = x[8]; // In addition, it is evident in Fig. 3 that all the integrations in the proposed observer are stopped when ω f is zero, which coincides with the natural property of stator flux at standstill.

    /* Pre-calculation */
    parksul.internal_error[0] = xPsi1(0) - xHatPsi1(0);
    parksul.internal_error[1] = xPsi1(1) - xHatPsi1(1);

    parksul.xPsi2[0] = xPsi1(0) - MOTOR.Lq*IS(0);
    parksul.xPsi2[1] = xPsi1(1) - MOTOR.Lq*IS(1);
    parksul.theta_d = atan2(parksul.xPsi2[1], parksul.xPsi2[0]);

    /* The amplitude limiter */
    parksul.xPsi2_Amplitude = sqrt(parksul.xPsi2[0]*parksul.xPsi2[0] + parksul.xPsi2[1]*parksul.xPsi2[1]);
    if(parksul.xPsi2_Amplitude > parksul.limiter_KE){
        REAL temp = 1.0/parksul.xPsi2_Amplitude;
        parksul.xPsi2_Limited[0] = parksul.limiter_KE*parksul.xPsi2[0]*temp;
        parksul.xPsi2_Limited[1] = parksul.limiter_KE*parksul.xPsi2[1]*temp;
    }else{
        parksul.xPsi2_Limited[0] = parksul.xPsi2[0];
        parksul.xPsi2_Limited[1] = parksul.xPsi2[1];
    }

    /* Modified: Add output error (CM correction) to Park.Sul2014 */
    #define TRY_CM_VM_FUSION_IN_REPLACE_OF_K_DF
    #ifdef  TRY_CM_VM_FUSION_IN_REPLACE_OF_K_DF
        /* State: Cosine and Sine of the flux angle */

        // 我们已经有角度了，直接求三角函数就可以了
        // REAL temp, cosT, sinT;
        // if(parksul.active_flux_ampl!=0){
        //     temp = 1.0/parksul.xPsi2_Amplitude;
        //     cosT = parksul.xPsi2[0] * temp;
        //     sinT = parksul.xPsi2[1] * temp;
        // }else{
        //     cosT = 1.0;
        //     sinT = 0.0;
        // }

        REAL cosT = cos(parksul.theta_d);
        REAL sinT = sin(parksul.theta_d);

        /* Input: Current at dq frame and KActive */
        REAL iDQ_at_current_step[2];
        iDQ_at_current_step[0] = AB2M(IS(0), IS(1), cosT, sinT);
        iDQ_at_current_step[1] = AB2T(IS(0), IS(1), cosT, sinT);
        REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_at_current_step[0];

        /* Output: Flux mismatch to KActive (CM) */
        parksul.output_error[0] = KActive*cosT - parksul.xPsi2[0];
        parksul.output_error[1] = KActive*sinT - parksul.xPsi2[1];
    #endif

    /* Flux Estimator */
    REAL emf[2];
    emf[0] = US(0) - MOTOR.R*IS(0);
    emf[1] = US(1) - MOTOR.R*IS(1);
    // xPsi1
    fx[0] = emf[0] \
        /*k_df*/- parksul.k_df*xD(0) \
        /*k_af*/- parksul.k_af*(parksul.xPsi2[0] - parksul.xPsi2_Limited[0]) \
        /*CM-I*/+ x[10] \
        /*CM-P*/+ parksul.CM_KP * parksul.output_error[0];
    fx[10] =      parksul.CM_KI * parksul.output_error[0];
    fx[1] = emf[1] \
        /*k_df*/- parksul.k_df*xD(1) \
        /*k_af*/- parksul.k_af*(parksul.xPsi2[1] - parksul.xPsi2_Limited[1]) \
        /*CM-I*/+ x[11] \
        /*CM-P*/+ parksul.CM_KP * parksul.output_error[1];
    fx[11] =      parksul.CM_KI * parksul.output_error[1];

    /* State: disturbance/offset estimated by an integrator */
    // xHatPsi1
    fx[2] = -parksul.omega_f * (xPsi1(1)-xD(1)) + 2*fabs(parksul.omega_f)*parksul.internal_error[0];
    fx[3] = +parksul.omega_f * (xPsi1(0)-xD(0)) + 2*fabs(parksul.omega_f)*parksul.internal_error[1];
    // xD
    fx[4] = -parksul.omega_f * parksul.internal_error[1];
    fx[5] = +parksul.omega_f * parksul.internal_error[0];

    /* T2S-1 for position */
    parksul.t2s_1_sin_error = sin(parksul.theta_d - x[7]);
    fx[6] =        parksul.T2S_1_KI * parksul.t2s_1_sin_error;
    fx[7] = x[6] + parksul.T2S_1_KP * parksul.t2s_1_sin_error;

    /* T2S-2 for speed */
    parksul.t2s_2_sin_error = sin(parksul.theta_d - x[9]);
    fx[8] =        parksul.T2S_2_KI * parksul.t2s_2_sin_error;
    fx[9] = x[8] + parksul.T2S_2_KP * parksul.t2s_2_sin_error;

    #undef xPsi1
    #undef xHatPsi1
    #undef xD
    #undef xT2S_1
    #undef xT2S_2
}
void Main_parksul2014_FADO(){

    /* Time-varying gains */
    if(fabs(parksul.xOmg)        *MOTOR.npp_inv*ONE_OVER_2PI<1.5){ // [Hz]
    // if(fabs(CTRL.I->cmd_omg_elec)*MOTOR.npp_inv*ONE_OVER_2PI<1.5){ // [Hz]
        parksul.k_df = 0.0;
        parksul.k_af = 2*M_PI*100;
        parksul.limiter_Flag = TRUE;
    }else{
        parksul.k_df = 0.50;
        parksul.k_af = 0.0;
        parksul.limiter_Flag = FALSE;
    }

    // debug: turn off k_df/k_af
    parksul.k_df = 0.0;
    // parksul.k_af = 0.0;
    // parksul.CM_KP = 0.0;
    // parksul.CM_KI = 0.0;

    general_10states_rk4_solver(&rhf_parksul2014_Dynamics, CTRL.timebase, parksul.x, CL_TS);

    /* Unpack x (for the best) */
    if(parksul.x[7]>M_PI){
        parksul.x[7]     -= 2*M_PI;
    }else if(parksul.x[7]<-M_PI){
        parksul.x[7]     += 2*M_PI;
    }
    if(parksul.x[9]>M_PI){
        parksul.x[9]     -= 2*M_PI;
    }else if(parksul.x[9]<-M_PI){
        parksul.x[9]     += 2*M_PI;
    }
    parksul.xPsi1[0]     = parksul.x[0];
    parksul.xPsi1[1]     = parksul.x[1];
    parksul.xHatPsi1[0]  = parksul.x[2]; // aux
    parksul.xHatPsi1[1]  = parksul.x[3]; // aux
    parksul.xD[0]        = parksul.x[4];
    parksul.xD[1]        = parksul.x[5];
    parksul.xT2S_1[0]    = parksul.x[6]; // speed
    parksul.xT2S_1[1]    = parksul.x[7]; // position
    parksul.xT2S_2[0]    = parksul.x[8]; // speed
    parksul.xT2S_2[1]    = parksul.x[9]; // position
    parksul.xCM_state[0] = parksul.x[10]; // CM-correction-alpha
    parksul.xCM_state[1] = parksul.x[11]; // CM-correction-beta

    parksul.xTheta_d     = parksul.x[7]; // parksul2014 uses this instead of atan2(parksul.xPsi2[1], parksul.xPsi2[0]);
    parksul.xOmg         = parksul.x[8];

    /* Post-observer calculations */
    parksul.xPsi2[0] = parksul.xPsi1[0] - MOTOR.Lq*IS_C(0);
    parksul.xPsi2[1] = parksul.xPsi1[1] - MOTOR.Lq*IS_C(1);
    if(TRUE){
        parksul.theta_d = atan2(parksul.xPsi2[1], parksul.xPsi2[0]);
    }else{
        // Park Sul 2014 uses this
        parksul.theta_d = parksul.xT2S_1[1]; // x[7]
    }
}
#endif



/* DOB Stationary Voltage that is valid for SPMSM only */
// void stationary_voltage_DOB(){
//     // CTRL.I->iab[0]
//     // CTRL.I->iab[1]

//     CTRL.inv->iab_lpf[0] = _lpf(CTRL.I->iab[0], CTRL.inv->iab_lpf[0], CTRL.inv->filter_pole);
//     CTRL.inv->iab_lpf[1] = _lpf(CTRL.I->iab[1], CTRL.inv->iab_lpf[1], CTRL.inv->filter_pole);

//     CTRL.inv->iab_hpf[0] = CTRL.I->iab[0] - CTRL.inv->iab_lpf[0];
//     CTRL.inv->iab_hpf[1] = CTRL.I->iab[1] - CTRL.inv->iab_lpf[1];

//     CTRL.inv->uab_DOB[0] = - CTRL.motor->R * CTRL.inv->iab_lpf[0] - CTRL.motor->Ld * CTRL.inv->iab_lpf[0];
//     CTRL.inv->uab_DOB[1] = - CTRL.motor->R * CTRL.inv->iab_lpf[1] - CTRL.motor->Ld * CTRL.inv->iab_lpf[1];

//     // add back emf
//     // CTRL.inv->uab_DOB[0] += CTRL.I->omg_elec * CTRL.motor->KE * -sin(CTRL.I->theta_d_elec);
//     // CTRL.inv->uab_DOB[1] += CTRL.I->omg_elec * CTRL.motor->KE *  cos(CTRL.I->theta_d_elec);
// }




/********************************************/
/* COMMON *
 ********************************************/
void init_rk4(){
    int i;
    for(i=0; i<2; ++i){
        rk4.us[i] = 0;
        rk4.is[i] = 0;
        // rk4.us_curr[i] = 0;
        rk4.is_curr[i] = 0;
        rk4.us_prev[i] = 0;
        rk4.is_prev[i] = 0;
        rk4.is_lpf[i]  = 0;
        rk4.is_hpf[i]  = 0;
        rk4.is_bpf[i]  = 0;

        rk4.current_lpf_register[i] = 0;
        rk4.current_hpf_register[i] = 0;
        rk4.current_bpf_register1[i] = 0;
        rk4.current_bpf_register2[i] = 0;
    }
}
void pmsm_observers(){
    // stationary_voltage_DOB();

    CTRL.motor->KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * CTRL.I->idq_cmd[0];

    #if PC_SIMULATION
        /* Cascaded Flux Estimator */
        test_flux_estimators();
        // nsoaf.theta_d = AFE_USED.theta_d;

        /* Speed and Position Estimator */
        // harnefors_scvm();
        // cjh_eemfao();
        // cjh_eemfhgo_farza09();
        Main_nsoaf_chen2020();
        // Main_QiaoXia2013_emfSMO();
        Main_ChiXu2009_emfSMO();
        Main_parksul2014_FADO();
    #else
        /* 资源有限 */
        #if SELECT_ALGORITHM == ALG_NSOAF
            Main_the_active_flux_estimator();
            MainFE_HuWu_1998();
            Main_nsoaf_chen2020();
        #elif SELECT_ALGORITHM == ALG_Farza_2009
            Main_cjh_eemfhgo_farza09();
        #elif SELECT_ALGORITHM == ALG_CJH_EEMF
            Main_cjh_eemfao();
        #elif SELECT_ALGORITHM == ALG_Harnefors_2006
            Main_harnefors_scvm();
        #elif SELECT_ALGORITHM == ALG_Qiao_Xia
            Main_QiaoXia2013_emfSMO();
        #elif SELECT_ALGORITHM == ALG_Chi_Xu
            Main_ChiXu2009_emfSMO();
        #elif SELECT_ALGORITHM == ALG_Park_Sul
            Main_parksul2014_FADO();
        #endif
    #endif

    G.omg_elec = ELECTRICAL_SPEED_FEEDBACK;
    G.theta_d  = ELECTRICAL_POSITION_FEEDBACK;

    // if(G.Select_algorithm == 1){
    //     G.omg_elec = nsoaf.xOmg;
    //     G.theta_d  = AFE_USED.theta_d;
    // }else if(G.Select_algorithm == 2){
    //     G.omg_elec = parksul.xOmg;
    //     G.theta_d  = parksul.theta_d;
    // }else if(G.Select_algorithm == 3){
    //     G.omg_elec = chixu.xOmg;
    //     G.theta_d  = chixu.theta_d;
    // }else if(G.Select_algorithm == 4){
    //     G.omg_elec = qiaoxia.xOmg;
    //     G.theta_d  = qiaoxia.theta_d;
    // }


    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    // US_P(0) = US_C(0); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    // US_P(1) = US_C(1); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}
void init_pmsm_observers(){
    // RK4
    init_rk4();     // 龙格库塔法结构体初始化

    // FE
    init_FE();


    #if PC_SIMULATION
        // OBSV
        init_nsoaf();
        init_hgo4eemf();
        init_cjheemf(); // cjheemf 结构体初始化
        init_harnefors(); // harnefors结构体初始化
        init_QiaoXia2013();
        init_ChiXu2009();
        init_parksul2014();
    #else
        /* 资源有限 */
        #if SELECT_ALGORITHM == ALG_NSOAF
            init_nsoaf();
        #elif SELECT_ALGORITHM == ALG_Farza_2009
            init_hgo4eemf();
        #elif SELECT_ALGORITHM == ALG_CJH_EEMF
            init_cjheemf(); // cjheemf 结构体初始化
        #elif SELECT_ALGORITHM == ALG_Harnefors_2006
            init_harnefors(); // harnefors结构体初始化
        #elif SELECT_ALGORITHM == ALG_Qiao_Xia
            init_QiaoXia2013();
        #elif SELECT_ALGORITHM == ALG_Chi_Xu
            init_ChiXu2009();
        #elif SELECT_ALGORITHM == ALG_Park_Sul
            init_parksul2014();
        #endif
    #endif
}
#endif
