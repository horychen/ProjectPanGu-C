#include "ACMSim.h"

#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

/********************************************
 * Natural Speed Observer for IPMSM with Active Flux Concept (Chen 2020)
 ********************************************/
#define NS 3
REAL one_over_six = 1.0/6.0;
typedef void (*pointer_to_dynamics)(REAL t, REAL *x, REAL *fx);
void general_NS_states_rk4_solver(pointer_to_dynamics fp, REAL t, REAL *x, REAL hs){
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;

    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    (*fp)(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }

    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<NS;++i){
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }

    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<NS;++i){
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    (*fp)(t, xk, fx); // timer.t+hs,
    for(i=0;i<NS;++i){
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;
    }
}
/* The 3rd-order dynamic system */
void rhf_NSOAF_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xIq  = x[0];
    REAL xOmg = x[1];
    REAL xTL  = x[2];

    /* Know Signals */
    #define MOTOR (*CTRL.motor)
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS(0), IS(1), nsoaf.cosT, nsoaf.sinT);
    iDQ_now[1]  = AB2T(IS(0), IS(1), nsoaf.cosT, nsoaf.sinT);
    REAL uQ_now = AB2T(US(0), US(1), nsoaf.cosT, nsoaf.sinT);
    nsoaf.q_axis_voltage = uQ_now;

    /* Filter */
    static REAL uQ_now_filtered = 0.0;
    uQ_now_filtered = _lpf(uQ_now, uQ_now_filtered, 30); // 越大滤越狠
    // uQ_now_filtered = uQ_now;

    /* Output Error = \tilde i_q (scalar) */
    nsoaf.output_error = iDQ_now[1] - xIq;
    #ifdef NSOAF_SPMSM
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
    fx[0] = MOTOR.Lq_inv * (uQ_now - MOTOR.R * xIq - xOmg*(MOTOR.KE + MOTOR.Ld*iDQ_now[0]));
    // xOmg
    REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * KActive * xIq;
    // xTL = ACM.TLoad; // DEBUG
    fx[1] = MOTOR.Js_inv * MOTOR.npp * (nsoaf.xTem - xTL - nsoaf.KP*nsoaf.active_power_error);
    /* Parameter Adaptation */
    // xTL
    fx[2] = nsoaf.KI * nsoaf.active_power_error;
}
/* Main Observer */
void nsoaf_chen2020(){

    /* OBSERVATION */

    /* Unpack States */
    nsoaf.xIq      = nsoaf.xBest[0];
    nsoaf.xOmg     = nsoaf.xBest[1];
    nsoaf.xTL      = nsoaf.xBest[2];

    /* Know Signals */
    #define MOTOR (*CTRL.motor)
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS_P(0), IS_P(1), nsoaf.cosT, nsoaf.sinT);
    iDQ_now[1]  = AB2T(IS_P(0), IS_P(1), nsoaf.cosT, nsoaf.sinT);
    REAL uQ_now = AB2T(US_P(0), US_P(1), nsoaf.cosT, nsoaf.sinT);
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
        //     general_NS_states_rk4_solver(&rhf_NSOAF_Dynamics, CTRL.timebase, (nsoaf.x+j*NS), CL_TS);
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

    general_NS_states_rk4_solver(&rhf_NSOAF_Dynamics, CTRL.timebase, nsoaf.xBest, CL_TS);

    /* Unpack x (for the best) */
    // nsoaf.xIq      = nsoaf.xBest[0];
    // nsoaf.xOmg     = nsoaf.xBest[1];
    // nsoaf.xTL      = nsoaf.xBest[2];
    /* Post-observer calculations */
    ;
    
    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    // US_P(0) = US_C(0); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    // US_P(1) = US_C(1); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}
void nsoaf_init(){
    nsoaf.KP = NSOAF_TL_P;
    nsoaf.KI = NSOAF_TL_I;
    nsoaf.KD = NSOAF_TL_D;

    nsoaf.afest_states[0] = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // TODO：这里假设了初始位置是alpha轴对着d轴的
    nsoaf.afest_states[1] = 0.0;

    nsoaf.ActiveFlux_KP = OUTPUT_ERROR_CLEST_GAIN_KP;
    nsoaf.ActiveFlux_KI = OUTPUT_ERROR_CLEST_GAIN_KI;
}
#undef NS


/********************************************
 * EEMF-Speed-Adaptive-High-Gain-Observer-Farza-2009
 ********************************************/
/* The 10th-order dynamic system */
#define NS 10
void rhs_func_hgoeemf(REAL *increment_n, REAL *xPsi, REAL *xEemf, 
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

    /* Theoritically speaking, rhs_func should be time-varing like rhs_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhs_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhs_func_hgoeemf( increment_1, hgo4eemf.xPsi, hgo4eemf.xEemf, 
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
    rhs_func_hgoeemf( increment_2, p_x_temp+0, p_x_temp+2, 
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
    rhs_func_hgoeemf( increment_3, p_x_temp+0, p_x_temp+2, 
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
    rhs_func_hgoeemf( increment_4, p_x_temp+0, p_x_temp+2, 
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
void cjh_eemfhgo_farza09(){

    /* OBSERVATION */
    hgo4eemf_dedicated_rk4_solver(1*CL_TS);

    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    // US_P(0) = US_C(0); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    // US_P(1) = US_C(1); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}
void hgo4eemf_init(){
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



/********************************************
 * EEMF-Error-Dynamics-based-AO-Design
 ********************************************/
/* The 13th-order dynamic system */
#define NS 13
void rhs_func_eemfaod(REAL *increment_n, REAL *xPsi, REAL *xChi, REAL xOmg, 
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

    /* Theoritically speaking, rhs_func should be time-varing like rhs_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhs_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhs_func_eemfaod( increment_1, cjheemf.xPsi, cjheemf.xChi, cjheemf.xOmg,
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
    rhs_func_eemfaod( increment_2, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
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
    rhs_func_eemfaod( increment_3, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
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
    rhs_func_eemfaod( increment_4, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
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
void cjh_eemfao(){

    /* OBSERVATION */
    eemf_ao_dedicated_rk4_solver(1*CL_TS);

    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    // US_P(0) = US_C(0); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    // US_P(1) = US_C(1); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}
void cjheemf_init(){
    cjheemf.k1        = CJH_EEMF_K1;
    cjheemf.k2        = CJH_EEMF_K2;
    cjheemf.gamma_omg = CJH_EEMF_GAMMA_OMEGA;
}
#undef NS



/********************************************
 * Harnefor 2006
 ********************************************/
void harnefors_init(){
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
void harnefors_scvm(){

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




/* Active Flux Estimator */
#if PC_SIMULATION == TRUE
    #define OFFSET_VOLTAGE_ALPHA (0*0.05*-0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  (0*0.05*+0.1 *(CTRL.timebase>3)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
#else
    #define OFFSET_VOLTAGE_ALPHA 0
    #define OFFSET_VOLTAGE_BETA  0
#endif
typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 4
    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;

    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    (*fp)(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }

    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<NS;++i){
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }

    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<NS;++i){
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    (*fp)(t, xk, fx); // timer.t+hs,
    for(i=0;i<NS;++i){
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;
    }
    #undef NS
}
/* 8. C. Output Error Closed-loop Flux Estimator */
void rhf_ActiveFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
    // x[0], x[1]: stator flux in ab frame
    // x[2]: \psi_{d\mu}
    // x[3], x[4]: integral action compensating offset voltage

    #define MOTOR (*CTRL.motor)

    nsoaf.active_flux_ab[0] = x[0] - MOTOR.Lq * IS(0);
    nsoaf.active_flux_ab[1] = x[1] - MOTOR.Lq * IS(1);
    nsoaf.active_flux_ampl = sqrt(nsoaf.active_flux_ab[0]*nsoaf.active_flux_ab[0] + nsoaf.active_flux_ab[1]*nsoaf.active_flux_ab[1]);
    REAL active_flux_ampl_inv=0.0;
    if(nsoaf.active_flux_ampl!=0){
        active_flux_ampl_inv = 1.0/nsoaf.active_flux_ampl;
    } 
    REAL cosT = nsoaf.active_flux_ab[0] * active_flux_ampl_inv;
    REAL sinT = nsoaf.active_flux_ab[1] * active_flux_ampl_inv;

    REAL iDQ_at_current_step[2];
    iDQ_at_current_step[0] = AB2M(IS(0), IS(1), cosT, sinT);
    iDQ_at_current_step[1] = AB2T(IS(0), IS(1), cosT, sinT);

    REAL current_estimate[2];
    REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_at_current_step[0];
    current_estimate[0] = MOTOR.Lq_inv * (x[0] - KActive*cosT);
    current_estimate[1] = MOTOR.Lq_inv * (x[1] - KActive*sinT);

    REAL current_error[2];
    current_error[0] = IS(0) - current_estimate[0];
    current_error[1] = IS(1) - current_estimate[1];

    REAL emf[2];
    emf[0] = US(0) - MOTOR.R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA;
    emf[1] = US(1) - MOTOR.R*IS(1) + 1*OFFSET_VOLTAGE_BETA ;
    fx[0] = emf[0] \
        /*P*/+ nsoaf.ActiveFlux_KP * current_error[0] \
        /*I*/+ x[2];
    fx[1] = emf[1] \
        /*P*/+ nsoaf.ActiveFlux_KP * current_error[1] \
        /*I*/+ x[3];

    fx[2] = nsoaf.ActiveFlux_KI * current_error[0];
    fx[3] = nsoaf.ActiveFlux_KI * current_error[1];
}
void the_active_flux_estimator(){
    /* Proposed closed loop estimator AB frame + ODE4 */
    // stator flux and integral states update
    general_4states_rk4_solver(&rhf_ActiveFluxEstimator_Dynamics, CTRL.timebase, nsoaf.afest_states, CL_TS);
    // Unpack x
    nsoaf.psi_1[0]    = nsoaf.afest_states[0];
    nsoaf.psi_1[1]    = nsoaf.afest_states[1];
    nsoaf.u_offset[0] = nsoaf.afest_states[2];
    nsoaf.u_offset[1] = nsoaf.afest_states[3];

    // active flux
    nsoaf.active_flux_ab[0] = nsoaf.afest_states[0] - CTRL.motor->Lq * IS(0);
    nsoaf.active_flux_ab[1] = nsoaf.afest_states[1] - CTRL.motor->Lq * IS(1);
    nsoaf.active_flux_ampl = sqrt(nsoaf.active_flux_ab[0]*nsoaf.active_flux_ab[0] + nsoaf.active_flux_ab[1]*nsoaf.active_flux_ab[1]);
    REAL active_flux_ampl_inv=0.0;
    if(nsoaf.active_flux_ampl!=0){
        active_flux_ampl_inv = 1.0/nsoaf.active_flux_ampl;
    } 
    nsoaf.cosT = nsoaf.active_flux_ab[0] * active_flux_ampl_inv;
    nsoaf.sinT = nsoaf.active_flux_ab[1] * active_flux_ampl_inv;

    nsoaf.theta_d = atan2(nsoaf.active_flux_ab[1], nsoaf.active_flux_ab[0]);
}



/********************************************
 * COMMON *
 ********************************************/
void rk4_init(){
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
    /* Cascaded Flux Estimator */
    the_active_flux_estimator();

    /* Speed and Position Estimator */
    // harnefors_scvm();
    // cjh_eemfao();
    // cjh_eemfhgo_farza09();
    nsoaf_chen2020();
}
void pmsm_observers_init(){
    harnefors_init();
    cjheemf_init();
    hgo4eemf_init();
    nsoaf_init();
}
#endif
