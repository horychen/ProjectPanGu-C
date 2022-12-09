#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

#if PC_SIMULATION
    #define OFFSET_VOLTAGE_ALPHA (1*+0.1 *(CTRL.timebase>20)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  (0*+0.1 *(CTRL.timebase>4)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
#else
    #define OFFSET_VOLTAGE_ALPHA (0.0)
    #define OFFSET_VOLTAGE_BETA  (0.0)
#endif

/********************************************
 * Marino 2005 Observer Part
 ********************************************/

/* The fourth-order dynamic system by Marino2005@Wiley */
void rhs_func_marino2005(REAL *increment_n, REAL xRho, REAL xTL, REAL xAlpha, REAL xOmg, REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/

    // time-varying quantities
    CTRL.S->cosT = cos(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    CTRL.S->sinT = sin(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    CTRL.I->idq[0] = AB2M(IS(0), IS(1), CTRL.S->cosT, CTRL.S->sinT);
    CTRL.I->idq[1] = AB2T(IS(0), IS(1), CTRL.S->cosT, CTRL.S->sinT);

    // f = \dot x = the time derivative
    REAL f[4];
    // xRho
    f[0] = xOmg + xAlpha*CTRL.motor->Lmu*CTRL.I->idq_cmd[1]*CTRL.I->cmd_psi_inv;
    // xTL
    f[1] = - marino.gamma_inv * CTRL.motor->Js * CTRL.I->cmd_psi * marino.e_psi_Qmu;
    // xAlpha
    f[2] = marino.delta_inv * (   xAlpha_LAW_TERM_D * marino.e_psi_Dmu*(CTRL.motor->Lmu*CTRL.I->idq_cmd[0] - CTRL.I->cmd_psi) \
                                + xAlpha_LAW_TERM_Q * marino.e_psi_Qmu* CTRL.motor->Lmu*CTRL.I->idq_cmd[1]);
    // xOmg
    REAL xTem = CLARKE_TRANS_TORQUE_GAIN*CTRL.motor->npp*( marino.psi_Dmu*CTRL.I->idq[1] - marino.psi_Qmu*CTRL.I->idq[0] );
    f[3] = CTRL.motor->npp*CTRL.motor->Js_inv*(xTem - xTL) + 2*marino.lambda_inv*CTRL.I->cmd_psi*marino.e_psi_Qmu;

    // bad
    // f[0] = xOmg + xAlpha*CTRL.motor->Lmu*CTRL.I->idq[1]*CTRL.I->cmd_psi_inv;

    // REAL xTem = CLARKE_TRANS_TORQUE_GAIN*CTRL.motor->npp*( marino.psi_Dmu*ACM.iTs - marino.psi_Qmu*ACM.iMs );
    // f[3] = CTRL.motor->npp*CTRL.motor->Js_inv*(ACM.Tem - ACM.TLoad);
    // f[3] = CTRL.motor->npp*CTRL.motor->Js_inv*(CLARKE_TRANS_TORQUE_GAIN*ACM.npp*(ACM.x[1]*ACM.x[2]-ACM.x[0]*ACM.x[3] - ACM.TLoad));

    // f[3] = ACM.x_dot[4]; 

    increment_n[0] = ( f[0] )*hs;
    increment_n[1] = ( f[1] )*hs;
    increment_n[2] = ( f[2] )*hs;
    increment_n[3] = ( f[3] )*hs;
}
void marino05_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[4];
    static REAL increment_2[4];
    static REAL increment_3[4];
    static REAL increment_4[4];
    static REAL x_temp[4];
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
    rhs_func_marino2005( increment_1, marino.xRho, marino.xTL, marino.xAlpha, marino.xOmg, hs); 
    x_temp[0]  = marino.xRho   + increment_1[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_1[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_1[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_1[3]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhs_func_marino2005( increment_2, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_2[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_2[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_2[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_2[3]*0.5;

    // time instant t+hs/2
    rhs_func_marino2005( increment_3, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_3[0];
    x_temp[1]  = marino.xTL    + increment_3[1];
    x_temp[2]  = marino.xAlpha + increment_3[2];
    x_temp[3]  = marino.xOmg   + increment_3[3];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhs_func_marino2005( increment_4, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    marino.xRho        += (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    marino.xTL         += (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    marino.xAlpha      += (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    marino.xOmg        += (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3

    // Also get derivatives:
    CTRL.S->omega_syn = marino.xOmg + marino.xAlpha*CTRL.motor->Lmu*CTRL.I->idq_cmd[1]*CTRL.I->cmd_psi_inv;
    marino.deriv_xTL    = (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xAlpha = (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xOmg   = (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667 * CL_TS_INVERSE;



    // Projection Algorithm
    // xRho   \in [-M_PI, M_PI]
    if(marino.xRho > M_PI){
        marino.xRho -= 2*M_PI;
    }else if(marino.xRho < -M_PI){
        marino.xRho += 2*M_PI; // 反转！
    }
    // xTL    \in [-xTL_Max, xTL_Max]
    ;
    // xAlpha \in [-xAlpha_min, xAlpha_Max]
    if(marino.xAlpha > marino.xAlpha_Max){
        marino.xAlpha = marino.xAlpha_Max;
    }else if(marino.xAlpha < marino.xAlpha_min){
        marino.xAlpha = marino.xAlpha_min;
    }
}

#define TAU_OFFSET_INVERSE 5 // 0.1滤得更狠，但是高速后不能有效减小误差了 // 0.5
#define K_VM (0.5*0)
#if PC_SIMULATION
    /* Flux estimator that is only valid in simulation */
    void simulation_only_flux_estimator(){
        // Flux Measurements
        simvm.psi_D2 = ACM.psi_Dmu;
        simvm.psi_Q2 = ACM.psi_Qmu;

        // CM
        FE.fme.psi_DQ2[0] += CL_TS * (-CTRL.motor->alpha*FE.fme.psi_DQ2[0] + CTRL.motor->Rreq*CTRL.I->idq[0]);
        FE.fme.psi_DQ1[0] = FE.fme.psi_DQ2[0] + CTRL.motor->Lsigma*CTRL.I->idq[0];


        // Flux Estimator (Open Loop Voltage Model in αβ frame + ODE1)
        if(TRUE){
            // 
            simvm.emf[0] = CTRL.O->uab_cmd[0] - CTRL.motor->R*CTRL.I->iab[0] + 1*OFFSET_VOLTAGE_ALPHA;
            simvm.emf[1] = CTRL.O->uab_cmd[1] - CTRL.motor->R*CTRL.I->iab[1] + 1*OFFSET_VOLTAGE_BETA;
            simvm.psi_1[0] += CL_TS * ( simvm.emf[0] - 1*K_VM*(simvm.psi_1[0] - FE.fme.psi_DQ1[0]* CTRL.S->cosT) );
            simvm.psi_1[1] += CL_TS * ( simvm.emf[1] - 1*K_VM*(simvm.psi_1[1] - FE.fme.psi_DQ1[0]* CTRL.S->sinT) );
            simvm.psi_2[0] = simvm.psi_1[0] - CTRL.motor->Lsigma*CTRL.I->iab[0];
            simvm.psi_2[1] = simvm.psi_1[1] - CTRL.motor->Lsigma*CTRL.I->iab[1];
            simvm.psi_2_ampl = sqrt(simvm.psi_2[0]*simvm.psi_2[0]+simvm.psi_2[1]*simvm.psi_2[1]);

            REAL al = simvm.psi_2[0] - 0*(simvm.psi_1[0] - FE.fme.psi_DQ1[0]* CTRL.S->cosT);
            REAL be = simvm.psi_2[1]  - 0*(simvm.psi_1[1]  - FE.fme.psi_DQ1[0]* CTRL.S->sinT);
            simvm.psi_D2_ode1_v2 = AB2M(al, be, CTRL.S->cosT, CTRL.S->sinT);
            simvm.psi_Q2_ode1_v2 = AB2T(al, be, CTRL.S->cosT, CTRL.S->sinT);

            simvm.emf_DQ[0] = AB2M(simvm.emf[0], simvm.emf[1], CTRL.S->cosT, CTRL.S->sinT);
            simvm.emf_DQ[1] = AB2T(simvm.emf[0], simvm.emf[1], CTRL.S->cosT, CTRL.S->sinT);

        }else{
            static REAL temp_Alpha1 = 0.0;
            static REAL temp_Beta1 = 0.0;
            static REAL temp_Alpha2 = 0.0;
            static REAL temp_Beta2 = 0.0;

            // static REAL FE.exact.offset_voltage_compensation[2] = {0,0};
            FE.exact.offset_voltage_compensation[0] = _lpf(FE.exact.u_offset[0], FE.exact.offset_voltage_compensation[0], TAU_OFFSET_INVERSE);
            FE.exact.offset_voltage_compensation[1] = _lpf(FE.exact.u_offset[1], FE.exact.offset_voltage_compensation[1], TAU_OFFSET_INVERSE);

            temp_Alpha1 += CL_TS * ( CTRL.O->uab_cmd[0] - CTRL.motor->R*CTRL.I->iab[0] + 1*OFFSET_VOLTAGE_ALPHA - 0*FE.exact.u_offset[0] - 0.5*FE.exact.u_offset[0] - 1*FE.exact.offset_voltage_compensation[0])     + 0*GAIN_OHTANI * ( CTRL.I->cmd_psi_ABmu[0] - (temp_Alpha1-CTRL.motor->Lsigma*CTRL.I->iab[0]) );
            temp_Beta1  += CL_TS * ( CTRL.O->uab_cmd[1] - CTRL.motor->R*CTRL.I->iab[1] + 1*OFFSET_VOLTAGE_BETA  - 0*FE.exact.u_offset[1] - 0.5*FE.exact.u_offset[1] - 1*FE.exact.offset_voltage_compensation[1])     + 0*GAIN_OHTANI * ( CTRL.I->cmd_psi_ABmu[1] - (temp_Beta1 -CTRL.motor->Lsigma*CTRL.I->iab[1]) );
            temp_Alpha2 = temp_Alpha1 - CTRL.motor->Lsigma*CTRL.I->iab[0];
            temp_Beta2  = temp_Beta1  - CTRL.motor->Lsigma*CTRL.I->iab[1];

            REAL al = temp_Alpha2 - 0*0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]);
            REAL be = temp_Beta2  - 0*0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]);
            simvm.psi_D2_ode1_v2 = AB2M(al, be, CTRL.S->cosT, CTRL.S->sinT);
            simvm.psi_Q2_ode1_v2 = AB2T(al, be, CTRL.S->cosT, CTRL.S->sinT);
                // simvm.psi_D2_ode1_v2 = AB2M(temp_Alpha2, temp_Beta2, CTRL.S->cosT, CTRL.S->sinT);
                // simvm.psi_Q2_ode1_v2 = AB2T(temp_Alpha2, temp_Beta2, CTRL.S->cosT, CTRL.S->sinT);
        }

        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // REAL ampl, cosT, sinT;
        // ampl = sqrtf(temp_Alpha2*temp_Alpha2+temp_Beta2*temp_Beta2);
        // if(ampl != 0){
        //     cosT = temp_Alpha2 / ampl;
        //     sinT = temp_Beta2  / ampl;
        //     simvm.psi_D2_ode1_v2 = AB2M(temp_Alpha2, temp_Beta2, cosT, sinT);
        //     simvm.psi_Q2_ode1_v2 = AB2T(temp_Alpha2, temp_Beta2, cosT, sinT);
        // }

        // simvm.psi_D2_ode1 = simvm.psi_D2_ode1_v2;
        // simvm.psi_Q2_ode1 = simvm.psi_Q2_ode1_v2;
        // simvm.psi_D2_ode1 = ACM.psi_Dmu;
        // simvm.psi_Q2_ode1 = ACM.psi_Qmu;




        REAL deriv_psi_D1;
        REAL deriv_psi_Q1;
        if(FALSE){
            // With CM Correction
            deriv_psi_D1 = CTRL.O->udq_cmd[0] - CTRL.motor->R*CTRL.I->idq[0] + CTRL.S->omega_syn*simvm.psi_Q1_ode1 - 1*K_VM*(simvm.psi_D1_ode1 - FE.fme.psi_DQ1[0]);
            deriv_psi_Q1 = CTRL.O->udq_cmd[1] - CTRL.motor->R*CTRL.I->idq[1] - CTRL.S->omega_syn*simvm.psi_D1_ode1 - 0*K_VM*(simvm.psi_Q1_ode1 - 0);
        }else{
            // Flux Estimator (Open Loop Voltage Model in indirect oriented DQ frame + ODE1)
            deriv_psi_D1 = CTRL.O->udq_cmd[0] - CTRL.motor->R*CTRL.I->idq[0] + CTRL.S->omega_syn*simvm.psi_Q1_ode1;
            deriv_psi_Q1 = CTRL.O->udq_cmd[1] - CTRL.motor->R*CTRL.I->idq[1] - CTRL.S->omega_syn*simvm.psi_D1_ode1;
        }

        simvm.psi_D1_ode1 += CL_TS * (deriv_psi_D1);
        simvm.psi_Q1_ode1 += CL_TS * (deriv_psi_Q1);
        simvm.psi_D2_ode1 = simvm.psi_D1_ode1 - CTRL.motor->Lsigma*CTRL.I->idq[0];
        simvm.psi_Q2_ode1 = simvm.psi_Q1_ode1 - CTRL.motor->Lsigma*CTRL.I->idq[1];

        // FE.fme.psi_DQ2_output[0] = simvm.psi_D2_ode1 - K_VM*(simvm.psi_Q1_ode1 - FE.fme.psi_DQ1[0]);
        // FE.fme.psi_DQ2_output[1] = simvm.psi_Q2_ode1 - K_VM*(simvm.psi_D1_ode1 - 0);


        FE.fme.temp += (marino.psi_Dmu - FE.fme.psi_DQ2[0]) * CTRL.S->cosT;
    }
#endif

/* Main Observer for Marino05 */
void observer_marino2005(){

    /* OBSERVATION */
    marino05_dedicated_rk4_solver(1*CL_TS);

    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    //  2017年1月20日，将控制器放到了观测器的后面。
    // * 所以，上一步电压US_P的更新也要延后了。
    // US_P(0) = US_C(0); 
    // US_P(1) = US_C(1);
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}



/********************************************
 * Collections of Flux Estimators 
 ********************************************/
void flux_observer(){

    watch.offset[0] = OFFSET_VOLTAGE_ALPHA;
    watch.offset[1] = OFFSET_VOLTAGE_BETA;

    // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Akatsu_RrId();

    #if PC_SIMULATION
        simulation_only_flux_estimator();
    #endif

    // VM_Ohtani1992();
    // VM_HuWu1998();
    // VM_HoltzQuan2002();
    VM_HoltzQuan2003(); // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
    // VM_Harnefors2003_SCVM();
    // VM_LascuAndreescus2006();
    // VM_Stojic2015();
    // VM_ExactCompensation();
    VM_ProposedCmdErrFdkCorInFrameRho();
    // VM_ClosedLoopFluxEstimator();
    // stableFME();

    watch.psi_2[0] = FLUX_FEEDBACK_ALPHA;
    watch.psi_2[1] = FLUX_FEEDBACK_BETA;
    watch.offset_compensation[0] = OFFSET_COMPENSATION_ALPHA;
    watch.offset_compensation[1] = OFFSET_COMPENSATION_BETA;
}

/********************************************
 * Category 1 Estimators: Flux Command Correction
 ********************************************/
    /* 1. Ohtani 1992 */
    void rhf_Ohtani1992_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA + VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi_ABmu[0] - (x[0]-CTRL.motor->Lsigma*IS(0)) );
        emf[1] = US(1) - CTRL.motor->R*IS(1) + OFFSET_VOLTAGE_BETA  + VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi_ABmu[1] - (x[1]-CTRL.motor->Lsigma*IS(1)) );
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void VM_Ohtani1992(){
        if(FALSE){
            #define RS CTRL.motor->R
            #define LSIGMA CTRL.motor->Lsigma
            #define OMEGA_SYN CTRL.S->omega_syn
            /* Ohtani Voltage Model in indirect oriented DQ frame + ODE1 */
            // 
            FE.ohtani.deriv_psi_DQ1[0] = CTRL.O->udq_cmd[0] - RS*CTRL.I->idq[0] + OMEGA_SYN*FE.ohtani.psi_DQ1[1] + 1*GAIN_OHTANI * ( CTRL.I->cmd_psi - FE.ohtani.psi_DQ2[0] );
            FE.ohtani.deriv_psi_DQ1[1] = CTRL.O->udq_cmd[1] - RS*CTRL.I->idq[1] - OMEGA_SYN*FE.ohtani.psi_DQ1[0] + 0*GAIN_OHTANI * (             0.0 - FE.ohtani.psi_DQ2[1] );
            // stator flux updates
            FE.ohtani.psi_DQ1[0] += CL_TS * (FE.ohtani.deriv_psi_DQ1[0]);
            FE.ohtani.psi_DQ1[1] += CL_TS * (FE.ohtani.deriv_psi_DQ1[1]);
            // rotor flux updates
            FE.ohtani.psi_DQ2_prev[0] = FE.ohtani.psi_DQ2[0];
            FE.ohtani.psi_DQ2_prev[1] = FE.ohtani.psi_DQ2[1];
            FE.ohtani.psi_DQ2[0] = FE.ohtani.psi_DQ1[0] - LSIGMA*CTRL.I->idq[0];
            FE.ohtani.psi_DQ2[1] = FE.ohtani.psi_DQ1[1] - LSIGMA*CTRL.I->idq[1];        
        }else{
            /* Ohtani Voltage Model in AB frame + ODE4 */
            // stator flux updates
            general_2states_rk4_solver(&rhf_Ohtani1992_Dynamics, CTRL.timebase, FE.ohtani.psi_1, CL_TS);
            // rotor flux updates
            FE.ohtani.psi_2[0]         = FE.ohtani.psi_1[0] - LSIGMA*IS_C(0);
            FE.ohtani.psi_2[1]         = FE.ohtani.psi_1[1] - LSIGMA*IS_C(1);
            FE.ohtani.psi_2_ampl = sqrt(FE.ohtani.psi_2[0]*FE.ohtani.psi_2[0]+FE.ohtani.psi_2[1]*FE.ohtani.psi_2[1]);
        }
    }

    /* 2. Holtz and Quan 2002 */
    void rhf_Holtz2002_Dynamics(REAL t, REAL *x, REAL *fx){

        REAL psi_2[2];
        psi_2[0] = x[0] - CTRL.motor->Lsigma*IS(0);
        psi_2[1] = x[1] - CTRL.motor->Lsigma*IS(1);
        REAL ampl_psi_2 = sqrt(psi_2[0]*psi_2[0] + psi_2[1]*psi_2[1]);
        REAL ampl_psi_2_inv=0.0;
        if(ampl_psi_2!=0){
            ampl_psi_2_inv = 1.0/ampl_psi_2;
        }
        REAL cos_rho = psi_2[0] * ampl_psi_2_inv;
        REAL sin_rho = psi_2[1] * ampl_psi_2_inv;

        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA + VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi*cos_rho - psi_2[0] );
        emf[1] = US(1) - CTRL.motor->R*IS(1) + OFFSET_VOLTAGE_BETA  + VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi*sin_rho - psi_2[1] );
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void VM_HoltzQuan2002(){
        /* Ohtani Voltage Model in AB frame + ODE4 */
        // stator flux updates
        general_2states_rk4_solver(&rhf_Holtz2002_Dynamics, CTRL.timebase, FE.holtz02.psi_1, CL_TS);
        // rotor flux updates
        FE.holtz02.psi_2[0]         = FE.holtz02.psi_1[0] - LSIGMA*IS_C(0);
        FE.holtz02.psi_2[1]         = FE.holtz02.psi_1[1] - LSIGMA*IS_C(1);
        FE.holtz02.psi_2_ampl = sqrt(FE.holtz02.psi_2[0]*FE.holtz02.psi_2[0]+FE.holtz02.psi_2[1]*FE.holtz02.psi_2[1]);
    }

    /* 3. Lascu and Andreescus 2006 TODO 非常好奇Lascu的方法会怎样！和我们的xRho校正项对比！ */
    void rhf_LascuAndreescus2006_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL rotor_flux[2];
        rotor_flux[0] = x[0]-CTRL.motor->Lsigma*IS(0);
        rotor_flux[1] = x[1]-CTRL.motor->Lsigma*IS(1);

        REAL ampl     = sqrt(rotor_flux[0]*rotor_flux[0] + rotor_flux[1]*rotor_flux[1]);
        REAL ampl_inv = 0;
        if(ampl!=0){
           ampl_inv = 1.0/ampl;
        }
        REAL rotor_flux_error[2]={0,0};
        rotor_flux_error[0] = ( CTRL.I->cmd_psi - ampl ) * rotor_flux[0] * ampl_inv;
        rotor_flux_error[1] = ( CTRL.I->cmd_psi - ampl ) * rotor_flux[1] * ampl_inv;

        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[0] \
            /*I*/+ 1*x[2];
        emf[1] = US(1) - CTRL.motor->R*IS(1) + 1*OFFSET_VOLTAGE_BETA  \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[1] \
            /*I*/+ 1*x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[0];
        fx[3] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[1];
    }
    void VM_LascuAndreescus2006(){
        /* Proposed VM based Flux Command Error Feedback Correction in Controller Frame (xRho), implemented in AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_LascuAndreescus2006_Dynamics, CTRL.timebase, FE.lascu.x, CL_TS);
        // Unpack x
        FE.lascu.psi_1[0]                    = FE.lascu.x[0];
        FE.lascu.psi_1[1]                    = FE.lascu.x[1];
        FE.lascu.correction_integral_term[0] = FE.lascu.x[2];
        FE.lascu.correction_integral_term[1] = FE.lascu.x[3];
        FE.lascu.u_offset[0] = FE.lascu.correction_integral_term[0];
        FE.lascu.u_offset[1] = FE.lascu.correction_integral_term[1];
        // rotor flux updates
        FE.lascu.psi_2[0] = FE.lascu.psi_1[0] - LSIGMA*IS_C(0);
        FE.lascu.psi_2[1] = FE.lascu.psi_1[1] - LSIGMA*IS_C(1);
        FE.lascu.psi_2_ampl = sqrt(FE.lascu.psi_2[0]*FE.lascu.psi_2[0]+FE.lascu.psi_2[1]*FE.lascu.psi_2[1]);
    }

    /* 4. B. Proposed Flux Command Error feedback PI Correction in Controller Frame (xRho) */
    void rhf_CmdErrFdkCorInFrameRho_Dynamics(REAL t, REAL *x, REAL *fx){

        REAL rotor_flux_error[2];
        rotor_flux_error[0] = ( CTRL.I->cmd_psi_ABmu[0] - (x[0]-CTRL.motor->Lsigma*IS(0)) );
        rotor_flux_error[1] = ( CTRL.I->cmd_psi_ABmu[1] - (x[1]-CTRL.motor->Lsigma*IS(1)) );

        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/+ VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[0] \
            /*I*/+ x[2];
        emf[1] = US(1) - CTRL.motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/+ VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[1] \
            /*I*/+ x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[0];
        fx[3] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[1];
    }
    void VM_ProposedCmdErrFdkCorInFrameRho(){
        /* Proposed VM based Flux Command Error Feedback Correction in Controller Frame (xRho), implemented in AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_CmdErrFdkCorInFrameRho_Dynamics, CTRL.timebase, FE.picorr.x, CL_TS);
        // Unpack x
        FE.picorr.psi_1[0]                    = FE.picorr.x[0];
        FE.picorr.psi_1[1]                    = FE.picorr.x[1];
        FE.picorr.correction_integral_term[0] = FE.picorr.x[2];
        FE.picorr.correction_integral_term[1] = FE.picorr.x[3];
        FE.picorr.u_offset[0] = FE.picorr.correction_integral_term[0];
        FE.picorr.u_offset[1] = FE.picorr.correction_integral_term[1];
        // rotor flux updates
        FE.picorr.psi_2[0] = FE.picorr.psi_1[0] - LSIGMA*IS_C(0);
        FE.picorr.psi_2[1] = FE.picorr.psi_1[1] - LSIGMA*IS_C(1);
        FE.picorr.psi_2_ampl = sqrt(FE.picorr.psi_2[0]*FE.picorr.psi_2[0]+FE.picorr.psi_2[1]*FE.picorr.psi_2[1]);
    }

/********************************************
 * Category 2 Estimators: Flux Command Free
 ********************************************/

    /* 5. Hu and Wu 1998 */
    void VM_HuWu1998(){
    }

    /* 6. Stojic 2015 */
    void VM_Stojic2015(){
    }

    /* 7. SCVM - Harnefors 2003 (TODO: Try mu=-1 ????????) */
    void rhf_Harnefors2003_Dynamics(REAL t, REAL *x, REAL *fx){
        #define OMEGA_SYN CTRL.S->omega_syn
        #define LAMBDA FE.harnefors.lambda

        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + 0*OFFSET_VOLTAGE_ALPHA;
        emf[1] = US(1) - CTRL.motor->R*IS(1) + 0*OFFSET_VOLTAGE_BETA ;
        fx[0] = - LAMBDA * fabs(OMEGA_SYN) * x[0] + emf[0] + LAMBDA * sign(OMEGA_SYN) * emf[1];
        fx[1] = - LAMBDA * fabs(OMEGA_SYN) * x[1] + emf[1] + LAMBDA * sign(OMEGA_SYN) *-emf[0];
    }
    void VM_Harnefors2003_SCVM(){
        #define LSIGMA CTRL.motor->Lsigma

        // stator flux updates
        general_2states_rk4_solver(&rhf_Harnefors2003_Dynamics, CTRL.timebase, FE.harnefors.psi_1, CL_TS);
        // rotor flux updates
        FE.harnefors.psi_2[0]         = FE.harnefors.psi_1[0] - LSIGMA*IS_C(0);
        FE.harnefors.psi_2[1]         = FE.harnefors.psi_1[1] - LSIGMA*IS_C(1);
        FE.harnefors.psi_2_ampl = sqrt(FE.harnefors.psi_2[0]*FE.harnefors.psi_2[0]+FE.harnefors.psi_2[1]*FE.harnefors.psi_2[1]);
    }

    /* 8. C. Output Error Closed-loop Flux Estimator */
    void rhf_ClosedLoopFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2]: \psi_{d\mu}
        // x[3], x[4]: integral action compensating offset voltage

        REAL psi_2[2];
        psi_2[0] = x[0] - CTRL.motor->Lsigma*IS(0);
        psi_2[1] = x[1] - CTRL.motor->Lsigma*IS(1);
        REAL ampl_psi_2 = sqrt(psi_2[0]*psi_2[0] + psi_2[1]*psi_2[1]);
        REAL ampl_psi_2_inv=0.0;
        if(ampl_psi_2!=0){
            ampl_psi_2_inv = 1.0/ampl_psi_2;
        }

        REAL cos_rho = psi_2[0] * ampl_psi_2_inv;
        REAL sin_rho = psi_2[1] * ampl_psi_2_inv;

        /* Variant with command rho */
        // REAL cos_rho = CTRL.S->cosT;
        // REAL sin_rho = CTRL.S->sinT;

        REAL current_estimate[2];
        current_estimate[0] = CTRL.motor->Lsigma_inv * (x[0] - x[2]*cos_rho);
        current_estimate[1] = CTRL.motor->Lsigma_inv * (x[1] - x[2]*sin_rho);

        REAL current_error[2];
        current_error[0] = IS(0) - current_estimate[0];
        current_error[1] = IS(1) - current_estimate[1];

        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/+ OUTPUT_ERROR_CLEST_GAIN_KP * current_error[0] \
            /*I*/+ x[3];
        emf[1] = US(1) - CTRL.motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/+ OUTPUT_ERROR_CLEST_GAIN_KP * current_error[1] \
            /*I*/+ x[4];
        fx[0] = emf[0];
        fx[1] = emf[1];
        /* Lascu CLE */
        // REAL psi_ds = x[0]*cos_rho + x[1]*sin_rho;
        // fx[2] = CTRL.motor->Rreq*CTRL.motor->Lsigma_inv*psi_ds - CTRL.motor->alpha*(CTRL.motor->Lmu+CTRL.motor->Lsigma)*CTRL.motor->Lsigma_inv*x[2]\
        //       + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
        /* MRAS */
        REAL ids = IS(0)*cos_rho + IS(1)*sin_rho;
        fx[2] = CTRL.motor->Rreq*ids - CTRL.motor->alpha*x[2]\
              + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
        fx[3] = OUTPUT_ERROR_CLEST_GAIN_KI * current_error[0];
        fx[4] = OUTPUT_ERROR_CLEST_GAIN_KI * current_error[1];
    }
    void VM_ClosedLoopFluxEstimator(){
        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_5states_rk4_solver(&rhf_ClosedLoopFluxEstimator_Dynamics, CTRL.timebase, FE.clest.x, CL_TS);
        // Unpack x
        FE.clest.psi_1[0]                    = FE.clest.x[0];
        FE.clest.psi_1[1]                    = FE.clest.x[1];
        FE.clest.psi_dmu                     = FE.clest.x[2];
        FE.clest.correction_integral_term[0] = FE.clest.x[3];
        FE.clest.correction_integral_term[1] = FE.clest.x[4];
        FE.clest.u_offset[0] = FE.clest.correction_integral_term[0];
        FE.clest.u_offset[1] = FE.clest.correction_integral_term[1];
        // rotor flux updates
        FE.clest.psi_2[0] = FE.clest.psi_1[0] - CTRL.motor->Lsigma*IS_C(0);
        FE.clest.psi_2[1] = FE.clest.psi_1[1] - CTRL.motor->Lsigma*IS_C(1);    
        FE.clest.psi_2_ampl = sqrt(FE.clest.psi_2[0]*FE.clest.psi_2[0]+FE.clest.psi_2[1]*FE.clest.psi_2[1]);
    }

    // CM in synchronous dq frame
        // void rhf_stableFME_Dynamics(REAL t, REAL *x, REAL *fx){
        //     #define OMEGA_SYN CTRL.S->omega_syn
        //     #define LAMBDA FE.harnefors.lambda

        //     REAL emf[2];
        //     emf[0] = US(0) - CTRL.motor->R*IS(0) + 0*OFFSET_VOLTAGE_ALPHA;
        //     emf[1] = US(1) - CTRL.motor->R*IS(1) + 0*OFFSET_VOLTAGE_BETA ;
        //     fx[0] = - LAMBDA * fabs(OMEGA_SYN) * x[0] + emf[0] + LAMBDA * sign(OMEGA_SYN) * emf[1];
        //     fx[1] = - LAMBDA * fabs(OMEGA_SYN) * x[1] + emf[1] + LAMBDA * sign(OMEGA_SYN) *-emf[0];
        // }
        // void stableFME(){
            //FE.fme.psi_DQ2[0] += CL_TS * (-CTRL.motor->alpha*FE.fme.psi_DQ2[0] + CTRL.motor->Rreq*CTRL.I->idq[0]);
        // }

/********************************************
 * Category 3 Estimators: Saturation
 ********************************************/
    /* 9. Holtz and Quan 2003 (LPF) */
    void VM_HoltzQuan2003(){
        VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();    
    }

    /* 10. Holtz and Quan 2003 (Integrator) */


    /* 11. Exact Offset Compensation */

    /* 12. Adaptive Limit */

    /* 13. Saturation Time Difference */


    /* 14. A. [Top and Butt] Exact Compensation Method (Proposed) */
    // #define GAIN_OHTANI (0.5) // must be zero for estimating u_offset by top change
    void rhf_ExactCompensation_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL emf[2];
        emf[0] = US(0) - CTRL.motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA - 0*K_VM*(x[0]-FE.fme.psi_DQ1[0]*CTRL.S->cosT) - 0*FE.exact.u_offset_estimated_by_integration_correction[0] - 0*FE.exact.u_offset_estimated_by_top_minus_butt[0] - 0*FE.exact.offset_voltage_compensation[0] - 0*FE.exact.u_offset_estimated_by_psi_2_top_change[0] - 0*FE.exact.u_offset[0] + 0*VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi_ABmu[0] - (x[0]-CTRL.motor->Lsigma*IS(0)) ) + 0*FE.exact.correction_integral_term[0];
        emf[1] = US(1) - CTRL.motor->R*IS(1) + OFFSET_VOLTAGE_BETA  - 0*K_VM*(x[1]-FE.fme.psi_DQ1[0]*CTRL.S->sinT) - 0*FE.exact.u_offset_estimated_by_integration_correction[1] - 0*FE.exact.u_offset_estimated_by_top_minus_butt[1] - 0*FE.exact.offset_voltage_compensation[1] - 0*FE.exact.u_offset_estimated_by_psi_2_top_change[1] - 0*FE.exact.u_offset[1] + 0*VM_OHTANI_CORRECTION_GAIN_P * ( CTRL.I->cmd_psi_ABmu[1] - (x[1]-CTRL.motor->Lsigma*IS(1)) ) + 0*FE.exact.correction_integral_term[1];
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void zero_crossing_method(int i){
        // ZERO cannot be changed during the loop
        #define ZERO FE.exact.psi_2_last_zero_level_inLoopTheSame[i]
        ZERO = FE.exact.psi_2_last_zero_level[i];

        #if PC_SIMULATION
        if((int)(CTRL.timebase/CL_TS)%10000==0){
            printf("%g: %g, %g\n", CTRL.timebase, FE.exact.psi_2_prev[i], FE.exact.psi_2[i]);
        }
        #endif

        /* 负负负负负负负负负负负负负负负负负负负，寻找最小值 */
        // 进入磁链负半周？
        if(FE.exact.flag_Neg_stageA[i] == TRUE){
            /* 进入【磁链负半周】！ Stage A */
            // 二次检查，磁链已经是负的了？
            if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]<ZERO){
                /* 进入【磁链负半周】！ Stage B */
                if(FE.exact.flag_Neg_stageB[i] == FALSE){
                    //MUTE printf("[%d] Neg Stage A Enter %g\n", i, CTRL.timebase);
                    // 这边只做一次
                    FE.exact.flag_Neg_stageB[i] = TRUE; 
                    // 计算相邻两次过零间隔时间：Delta_t
                    FE.exact.Delta_t_NegMinusPos[i] = FE.exact.time_Neg[i] - FE.exact.time_Pos[i];
                    // 初始化对立面的flag
                    FE.exact.flag_Pos_stageA[i] = FALSE;
                    FE.exact.flag_Pos_stageB[i] = FALSE;
                    // 初始化最小值
                    FE.exact.psi_2_min[i] = 0.0;
                }
                // 寻找磁链最小值
                if(FE.exact.psi_2[i] < FE.exact.psi_2_min[i]) {FE.exact.psi_2_min[i] = FE.exact.psi_2[i];}
            }else{
                /* 结算开始 */
                //MUTE printf("[%d] Neg Quit %g\n", i, CTRL.timebase);
                // 退出【磁链负半周】！
                // 初始化flag
                FE.exact.flag_Neg_stageA[i] = FALSE;
                FE.exact.flag_Neg_stageB[i] = FALSE;
                // 退出的时候，顺便计算一次李萨如图圆心横/纵坐标
                FE.exact.psi_2_maxmin_difference[i] = (FE.exact.psi_2_max[i]) + (FE.exact.psi_2_min[i]);
                FE.exact.psi_2_moved_distance_in_LissajousPlot[i] = 0.5*FE.exact.psi_2_maxmin_difference[i] - ZERO;
                if( FE.exact.Delta_t_PosMinusNeg[i] !=0 ){
                    // 这里只是近似，真正的时间应该是最大值所对应的时间和最小值对应的时间的差，而不是过零变负和过零变正之间的时间差！
                    REAL time_of_offset_voltage_contributing_to_the_difference_between_max_and_min = FE.exact.Delta_t_PosMinusNeg[i];
                    FE.exact.offset_voltage_by_zero_crossing[i] = FE.exact.psi_2_maxmin_difference[i] / time_of_offset_voltage_contributing_to_the_difference_between_max_and_min;
                }
                FE.exact.psi_2_last_zero_level[i] = 0.5*(FE.exact.psi_2_max[i] + FE.exact.psi_2_min[i]);
            }
        }
        /* 磁链正切负（必须放在后面执行！否则已进入【磁链负半周】判断后，会因为当前和上一步异号马上退出） */
        // 发现磁链由正变负的时刻
        if(FE.exact.psi_2_prev[i]>ZERO && FE.exact.psi_2[i]<ZERO){
            FE.exact.flag_Neg_stageA[i] = TRUE;
            FE.exact.time_Neg[i] = CTRL.timebase;
            //MUTE printf("[%d] Neg Crossing %g\n", i, CTRL.timebase);
        }

        /* 正正正正正正正正正正正正正正正正正正正，寻找最大值 */
        // 进入【磁链正半周】？
        if(FE.exact.flag_Pos_stageA[i] == TRUE){
            /* 进入【磁链正半周】！ Stage A */
            // 二次检查，磁链已经是正的了？
            if(FE.exact.psi_2_prev[i]>ZERO && FE.exact.psi_2[i]>ZERO){
                /* 进入【磁链正半周】！ Stage B */
                if(FE.exact.flag_Pos_stageB[i] == FALSE){
                    //MUTE printf("[%d] Pos Stage A Enter %g\n", i, CTRL.timebase);
                    // 这边只做一次
                    FE.exact.flag_Pos_stageB[i] = TRUE; 
                    // 计算相邻两次过零间隔时间：Delta_t
                    FE.exact.Delta_t_PosMinusNeg[i] = FE.exact.time_Pos[i] - FE.exact.time_Neg[i];
                    // 初始化对立面的flag
                    FE.exact.flag_Neg_stageA[i] = FALSE;
                    FE.exact.flag_Neg_stageB[i] = FALSE;
                    // 初始化最大值
                    FE.exact.psi_2_max[i] = 0.0;
                }
                // 寻找磁链最大值
                if(FE.exact.psi_2[i] > FE.exact.psi_2_max[i]){FE.exact.psi_2_max[i] = FE.exact.psi_2[i];}
            }else{
                /* 结算开始 */
                //MUTE printf("[%d] Pos Quit %g\n", i, CTRL.timebase);
                // 退出【磁链正半周】！
                // 初始化flag
                FE.exact.flag_Pos_stageA[i] = FALSE;
                FE.exact.flag_Pos_stageB[i] = FALSE;
                // 退出的时候，顺便计算一次李萨如图圆心横/纵坐标
                FE.exact.psi_2_maxmin_difference[i] = (FE.exact.psi_2_max[i]) + (FE.exact.psi_2_min[i]);
                FE.exact.psi_2_moved_distance_in_LissajousPlot[i] = 0.5*FE.exact.psi_2_maxmin_difference[i] - ZERO;
                if( FE.exact.Delta_t_NegMinusPos[i] !=0 ){
                    // 这里只是近似，真正的时间应该是最大值所对应的时间和最小值对应的时间的差，而不是过零变负和过零变正之间的时间差！
                    REAL time_of_offset_voltage_contributing_to_the_difference_between_max_and_min = FE.exact.Delta_t_NegMinusPos[i];
                    FE.exact.offset_voltage_by_zero_crossing[i] = FE.exact.psi_2_maxmin_difference[i] / time_of_offset_voltage_contributing_to_the_difference_between_max_and_min;                    
                }
                FE.exact.psi_2_last_zero_level[i] = 0.5*(FE.exact.psi_2_max[i] + FE.exact.psi_2_min[i]);                
            }
        }
        /* 磁链负切正（必须放在后面执行！否则已进入【磁链正半周】判断后，会因为当前和上一步异号马上退出） */
        // 发现磁链由负变正的时刻
        if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]>ZERO){
            FE.exact.flag_Pos_stageA[i] = TRUE;
            FE.exact.time_Pos[i] = CTRL.timebase;
            #if PC_SIMULATION
            printf("[%d] Pos Crossing %g\n", i, CTRL.timebase);
            #endif
        }

        #if PC_SIMULATION
        // DEBUG
        if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]>ZERO){
            printf("!!!!!!!!!!!!!! %g\n", CTRL.timebase);
        }
        #endif
    }
    void VM_ExactCompensation(){
        #define RS CTRL.motor->R
        #define LSIGMA CTRL.motor->Lsigma
        #define OMEGA_SYN CTRL.S->omega_syn

        if(FALSE){
            /* Ohtani Voltage Model in indirect oriented DQ frame + ODE1 */
            // 
            REAL deriv_psi_DQ1[2];
            deriv_psi_DQ1[0] = CTRL.O->udq_cmd[0] - RS*CTRL.I->idq[0] + OMEGA_SYN*FE.exact.psi_DQ1[1] + GAIN_OHTANI * ( CTRL.I->cmd_psi - FE.exact.psi_DQ2[0] );
            deriv_psi_DQ1[1] = CTRL.O->udq_cmd[1] - RS*CTRL.I->idq[1] - OMEGA_SYN*FE.exact.psi_DQ1[0] + GAIN_OHTANI * (             0.0 - FE.exact.psi_DQ2[1] );
            // stator flux updates
            FE.exact.psi_DQ1[0] += CL_TS * (deriv_psi_DQ1[0]);
            FE.exact.psi_DQ1[1] += CL_TS * (deriv_psi_DQ1[1]);
            // rotor flux updates
            FE.exact.psi_DQ2_prev[0] = FE.exact.psi_DQ2[0];
            FE.exact.psi_DQ2_prev[1] = FE.exact.psi_DQ2[1];
            FE.exact.psi_DQ2[0] = FE.exact.psi_DQ1[0] - LSIGMA*CTRL.I->idq[0];
            FE.exact.psi_DQ2[1] = FE.exact.psi_DQ1[1] - LSIGMA*CTRL.I->idq[1];        
        }else{
            /* Ohtani Voltage Model in AB frame + ODE4 */

            /* Low-Pass Filter */
            // FE.exact.filtered_compensation[0] = _lpf(0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]), FE.exact.filtered_compensation[0], 15);
            // FE.exact.filtered_compensation[1] = _lpf(0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]), FE.exact.filtered_compensation[1], 15);
            // FE.exact.filtered_compensation2[0] = _lpf(0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] ), FE.exact.filtered_compensation2[0], 15);
            // FE.exact.filtered_compensation2[1] = _lpf(0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] ), FE.exact.filtered_compensation2[1], 15);
            /* No Filter */
            FE.exact.filtered_compensation[0]  = 0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]);
            FE.exact.filtered_compensation[1]  = 0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]);
            FE.exact.filtered_compensation2[0] = 0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] );
            FE.exact.filtered_compensation2[1] = 0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] );

            /* 补偿 psi_offset 没用！ */
            // if(FE.exact.bool_compensate_psi_offset[0]){
            //     FE.exact.bool_compensate_psi_offset[0] = FALSE;
            //     FE.exact.psi_1[0] -= FE.exact.filtered_compensation[0];
            // }
            // if(FE.exact.bool_compensate_psi_offset[1]){
            //     FE.exact.bool_compensate_psi_offset[1] = FALSE;
            //     FE.exact.psi_1[1] -= FE.exact.filtered_compensation[1];
            // }

            // FE.exact.psi_1[0] -= FE.exact.filtered_compensation[0]; // 只需要在波峰和波谷的时候补一次就可以了
            // FE.exact.psi_1[1] -= FE.exact.filtered_compensation[1]; // 只需要在波峰和波谷的时候补一次就可以了

            // stator flux updates
            // #define VM_CORRECTION_KI (2.5)
            // FE.exact.correction_integral_term[0] += CL_TS * VM_CORRECTION_KI * ( CTRL.I->cmd_psi_ABmu[0] - FE.exact.psi_2[0] );
            // FE.exact.correction_integral_term[1] += CL_TS * VM_CORRECTION_KI * ( CTRL.I->cmd_psi_ABmu[1] - FE.exact.psi_2[1] );
            general_2states_rk4_solver(&rhf_ExactCompensation_Dynamics, CTRL.timebase, FE.exact.psi_1, CL_TS);
            // rotor flux updates
            FE.exact.psi_2_pprev[0]   = FE.exact.psi_2_prev[0];
            FE.exact.psi_2_pprev[1]   = FE.exact.psi_2_prev[1];
            FE.exact.psi_2_prev[0]    = FE.exact.psi_2[0];
            FE.exact.psi_2_prev[1]    = FE.exact.psi_2[1];
            FE.exact.psi_2[0]         = FE.exact.psi_1[0] - LSIGMA*IS_C(0);
            FE.exact.psi_2[1]         = FE.exact.psi_1[1] - LSIGMA*IS_C(1);
            FE.exact.psi_2_output[0]  = FE.exact.psi_2[0]        - 1*FE.exact.filtered_compensation[0];
            FE.exact.psi_2_output[1]  = FE.exact.psi_2[1]        - 1*FE.exact.filtered_compensation[1];
            FE.exact.psi_2_output2[0] = FE.exact.psi_2_output[0] - 1*FE.exact.filtered_compensation2[0];
            FE.exact.psi_2_output2[1] = FE.exact.psi_2_output[1] - 1*FE.exact.filtered_compensation2[1];

            FE.exact.psi_2_real_output[0] = 0.5*(FE.exact.psi_2_output[0] + FE.exact.psi_2_output2[0]);
            FE.exact.psi_2_real_output[1] = 0.5*(FE.exact.psi_2_output[1] + FE.exact.psi_2_output2[1]);
            FE.exact.psi_2_ampl = sqrt(FE.exact.psi_2_real_output[0]*FE.exact.psi_2_real_output[0]+FE.exact.psi_2_real_output[1]*FE.exact.psi_2_real_output[1]);
        }

        // #define INTEGRAL_GAIN_OHTANI 1
        // FE.exact.u_offset_estimated_by_Lascu_integral_term[0] += CL_TS * INTEGRAL_GAIN_OHTANI * ( CTRL.I->cmd_psi_ABmu[0] - (FE.exact.psi_2[0]-CTRL.motor->Lsigma*IS_C(0)) );
        // FE.exact.u_offset_estimated_by_Lascu_integral_term[1] += CL_TS * INTEGRAL_GAIN_OHTANI * ( CTRL.I->cmd_psi_ABmu[1] - (FE.exact.psi_2[1]-CTRL.motor->Lsigma*IS_C(1)) );

        int i;
        /* αβ */
        for(i=0; i<2; ++i){

            /* TOP */
            if (   FE.exact.psi_2_prev[i] - FE.exact.psi_2_pprev[i] >= 0 \
                && FE.exact.psi_2_prev[i] - FE.exact.psi_2      [i] >= 0 \
                && FE.exact.psi_2_pprev[i] > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2_prev[i]  > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2[i]       > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) ){
                /* 发现最高点，而且不能简单根据正负来判断，一旦磁链掉下去了，这判断就无法成立了 */
                    // 根据求和结果计算 u_offset
                    // REAL denumerator = 0.5*(1+FE.exact.top2top_count[i]) * CL_TS;
                    // FE.exact.u_offset_estimated_by_psi_2_sumup[i] = ( FE.exact.top2top_sum[i] / FE.exact.top2top_count[i] - FE.exact.psi_2_offset[i] ) / denumerator;
                    // FE.exact.psi_2_offset[i] = FE.exact.top2top_sum[i] / FE.exact.top2top_count[i];
                    // FE.exact.top2top_sum[i] = 0.0;
                if( FE.exact.top2top_count[i]> 10 ){ // 防止静止时疯狂判断成立，因为pprev==prev==curr // 10 / TS = 1000 Hz
                    FE.exact.top2top_count[i] = 0;

                    // 磁链最大值变化信息
                    FE.exact.flag_top2top[i] = TRUE;
                    FE.exact.change_in_psi_2_top[i] = FE.exact.psi_2[i] - FE.exact.psi_2_last_top[i];

                    { /* 补偿 psi_offset */
                        // FE.exact.psi_1[i]      -= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2_pprev[i]-= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2_prev[i] -= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2[i]      -= FE.exact.change_in_psi_2_top[i];
                    }
                        // printf("TOP {%d} [%g] %g - %g\n", i, CTRL.timebase, FE.exact.psi_2[i], FE.exact.psi_2_last_top[i]);
                    FE.exact.psi_2_last_top[i]         = FE.exact.psi_2[i];
                    FE.exact.psi_2_output_last_top[i]  = FE.exact.psi_2_output[i];
                    FE.exact.psi_2_output2_last_top[i] = FE.exact.psi_2_output2[i];
                    // FE.exact.u_offset_estimated_by_top_minus_butt[i] += 190 * CL_TS * 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]);

                    FE.exact.current_Lissa_move[i] = 0.5*( FE.exact.psi_2_output_last_top[i]+FE.exact.psi_2_output_last_butt[i] );
                    FE.exact.u_offset_error[i] = (FE.exact.current_Lissa_move[i] - FE.exact.last_Lissa_move[i]) / (CTRL.timebase - FE.exact.last_time_reaching_butt[i]);
                    FE.exact.last_Lissa_move[i] = FE.exact.current_Lissa_move[i];

                    FE.exact.u_offset_estimated_by_integration_correction[i] = FE.exact.u_offset_error[i];

                    FE.exact.bool_compensate_psi_offset[i] = TRUE;

                    // 时间信息
                    FE.exact.Delta_time_top2top[i] = CTRL.timebase - FE.exact.last_time_reaching_top[i];
                    FE.exact.last_time_reaching_top[i] = CTRL.timebase;
                    // 根据磁链最大值幅值变化来计算 u_offset
                    if(FE.exact.first_time_enter_top[i]){FE.exact.first_time_enter_top[i] = FALSE;}
                    else{
                        FE.exact.u_offset_estimated_by_psi_2_top_change[i] = FE.exact.change_in_psi_2_top[i] / FE.exact.Delta_time_top2top[i];
                        FE.exact.u_offset[i] = 0.5*(FE.exact.u_offset_estimated_by_psi_2_top_change[i] + FE.exact.u_offset_estimated_by_psi_2_butt_change[i]);
                    }

                }else{
                    FE.exact.top2top_count[i] = 0;
                }
            }else{
                FE.exact.flag_top2top[i] = FALSE;
            }
            // sum it up
            // FE.exact.top2top_sum[i] += FE.exact.psi_2[i];
            FE.exact.top2top_count[i] += 1;

            /* BUTT */
            if (   FE.exact.psi_2_prev[i] - FE.exact.psi_2_pprev[i] <= 0 \
                && FE.exact.psi_2_prev[i] - FE.exact.psi_2      [i] <= 0 \
                && FE.exact.psi_2_pprev[i] < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2_prev[i]  < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2[i]       < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) ){
                /* 发现最低点，而且不能简单根据正负来判断，一旦磁链飘上去了，这判断就无法成立了！ */

                if( FE.exact.butt2butt_count[i]> 10 ){ // 10 / TS = 1000 Hz
                    FE.exact.butt2butt_count[i] = 0;

                    // 磁链最小值变化信息
                    FE.exact.flag_butt2butt[i] = -1*TRUE;
                    FE.exact.change_in_psi_2_butt[i] = FE.exact.psi_2[i] - FE.exact.psi_2_last_butt[i];

                    { /* 补偿 psi_offset */
                        // FE.exact.psi_1[i]      -= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2_pprev[i]-= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2_prev[i] -= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2[i]      -= FE.exact.change_in_psi_2_butt[i];
                    }

                    FE.exact.psi_2_last_butt[i]        = FE.exact.psi_2[i];
                    FE.exact.psi_2_output_last_butt[i] = FE.exact.psi_2_output[i];
                    FE.exact.psi_2_output2_last_butt[i] = FE.exact.psi_2_output2[i];
                    // FE.exact.u_offset_estimated_by_top_minus_butt[i] += 190 * CL_TS * 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]);

                    FE.exact.current_Lissa_move[i] = 0.5*( FE.exact.psi_2_output_last_top[i]+FE.exact.psi_2_output_last_butt[i] );
                    FE.exact.u_offset_error[i] = (FE.exact.current_Lissa_move[i] - FE.exact.last_Lissa_move[i] ) / (CTRL.timebase - FE.exact.last_time_reaching_top[i]);
                    FE.exact.last_Lissa_move[i] = FE.exact.current_Lissa_move[i];

                    FE.exact.u_offset_estimated_by_integration_correction[i] = FE.exact.u_offset_error[i];

                    FE.exact.bool_compensate_psi_offset[i] = TRUE;

                    // 时间信息
                    FE.exact.Delta_time_butt2butt[i] = CTRL.timebase - FE.exact.last_time_reaching_butt[i];
                    FE.exact.last_time_reaching_butt[i] = CTRL.timebase;
                    // 根据磁链最大值幅值变化来计算 u_offset
                    if(FE.exact.first_time_enter_butt[i]){FE.exact.first_time_enter_butt[i] = FALSE;}
                    else{
                        FE.exact.u_offset_estimated_by_psi_2_butt_change[i] = FE.exact.change_in_psi_2_butt[i] / FE.exact.Delta_time_butt2butt[i];
                        FE.exact.u_offset[i] = 0.5*(FE.exact.u_offset_estimated_by_psi_2_top_change[i] + FE.exact.u_offset_estimated_by_psi_2_butt_change[i]);
                    }
                }else{
                    FE.exact.butt2butt_count[i] = 0;                
                }
            }else{
                FE.exact.flag_butt2butt[i] = FALSE;
            }
            // sum it up
            // FE.exact.butt2butt_sum[i] += FE.exact.psi_2[i];
            FE.exact.butt2butt_count[i] += 1;


            // zero_crossing_method(i);
        }
        // 开始。啥也没有，咱先找最大值，要靠递增到递减来确定，
        // 攥着最大值，然后要找最小值，从递减到递增来确定。
        // 有了最大值和最小值以后，可以做一次u_offset的计算和补偿。
        // 然后，攥着最小值，去找下一个最大值，有了最小值和下一个最大值以后，又可以做一次u_offset的计算和补偿。
    }



/* Holtz 2003 implemented in ACMSIMC-V4 */
REAL imife_realtime_gain_off = 1.0;
void init_FE_htz(){
    int ind;
    for(ind=0;ind<2;++ind){
        FE.htz.emf_stator[ind] = 0;

        FE.htz.psi_1[ind] = 0;
        FE.htz.psi_2[ind] = 0;
        FE.htz.psi_2_prev[ind] = 0;

        FE.htz.psi_1_nonSat[ind] = 0;
        FE.htz.psi_2_nonSat[ind] = 0;

        FE.htz.psi_1_min[ind] = 0;
        FE.htz.psi_1_max[ind] = 0;
        FE.htz.psi_2_min[ind] = 0;
        FE.htz.psi_2_max[ind] = 0;

        FE.htz.rs_est = IM_STAOTR_RESISTANCE;
        FE.htz.rreq_est = IM_ROTOR_RESISTANCE;

        FE.htz.Delta_t = 1;
        FE.htz.u_offset[ind] = 0;
        // FE.htz.u_offset_intermediate[ind] = 0;
        FE.htz.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
        FE.htz.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
        FE.htz.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
        FE.htz.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

        FE.htz.flag_pos2negLevelA[ind] = 0;
        FE.htz.flag_pos2negLevelB[ind] = 0;
        FE.htz.time_pos2neg[ind] = 0;
        FE.htz.time_pos2neg_prev[ind] = 0;

        FE.htz.flag_neg2posLevelA[ind] = 0;
        FE.htz.flag_neg2posLevelB[ind] = 0;
        FE.htz.time_neg2pos[ind] = 0;
        FE.htz.time_neg2pos_prev[ind] = 0;

        FE.htz.psi_aster_max = IM_FLUX_COMMAND_DC_PART + IM_FLUX_COMMAND_SINE_PART;

        FE.htz.maximum_of_sat_min_time[ind] = 0.0;
        FE.htz.maximum_of_sat_max_time[ind] = 0.0;
        FE.htz.sat_min_time[ind] = 0.0;
        FE.htz.sat_max_time[ind] = 0.0;
        FE.htz.sat_min_time_reg[ind] = 0.0;
        FE.htz.sat_max_time_reg[ind] = 0.0;
        FE.htz.extra_limit = 0.0;
        FE.htz.flag_limit_too_low = FALSE;
    }
}
void rhf_Holtz2003_Dynamics(REAL t, REAL *x, REAL *fx){
    FE.htz.emf_stator[0] = US(0) - FE.htz.rs_est*IS(0) - FE.htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
    FE.htz.emf_stator[1] = US(1) - FE.htz.rs_est*IS(1) - FE.htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
    fx[0] = (FE.htz.emf_stator[0]);
    fx[1] = (FE.htz.emf_stator[1]);
}
REAL marino_saturation_gain_scale_factor1 = 1.0;
REAL marino_saturation_gain_scale_factor2 = 1.0;
REAL marino_sat_d_axis_flux_control = 1.0;
REAL marino_sat_q_axis_flux_control = 1.0;
int bool_positive_extra_limit = TRUE;
void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit(){
    #define PSI_MU_ASTER_MAX FE.htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
    #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT TRUE // The limit is extended when both upper and lower limit are reached in a cycle.

    #define BOOL_USE_METHOD_LPF_INPUT      FALSE
    #define BOOL_USE_METHOD_INTEGRAL_INPUT TRUE

    // Euler's method is shit at higher speeds
    FE.htz.emf_stator[0] = US_C(0) - FE.htz.rs_est*IS_C(0) - FE.htz.u_offset[0];
    FE.htz.emf_stator[1] = US_C(1) - FE.htz.rs_est*IS_C(1) - FE.htz.u_offset[1];
    // FE.htz.psi_1[0] += CL_TS*(FE.htz.emf_stator[0]);
    // FE.htz.psi_1[1] += CL_TS*(FE.htz.emf_stator[1]);

    // rk4 
    general_2states_rk4_solver(&rhf_Holtz2003_Dynamics, CTRL.timebase, FE.htz.psi_1, CL_TS);
    FE.htz.psi_2[0] = FE.htz.psi_1[0] - CTRL.motor->Lsigma*IS_C(0);
    FE.htz.psi_2[1] = FE.htz.psi_1[1] - CTRL.motor->Lsigma*IS_C(1);
    FE.htz.psi_2_ampl = sqrt(FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);

    // 限幅前求角度还是应该限幅后？
    FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);

    FE.htz.psi_1_nonSat[0] += CL_TS*(FE.htz.emf_stator[0]);
    FE.htz.psi_1_nonSat[1] += CL_TS*(FE.htz.emf_stator[1]);
    FE.htz.psi_2_nonSat[0] = FE.htz.psi_1_nonSat[0] - CTRL.motor->Lsigma*IS_C(0);
    FE.htz.psi_2_nonSat[1] = FE.htz.psi_1_nonSat[1] - CTRL.motor->Lsigma*IS_C(1);

    // FE.htz.psi_aster_max = CTRL.taao_flux_cmd + 0.05;
    FE.htz.psi_aster_max = CTRL.I->cmd_psi + FE.htz.extra_limit;
    // FE.htz.psi_aster_max = CTRL.taao_flux_cmd;

    // 限幅是针对转子磁链限幅的
    int ind;
    for(ind=0;ind<2;++ind){
        if(CTRL.I->cmd_omg_elec != 0.0){
            if(FE.htz.psi_2[ind]    > PSI_MU_ASTER_MAX){ // TODO BUG呀！这里怎么可以是>应该是大于等于啊！
                FE.htz.psi_2[ind]   = PSI_MU_ASTER_MAX;
                FE.htz.sat_max_time[ind] += CL_TS;
                marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                marino_sat_d_axis_flux_control = 0.0;
                marino_sat_q_axis_flux_control = 0.0;
            }else if(FE.htz.psi_2[ind] < -PSI_MU_ASTER_MAX){
                FE.htz.psi_2[ind]   = -PSI_MU_ASTER_MAX;
                FE.htz.sat_min_time[ind] += CL_TS;
                marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                marino_sat_d_axis_flux_control = 0.0;
                marino_sat_q_axis_flux_control = 0.0;
            }else{
                // 这样可以及时清零饱和时间
                if(FE.htz.sat_max_time[ind]>0){FE.htz.sat_max_time[ind] -= CL_TS;}
                if(FE.htz.sat_min_time[ind]>0){FE.htz.sat_min_time[ind] -= CL_TS;}
                marino.lambda_inv = LAMBDA_INV_xOmg;
                marino.gamma_inv  = GAMMA_INV_xTL;
                marino_sat_d_axis_flux_control = 1.0;
                marino_sat_q_axis_flux_control = 1.0;
            }
        }
        // 上限饱和减去下限饱和作为误差，主要为了消除实际磁链幅值大于给定的情况，实际上这种现象在常见工况下出现次数不多。
        FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind];
        // u_offset波动会导致sat_min_time和sat_max_time的波动，这个时候最有效的办法是减少gain_off。
        // 但是同时，观察饱和时间sat_min_time等的波形，可以发现它里面也会出现一个正弦波包络线。
        if(FE.htz.sat_min_time[ind] > FE.htz.maximum_of_sat_min_time[ind]){FE.htz.maximum_of_sat_min_time[ind] = FE.htz.sat_min_time[ind];}
        if(FE.htz.sat_max_time[ind] > FE.htz.maximum_of_sat_max_time[ind]){FE.htz.maximum_of_sat_max_time[ind] = FE.htz.sat_max_time[ind];}
    }

    // 数数，算磁链周期
    if(FE.htz.psi_2[0]    > 0.0){
        FE.htz.count_positive_in_one_cycle[0] += 1;
        if(FE.htz.count_negative_in_one_cycle[0]!=0){ FE.htz.negative_cycle_in_count[0] = FE.htz.count_negative_in_one_cycle[0]; FE.htz.count_negative_in_one_cycle[0] = 0;}
    }else if(FE.htz.psi_2[0] < -0.0){
        FE.htz.count_negative_in_one_cycle[0] += 1;
        if(FE.htz.count_positive_in_one_cycle[0]!=0){ FE.htz.positive_cycle_in_count[0] = FE.htz.count_positive_in_one_cycle[0]; FE.htz.count_positive_in_one_cycle[0] = 0;}
    }
    if(FE.htz.psi_2[1]    > 0.0){
        FE.htz.count_positive_in_one_cycle[1] += 1;
        if(FE.htz.count_negative_in_one_cycle[1]!=0){ FE.htz.negative_cycle_in_count[1] = FE.htz.count_negative_in_one_cycle[1]; FE.htz.count_negative_in_one_cycle[1] = 0;}
    }else if(FE.htz.psi_2[1] < -0.0){
        FE.htz.count_negative_in_one_cycle[1] += 1;
        if(FE.htz.count_positive_in_one_cycle[1]!=0){ FE.htz.positive_cycle_in_count[1] = FE.htz.count_positive_in_one_cycle[1]; FE.htz.count_positive_in_one_cycle[1] = 0;}
    }

    // 限幅后的转子磁链，再求取限幅后的定子磁链
    FE.htz.psi_1[0] = FE.htz.psi_2[0] + CTRL.motor->Lsigma*IS_C(0);
    FE.htz.psi_1[1] = FE.htz.psi_2[1] + CTRL.motor->Lsigma*IS_C(1);

    // Speed Estimation
    if(TRUE){
        // FE.htz.ireq[0] = CTRL.Lmu_inv*FE.htz.psi_2[0] - IS_C(0);
        // FE.htz.ireq[1] = CTRL.Lmu_inv*FE.htz.psi_2[1] - IS_C(1);
        REAL temp;
        temp = (FE.htz.psi_1[0]*FE.htz.psi_1[0]+FE.htz.psi_1[1]*FE.htz.psi_1[1]);
        if(temp>0.001){
            FE.htz.field_speed_est = - (FE.htz.psi_1[0]*-FE.htz.emf_stator[1] + FE.htz.psi_1[1]*FE.htz.emf_stator[0]) / temp;
        }
        temp = (FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
        if(temp>0.001){
            FE.htz.slip_est = CTRL.motor->Rreq*(IS_C(0)*-FE.htz.psi_2[1]+IS_C(1)*FE.htz.psi_2[0]) / temp;
        }
        FE.htz.omg_est = FE.htz.field_speed_est - FE.htz.slip_est;
    }


    // TODO My proposed saturation time based correction method NOTE VERY COOL
    #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
        FE.htz.u_off_original_lpf_input[ind]         = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) /  (FE.htz.Delta_t+FE.htz.Delta_t_last); \
        FE.htz.u_off_calculated_increment[ind]       = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) / ((FE.htz.Delta_t+FE.htz.Delta_t_last) - (FE.htz.sat_max_time[ind]+FE.htz.sat_min_time[ind])); \
        FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind]; \
        FE.htz.u_off_direct_calculated[ind] += (FE.htz.count_negative_cycle+FE.htz.count_positive_cycle>4) * FE.htz.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
        // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

    for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

        // if( tmin!=0 && tmax!=0 ){
        //     // The sat func's limit is too small.
        //     limit += TS * min(tmin, tmax);
        // }


        /* 必须先检查是否进入levelA */
        if(FE.htz.flag_pos2negLevelA[ind] == TRUE){ 
            if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                if(FE.htz.flag_pos2negLevelB[ind] == FALSE){
                    FE.htz.count_negative_cycle+=1; // FE.htz.count_positive_cycle = 0;
                    // printf("POS2NEG: %g, %d\n", CTRL.timebase, ind);
                    // printf("%g, %g\n", FE.htz.psi_2_prev[ind], FE.htz.psi_2[ind]);
                    // getch();
                    // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                    FE.htz.psi_1_max[ind] = FE.htz.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.htz.Delta_t_last = FE.htz.Delta_t;
                    FE.htz.Delta_t = FE.htz.time_pos2neg[ind] - FE.htz.time_pos2neg_prev[ind];
                    FE.htz.time_pos2neg_prev[ind] = FE.htz.time_pos2neg[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.htz.flag_neg2posLevelA[ind] = FALSE;
                    FE.htz.flag_neg2posLevelB[ind] = FALSE;

                    // 注意这里是正半周到负半周切换的时候才执行一次的哦！
                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                    FE.htz.sign__u_off_saturation_time_correction[ind] = -1.0;
                    // 饱和时间的正弦包络线的正负半周的频率比磁链频率低多啦！需要再额外加一个低频u_offset校正
                    FE.htz.sat_time_offset[ind] = FE.htz.maximum_of_sat_max_time[ind] - FE.htz.maximum_of_sat_min_time[ind];
                    FE.htz.maximum_of_sat_max_time[ind] = 0.0;
                    FE.htz.maximum_of_sat_min_time[ind] = 0.0;

                    FE.htz.psi_1_min[ind] = 0.0;
                    FE.htz.psi_2_min[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.htz.sat_min_time_reg[ind] = FE.htz.sat_min_time[ind];
                        if(FE.htz.sat_max_time_reg[ind]>CL_TS && FE.htz.sat_min_time_reg[ind]>CL_TS){
                            FE.htz.flag_limit_too_low = TRUE;
                            FE.htz.extra_limit += 1e-2 * (FE.htz.sat_max_time_reg[ind] + FE.htz.sat_min_time_reg[ind]) / FE.htz.Delta_t; 
                        }else{
                            FE.htz.flag_limit_too_low = FALSE;
                            FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                            if(bool_positive_extra_limit){
                                if(FE.htz.extra_limit<0.0){
                                    FE.htz.extra_limit = 0.0;
                                }
                            }
                        }
                        FE.htz.sat_max_time_reg[ind] = 0.0;
                    }
                    FE.htz.sat_min_time[ind] = 0.0;
                }
                FE.htz.flag_pos2negLevelB[ind] = TRUE;
                if(FE.htz.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                    if(FE.htz.psi_2[ind] < FE.htz.psi_2_min[ind]){
                        FE.htz.psi_2_min[ind] = FE.htz.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]>0
                FE.htz.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
            }
        }
        if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]<0){ // 发现磁链由正变负的时刻
            FE.htz.flag_pos2negLevelA[ind] = TRUE;
            FE.htz.time_pos2neg[ind] = CTRL.timebase;
        }


        if(FE.htz.flag_neg2posLevelA[ind] == TRUE){ 
            if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
                if(FE.htz.flag_neg2posLevelB[ind] == FALSE){
                    FE.htz.count_positive_cycle+=1; // FE.htz.count_negative_cycle = 0;
                    // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                    FE.htz.psi_1_min[ind] = FE.htz.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.htz.Delta_t_last = FE.htz.Delta_t;
                    FE.htz.Delta_t = FE.htz.time_neg2pos[ind] - FE.htz.time_neg2pos_prev[ind];
                    FE.htz.time_neg2pos_prev[ind] = FE.htz.time_neg2pos[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.htz.flag_pos2negLevelA[ind] = FALSE;
                    FE.htz.flag_pos2negLevelB[ind] = FALSE;

                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                    FE.htz.sign__u_off_saturation_time_correction[ind] = 1.0;

                    FE.htz.psi_1_max[ind] = 0.0;
                    FE.htz.psi_2_max[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.htz.sat_max_time_reg[ind] = FE.htz.sat_max_time[ind];
                        if(FE.htz.sat_min_time_reg[ind]>CL_TS && FE.htz.sat_max_time_reg[ind]>CL_TS){
                            FE.htz.flag_limit_too_low = TRUE;
                            FE.htz.extra_limit += 1e-2 * (FE.htz.sat_min_time_reg[ind] + FE.htz.sat_max_time_reg[ind]) / FE.htz.Delta_t; 
                        }else{
                            FE.htz.flag_limit_too_low = FALSE;
                            FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                            if(FE.htz.extra_limit<0.0){
                                FE.htz.extra_limit = 0.0;
                            }
                        }
                        FE.htz.sat_min_time_reg[ind] = 0.0;
                    }
                    FE.htz.sat_max_time[ind] = 0.0;
                }
                FE.htz.flag_neg2posLevelB[ind] = TRUE; 
                if(FE.htz.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                    if(FE.htz.psi_2[ind] > FE.htz.psi_2_max[ind]){
                        FE.htz.psi_2_max[ind] = FE.htz.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]<0
                FE.htz.flag_neg2posLevelA[ind] = FALSE;
            }
        }
        if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]>0){ // 发现磁链由负变正的时刻
            FE.htz.flag_neg2posLevelA[ind] = TRUE;
            FE.htz.time_neg2pos[ind] = CTRL.timebase;
        }
    }

    /*这里一共有四种方案，积分两种，LPF两种：
    1. Holtz03原版是用u_off_original_lpf_input过LPF，
    2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
    3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
    4. 我还提出了用饱和时间去做校正的方法*/

    // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
    #if BOOL_USE_METHOD_INTEGRAL_INPUT
        #define INTEGRAL_INPUT(X)   FE.htz.u_off_saturation_time_correction[X] // exact offset calculation for compensation
        // FE.htz.sat_time_offset[X]
        // #define INTEGRAL_INPUT(X)   FE.htz.accumulated__u_off_saturation_time_correction[X]
        // #define INTEGRAL_INPUT(X)   FE.htz.u_off_original_lpf_input[X]

        long int local_sum = FE.htz.negative_cycle_in_count[0] + FE.htz.positive_cycle_in_count[0] + FE.htz.negative_cycle_in_count[1] + FE.htz.positive_cycle_in_count[1];
        if(local_sum>0){
            FE.htz.gain_off = imife_realtime_gain_off * HOLTZ_2002_GAIN_OFFSET / ((REAL)local_sum*CL_TS);
        }
        FE.htz.u_offset[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        FE.htz.u_offset[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);

        // 想法是好的，但是没有用，因为咱们要的不是磁链正负半周的饱和时间最大和最小相互抵消；实际上观察到正负半周是饱和时间的正弦包络线的正负半周，频率比磁链频率低多啦！
        // FE.htz.accumulated__u_off_saturation_time_correction[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.htz.accumulated__u_off_saturation_time_correction[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // if(FE.htz.sign__u_off_saturation_time_correction[0]>0){
        //     FE.htz.u_offset[0] = FE.htz.accumulated__u_off_saturation_time_correction[0];
        //     FE.htz.sign__u_off_saturation_time_correction[0] = 0;
        // }
        // if(FE.htz.sign__u_off_saturation_time_correction[1]>0){
        //     FE.htz.u_offset[1] = FE.htz.accumulated__u_off_saturation_time_correction[1];
        //     FE.htz.sign__u_off_saturation_time_correction[1] = 0;
        // }

        // 本来想二重积分消除交流波动，但是我发现饱和时间误差波动的原因是上下饱和时间清零不及时导致的。但是哦……好像也没有有效地及时清零的方法，所以还是试试双重积分吧：
        // FE.htz.u_offset_intermediate[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.htz.u_offset_intermediate[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // FE.htz.u_offset[0] += CL_TS * FE.htz.u_offset_intermediate[0];
        // FE.htz.u_offset[1] += CL_TS * FE.htz.u_offset_intermediate[1];
    #endif

    // 低通
    #if BOOL_USE_METHOD_LPF_INPUT
        #define LPF_INPUT(X) FE.htz.u_off_direct_calculated[X]
        // #define LPF_INPUT(X) FE.htz.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
        #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
        FE.htz.u_offset[0] = _lpf( LPF_INPUT(0), FE.htz.u_offset[0], TAU_OFF_INVERSE);
        FE.htz.u_offset[1] = _lpf( LPF_INPUT(1), FE.htz.u_offset[1], TAU_OFF_INVERSE);
    #endif

    // 差分
    // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
    // FE.htz.u_offset[0] += DIFFERENCE_INPUT(0);
    // FE.htz.u_offset[1] += DIFFERENCE_INPUT(1);

    // 直通
    // FE.htz.u_offset[0] = FE.htz.u_off_direct_calculated[0];
    // FE.htz.u_offset[1] = FE.htz.u_off_direct_calculated[1];

    // FE.htz.psi_1_nonSat[0] = FE.htz.psi_1[0];
    // FE.htz.psi_1_nonSat[1] = FE.htz.psi_1[1];
    // FE.htz.psi_2_nonSat[0] = FE.htz.psi_2[0];
    // FE.htz.psi_2_nonSat[1] = FE.htz.psi_2[1];

    FE.htz.psi_2_prev[0] = FE.htz.psi_2[0];
    FE.htz.psi_2_prev[1] = FE.htz.psi_2[1];
}
void VM_Saturated_ExactOffsetCompensation(){
}
void VM_Saturated_ExactOffsetCompensation_WithParallelNonSaturatedEstimator(){
    /* 重点是正常情况下，咱们用VM_Saturated_ExactOffsetCompensation的 u_offset 以及磁链估计结果 psi_2 直接去校正第二个平行的观测器 ParallelNonSaturatedEstimator，
        但是一旦出现t_alpha,sat,min>0 且 t_bet,sat,max>0的情况，这时就以第二个平行观测器在一个周期内观测得到的磁链的幅值结果作为第一个观测器的限幅值，
        同时，在那个周期内，第一个观测器也并不是没用的，而是继续作为一个 u_offset detector（？实际上不行，见下面hint）
        因为就算t_alpha,sat,min>0 且 t_bet,sat,max>0，此时仍然会因为直流偏置 u_offset 的存在而导致t_alpha,sat,min != t_bet,sat,max，它们的差值就算 u_offset 的值大小的一个 indicator。
        *当然，实际情况下可以给第二个观测器设置一个比较大的饱和值，因为磁链不可能无限变大的。

        hint：值得一提的是，如果同时上下界都饱和了，那么 psi_alpha2min + psi_alpha2max = 0，此时怎么算偏置电压 u_offset 都是零了！
        那么有意思的问题就来了，这个时候根据第二个观测器输出的磁链的 (最大值+最小值)/2 是否可以作为 u_offset？
        如果答案是 yes 的话，那么打从一开始为什么要引入 Saturation 呢？
        Saturation的作用，就是能够校正已经偏离了 t轴 的磁链观测结果。只补偿 u_offset，只是保证了磁链波形不会进一步上升（或下降），但是无法把已经偏离的磁链波形的中心移回到t轴上。

        那如果我假设磁链正负总是应该对称的，每个周期更新一次 psi_offset 和 u_offset 呢？
        psi_offset 是磁链波形正负面积的积分。 u_offset 在一定时间内是造成这 psi_offset 的磁链导数。
    */
}

/* Init functions */
void rk4_init(){

    OBSV.rk4.us[0] = 0.0;
    OBSV.rk4.us[1] = 0.0;
    OBSV.rk4.is[0] = 0.0;
    OBSV.rk4.is[1] = 0.0;
    OBSV.rk4.us_curr[0] = 0.0;
    OBSV.rk4.us_curr[1] = 0.0;
    OBSV.rk4.is_curr[0] = 0.0;
    OBSV.rk4.is_curr[1] = 0.0;
    OBSV.rk4.us_prev[0] = 0.0;
    OBSV.rk4.us_prev[1] = 0.0;
    OBSV.rk4.is_prev[0] = 0.0;
    OBSV.rk4.is_prev[1] = 0.0;
}
void observer_init(){
    init_FE_htz();

    int i;
    for(i=0; i<2; ++i){
        /* Ohtani 1990 */
        // FE.exact.psi_2_max[i] = +(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        // FE.exact.psi_2_min[i] = -(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);

        FE.exact.psi_2_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output2_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output2_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);

        FE.exact.first_time_enter_top[i] = TRUE;
        FE.exact.first_time_enter_butt[i] = TRUE;
    }


    FE.harnefors.lambda = GAIN_HARNEFORS_LAMBDA;
}
#endif
