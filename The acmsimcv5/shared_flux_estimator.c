#include "ACMSim.h"

/* Shared Solver and typedef */
void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    REAL k1[2], k2[2], k3[2], k4[2], xk[2];
    REAL fx[2];
    int i;

    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    (*fp)(t, x, fx); // timer.t,
    for(i=0;i<2;++i){
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }

    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<2;++i){
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }

    (*fp)(t, xk, fx); // timer.t+hs/2.,
    for(i=0;i<2;++i){
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    (*fp)(t, xk, fx); // timer.t+hs,
    for(i=0;i<2;++i){
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;
    }
}
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
void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    #define NS 5
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

void test_flux_estimators(){

    //     // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
    //     // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    //     // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    //     // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    //     // Akatsu_RrId();
    //     // simulation_only_flux_estimator();


    //     VM_Ohtani1992();
    //     VM_HuWu1998();
    //     VM_HoltzQuan2002();
    //     VM_HoltzQuan2003(); // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
    //     VM_Harnefors2003_SCVM();
    //     VM_LascuAndreescus2006();
    //     VM_Stojic2015();
    //     VM_ExactCompensation();
    //     VM_ProposedCmdErrFdkCorInFrameRho();
    //     VM_ClosedLoopFluxEstimator();
    //     // stableFME();

    //     watch.psi_2[0] = FLUX_FEEDBACK_ALPHA;
    //     watch.psi_2[1] = FLUX_FEEDBACK_BETA;
    //     watch.offset_compensation[0] = OFFSET_COMPENSATION_ALPHA;
    //     watch.offset_compensation[1] = OFFSET_COMPENSATION_BETA;
}
