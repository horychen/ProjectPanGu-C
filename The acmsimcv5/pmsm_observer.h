#ifndef ADD_PMSM_OBSERVER_H
#define ADD_PMSM_OBSERVER_H
#if MACHINE_TYPE == 2

/* Macro for External Access Interface */
#define US(X)   rk4.us[X]
#define IS(X)   rk4.is[X]
// #define US_C(X) rk4.us_curr[X] // 当前步电压是伪概念，测量的时候，没有电压传感器，所以也测量不到当前电压；就算有电压传感器，由于PWM比较寄存器没有更新，输出电压也是没有变化的。
#define IS_C(X) rk4.is_curr[X]
#define US_P(X) rk4.us_prev[X]
#define IS_P(X) rk4.is_prev[X]


/* Natural Speed Observer with active flux concept Chen2020 */
struct Chen20_NSO_AF{
    REAL xIq;
    REAL xOmg;
    REAL xTL;
    REAL x[6];
    REAL xBest[3]; // the actual 3 states
    // REAL best_xTL;
    // REAL best_output_error;
    // REAL best_index;
    // REAL solved_xOmg;

    REAL KP;
    REAL KI;
    REAL KD;

    REAL ActiveFlux_KP;
    REAL ActiveFlux_KI;

    // output
    REAL active_flux_ab[2];
    REAL theta_d;

    REAL output_error; // \tilde i_q

    REAL active_power_real;
    REAL active_power_est;
    REAL active_power_error;

    // REAL gamma_TL;
    REAL active_flux_ampl;
    REAL xTem;

    /* Select Signals from Block Diagram*/
    REAL LoadTorquePI;
    REAL load_torque_pid_output;
    REAL q_axis_voltage;

    /* Active Flux Estimator */
    REAL afest_states[4];
    REAL psi_1[2];
    REAL u_offset[2];
    REAL cosT;
    REAL sinT;
};
extern struct Chen20_NSO_AF nsoaf;
void hgo4eemf_init();


/* Farza 2009 HGO */
struct FARZA09_EEMF_HGO{
    REAL xPsi[2];
    REAL xEemf[2];
    REAL xOmg;
    REAL xGammaOmg;
    REAL xPhi[4];

    // output
    REAL theta_d;

    REAL output_error[2]; // \varepsilon

    REAL vartheta;
    REAL vartheta_inv;
};
extern struct FARZA09_EEMF_HGO hgo4eemf;
void hgo4eemf_init();


/* CJH EEMF AO Design */
struct CJH_EEMF_AO_Design{
    REAL xPsi[2];
    REAL xChi[2];
    REAL xOmg;

    // output
    REAL xEemf[2];
    REAL theta_d;

    REAL xEta[2];     // \eta = G(p)\phi\hat\omega
    REAL xVarsigma[2];

    REAL xUpsilon[2]; // \Upsilon = [G(p) \phi]
    REAL xZeta[2];

    REAL output_error[2]; // \varepsilon
    REAL effective_output_error[2]; // \varepsilon_\mathrm{eff}

    REAL k1;
    REAL k2;
    REAL gamma_omg;
};
extern struct CJH_EEMF_AO_Design cjheemf;
void cjheemf_init();





/* Harnefors 2006 */
void rk4_init();
// void harnefors_init();
// void harnefors_scvm();
#define SVF_POLE_0_VALUE (2000*2*M_PI) /* 定子电阻在高速不准确，就把SVF极点加大！加到3000反而比20000要差。*/
#define SVF_POLE_0 harnefors.svf_p0 
#define SVF_C(X)   harnefors.xSVF_curr[X]
#define SVF_P(X)   harnefors.xSVF_prev[X]
#define IDQ(X)     harnefors.is_dq[X]
#define IDQ_C(X)   harnefors.is_dq_curr[X]
#define IDQ_P(X)   harnefors.is_dq_prev[X]
#define PIDQ(X)    harnefors.pis_dq[X]
struct Harnefors2006{
    REAL theta_d;
    REAL omg_elec;

    REAL deriv_id;
    REAL deriv_iq;

    // SVF for d/q current derivative
    REAL svf_p0;
    REAL xSVF_curr[2];
    REAL xSVF_prev[2];
    REAL is_dq[2];
    REAL is_dq_curr[2];
    REAL is_dq_prev[2];
    REAL pis_dq[2];
};
extern struct Harnefors2006 harnefors;



/* Common */
struct RK4_DATA{
    REAL us[2];
    REAL is[2];
    // REAL us_curr[2];
    REAL is_curr[2];
    REAL us_prev[2];
    REAL is_prev[2];

    REAL is_lpf[2];
    REAL is_hpf[2];
    REAL is_bpf[2];

    REAL current_lpf_register[2];
    REAL current_hpf_register[2];
    REAL current_bpf_register1[2];
    REAL current_bpf_register2[2];

    // REAL omg_elec; // omg_elec = npp * omg_mech
    // REAL theta_d;
};
extern struct RK4_DATA rk4;

// Call all pmsm observers at one place here
void pmsm_observers();
void pmsm_observers_init();

void active_flux_estimator();

#endif
#endif
