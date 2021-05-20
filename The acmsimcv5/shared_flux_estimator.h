
#ifndef SHARED_FLUX_ESTIMATOR_H
#define SHARED_FLUX_ESTIMATOR_H

typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);

struct ActiveFluxEstimator{
    /* Coefficients */
    REAL ActiveFlux_KP;
    REAL ActiveFlux_KI;
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
};
extern struct ActiveFluxEstimator AFEOE;


void test_flux_estimators();
void afe_init();

#endif
