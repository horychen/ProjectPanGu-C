
#ifndef SHARED_FLUX_ESTIMATOR_H
#define SHARED_FLUX_ESTIMATOR_H

typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);

#endif
