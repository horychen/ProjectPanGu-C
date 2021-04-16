#ifndef ACMSIM_H
#define ACMSIM_H

#define PC_SIMULATION FALSE
#include "math.h"
#include "stdlib.h"
#define printf DoNotCallPrintFunctionInDSP

// 重定义变量类型
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef          int int16;
typedef unsigned int Uint16;
typedef          long int32;
typedef unsigned long Uint32;
typedef          long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;
#endif
typedef float32 REAL;
// typedef float64 REAL;

/* Constants */
#define CONST_PI_OVER_180 (0.0174533)
#define CONST_180_OVER_PI (57.2958)
#define CONST_1_OVER_SQRT3 (0.57735)

/* Macro for Park transformation*/
#define AB2M(A, B, COS, SIN)  ( (A)*COS  + (B)*SIN )
#define AB2T(A, B, COS, SIN)  ( (A)*-SIN + (B)*COS )
#define MT2A(M, T, COS, SIN)  ( (M)*COS - (T)*SIN )
#define MT2B(M, T, COS, SIN)  ( (M)*SIN + (T)*COS )

/* Macro for two-phase Amplitude-invariant Clarke transformation*/
#define UV2A_AI(U, V) ( U )
#define UV2B_AI(U, V) ( (U + 2*(V)) * CONST_1_OVER_SQRT3 )
#define AB2U_AI(A, B) ( ( A ) )
#define AB2V_AI(A, B) ( ( (A)*-0.5 + (B)*0.866 ) )
#define AB2W_AI(A, B) ( ( (A)*-0.5 + (B)*-0.866 ) )

// Macro for Power-invariant inverse Clarke transformation
#define AB2U_PI(A, B) ( 0.816496580927726 * ( A ) )
#define AB2V_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*0.8660254037844387 ) )
#define AB2W_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*-0.8660254037844385 ) )

/* General Constants */
#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif
#define ONE_OVER_2PI          0.15915494309189535 // 1/(2*pi)
#define TWO_PI_OVER_3         2.0943951023931953
#define SIN_2PI_SLASH_3       0.86602540378443871 // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3        0.81649658092772603 // sqrt(2.0/3.0)
#define abs                   use_fabs_instead_or_you_will_regret
#define prinf                 dont_use_print_in_experiment_codes
#define ELEC_RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*CTRL.motor->npp) )
#define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*CTRL.motor->npp)/60.0 )
#define M_PI_OVER_180   0.017453292519943295

#define PHASE_NUMBER 3 // three phase machine


// Everthing that is configurable is in here
#include "ACMConfig.h"
#include "pid_regulator.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"
#include "pmsm_comm.h"
#include "sweep_frequency.h"


/* Declaration of Utility Function */
int isNumber(double x);
double sign(double x);
double fabs(double x);
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);
REAL PostionSpeedMeasurement_MovingAvergage(Uint32 QPOSCNT);

struct GlobalWatch
{
    #if MACHINE_TYPE % 10 == 1
        // induction motor
        double offset[2];
        double offset_compensation[2];
        double psi_2[2];
    #elif MACHINE_TYPE % 10 == 2
        // PM motor
    #endif
};
extern struct GlobalWatch watch;


/* Below is moved originally from ACMConfig.h */

#define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
#define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (MOTOR_RATED_CURRENT_RMS*1.414) )
#define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / MOTOR_NUMBER_OF_POLE_PAIRS )
#define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/MOTOR_NUMBER_OF_POLE_PAIRS/2/3.1415926*60) )

// #define SPEED_LOOP_PID_PROPORTIONAL_GAIN 0.00121797
// #define SPEED_LOOP_PID_INTEGRAL_TIME_CONSTANT (1/312.3)
// #define SPEED_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define SPEED_LOOP_LIMIT_NEWTON_METER (1*MOTOR_RATED_TORQUE)
#define SPEED_LOOP_LIMIT_AMPERE       (1.414*MOTOR_RATED_CURRENT_RMS)

// #define CURRENT_LOOP_PID_PROPORTIONAL_GAIN 9.2363 // (CL_TS_INVERSE*0.1*PMSM_D_AXIS_INDUCTANCE) // 9.2363
// #define CURRENT_LOOP_PID_INTEGRAL_TIME_CONSTANT (1/352.143)
// #define CURRENT_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define CURRENT_LOOP_LIMIT_VOLTS (200)

#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define CLARKE_TRANS_TORQUE_GAIN_INVERSE (0.666666667)
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)
#endif
