#ifndef ACMSIM_H
#define ACMSIM_H

#define PC_SIMULATION FALSE
#include "math.h"
#include "stdlib.h"
#define printf DoNotCallPrintFunctionInDSP

#if PC_SIMULATION == FALSE
/*  SYSTEM Configuration -----------------------------------------------------------------------------------*/
    #define NUMBER_OF_DSP_CORES 2  // 1 or 2
    #define SYSTEM_PROGRAM_MODE 223  //223 for CJHMainISR //0 stands for MainISR ,1 stands for Motor_Parameter_Compute_ISR
    #if SYSTEM_PROGRAM_MODE==223
        /* ePWM CONFIGURATION */
        #define SYSTEM_PROGRAM                     EPWM1ISR
        #define SYSTEM_PWM_FREQUENCY               10 // 10kHz
        //        #define SYSTEM_CARRIER_PERIOD              (100000/SYSTEM_PWM_FREQUENCY) // = TBCLK (100 MHz) / 10 (kHz) = 100e6/1e4 = 100e2 = 1e4 cnt 对应一个载波周期
        //        #define SYSTEM_TBPRD                       (SYSTEM_CARRIER_PERIOD/2)
        #define SYSTEM_CARRIER_PERIOD              10000 // =100000/10
        #define SYSTEM_TBPRD                       5000 // =10000/2
        #define SYSTEM_PWM_DEADTIME_CNT            500 // 500 个 TBCLK = EPWMCLK/1 = 100 MHz
        #define SYSTEM_PWM_DEADTIME_COMPENSATION   483
            // Td = 5.0us; Ton = 0.15us; Toff = 0.32us;
            // TM = Td + Ton -Toff;
            // CJH: (5.0 + 0.15 - 0.32 )*1e-6 * TBCLK (100 MHz) = 483
        #define SYSTEM_MAX_PWM_DUTY_LIMATATION     0.96
        #define SYSTEM_MIN_PWM_DUTY_LIMATATION     0.04
        #define SYSTEM_PWM_UDC_UTILIZATION         SYSTEM_MAX_PWM_DUTY_LIMATATION

        /* eCAP CONFIGURATION */ /* eCAP最大的波谷？或波峰？中断周期计数值是20000，这进一步验证了TBCLK被降频了，不是200MHz，而是100MHz。 */
        #define SYSTEM_PWM_INT_MAX_COUNT               20000
        #define SYSTEM_PWM_INT_MAX_COUNT_INVERSE       5e-5 // = 1 / 20000
        #define SYSTEM_HALF_PWM_MAX_COUNT          10000
    #endif
#endif


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
#define PHASE_NUMBER 3 // three phase machine
#ifndef BOOL
#define BOOL int
#endif
#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif
#define ONE_OVER_2PI          0.15915494309189535 // 1/(2*pi)
#define ONE_OVER_60           0.01666666666666667
#define TWO_PI_OVER_3         2.0943951023931953
#define SIN_2PI_SLASH_3       0.86602540378443871 // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3        0.81649658092772603 // sqrt(2.0/3.0)
#define abs                   use_fabs_instead_or_you_will_regret
#define prinf                 dont_use_print_in_experiment_codes
#define ELEC_RAD_PER_SEC_2_RPM ( 60.0*ONE_OVER_2PI*CTRL.motor->npp_inv )
// #define ELEC_RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*CTRL.motor->npp) )
#define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*CTRL.motor->npp)*ONE_OVER_60 )
// #define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*CTRL.motor->npp)/60.0 )
#define M_PI_OVER_180   0.017453292519943295

#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define CLARKE_TRANS_TORQUE_GAIN_INVERSE (0.666666667)
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)

// Everthing that is configurable is in here
#include "ACMConfig.h"
#include "pid_regulator.h"
#include "shared_flux_estimator.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"
#include "pmsm_comm.h"
#include "sweep_frequency.h"


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




/* Declaration of Utility Function */
int isNumber(double x);
double sign(double x);
double fabs(double x);
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);
REAL PostionSpeedMeasurement_MovingAvergage(Uint32 QPOSCNT);
extern REAL one_over_six;
double difference_between_two_angles(double first, double second);


#endif
