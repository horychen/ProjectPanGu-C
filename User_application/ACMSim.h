#ifndef ACMSIM_H
#define ACMSIM_H

/* Try to use Holtz03 developed for IM for PMSM */
//#define U_MOTOR_R       PMSM_RESISTANCE
//#define U_MOTOR_LD      PMSM_D_AXIS_INDUCTANCE
//#define U_MOTOR_LQ      PMSM_Q_AXIS_INDUCTANCE
//#define U_MOTOR_KE      PMSM_PERMANENT_MAGNET_FLUX_LINKAGE
//#define U_MOTOR_RREQ    0
//#define IM_FLUX_COMMAND_SINE_PART 0
//#define HOLTZ_2002_GAIN_OFFSET 20


#define PC_SIMULATION FALSE
#include "math.h"
#include "stdlib.h"
#define printf DoNotCallPrintFunctionInDSP

#if PC_SIMULATION == FALSE
    extern int axisCnt;
    #define NUMBER_OF_AXES 2

/*  SYSTEM Configuration -----------------------------------------------------------------------------------*/
    #define NUMBER_OF_DSP_CORES 2  // 1 or 2
    #define SYSTEM_PROGRAM_MODE 223  //223 for CJHMainISR
    #if SYSTEM_PROGRAM_MODE==223
        /* ePWM CONFIGURATION */
        #define SYSTEM_PROGRAM                     EPWM1ISR
        //        #define SYSTEM_PWM_KILO_FREQUENCY               10 // 10kHz
        //        #define SYSTEM_CARRIER_PERIOD              (100000/SYSTEM_PWM_KILO_FREQUENCY) // = TBCLK (100 MHz) / 10 (kHz) = 100e6/1e4 = 100e2 = 1e4 cnt 锟斤拷应一锟斤拷锟截诧拷锟斤拷锟斤拷
        //        #define SYSTEM_TBPRD                       (SYSTEM_CARRIER_PERIOD/2)
        #define SYSTEM_CARRIER_PERIOD              10000 // =100000/10, max is 100kHz
        #define SYSTEM_TBPRD                       5000  // =SYSTEM_CARRIER_PERIOD/2
        //#define SYSTEM_PWM_DEADTIME_CNT            500 // 500 锟斤拷 TBCLK = EPWMCLK/1 = 100 MHz
        #define SYSTEM_PWM_DEADTIME_CNT            200
        #define SYSTEM_PWM_DEADTIME_COMPENSATION   200
            // #define SYSTEM_PWM_DEADTIME_COMPENSATION   483
            // Td = 5.0us; Ton = 0.15us; Toff = 0.32us;
            // TM = Td + Ton -Toff;
            // CJH: (5.0 + 0.15 - 0.32 )*1e-6 * TBCLK (100 MHz) = 483
        #define SYSTEM_MAX_PWM_DUTY_LIMATATION     0.96
        #define SYSTEM_MIN_PWM_DUTY_LIMATATION     0.04
        #define SYSTEM_PWM_UDC_UTILIZATION         SYSTEM_MAX_PWM_DUTY_LIMATATION

        /* eCAP CONFIGURATION */ /* eCAP锟斤拷锟侥诧拷锟饺ｏ拷锟津波峰？锟叫讹拷锟斤拷锟节硷拷锟斤拷值锟斤拷20000锟斤拷锟斤拷锟揭伙拷锟斤拷锟街わ拷锟絋BCLK锟斤拷锟斤拷频锟剿ｏ拷锟斤拷锟斤拷200MHz锟斤拷锟斤拷锟斤拷100MHz锟斤拷 */
        #define SYSTEM_PWM_INT_MAX_COUNT               20000
        #define SYSTEM_PWM_INT_MAX_COUNT_INVERSE       5e-5 // = 1 / 20000
        #define SYSTEM_HALF_PWM_MAX_COUNT          10000
    #endif
#endif


// 锟截讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef          int int16;
typedef unsigned int Uint16;
typedef          long int32;
typedef unsigned long Uint32;
typedef          long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef double float64;
typedef long double float128;
#endif
typedef float32 REAL;

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

#define UVW2A_AI(U, V, W) ( 0.666667 * U - 0.333333 * V - 0.333333 * W )
#define UVW2B_AI(U, V, W) ( 0 * U + (V - W) * CONST_1_OVER_SQRT3 )
#define UVW2G_AI(U, V, W) ( 0.333333 * (U + V + W))

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
#define ELEC_RAD_PER_SEC_2_RPM ( 60.0*ONE_OVER_2PI*(*CTRL).motor->npp_inv )
// #define ELEC_RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*(*CTRL).motor->npp) )
#define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*(*CTRL).motor->npp)*ONE_OVER_60 )
// #define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*(*CTRL).motor->npp)/60.0 )
#define M_PI_OVER_180   0.017453292519943295

#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define CLARKE_TRANS_TORQUE_GAIN_INVERSE (0.666666667)
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)

// Everthing that is configurable is in here
#include "ACMConfig.h"
#include "pid_regulator.h"
#include "shared_flux_estimator.h"
#include "im_controller.h"
#include "im_observer.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"
#include "pmsm_comm.h"
#include "sweep_frequency.h"


// Header for global_variabels_definition.c
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
REAL PostionSpeedMeasurement_MovingAvergage(int32 QPOSCNT, st_enc *p_enc);
extern REAL one_over_six;
double difference_between_two_angles(double first, double second);








/* Encoder QEP TODO: should read from excel */
#define ABSOLUTE_ENCODER_SCI_SHANK 1
#define ABSOLUTE_ENCODER_SCI_HIP  2
#define ABSOLUTE_ENCODER_CAN_ID0x01 3
#define ABSOLUTE_ENCODER_CAN_ID0x03 4
#define RESOLVER_1 5
#define RESOLVER_2 6
#define ABSOLUTE_ENCODER_MD1 7
#define INCREMENTAL_ENCODER_QEP 8

#define ENCODER_TYPE ABSOLUTE_ENCODER_MD1 // ABSOLUTE_ENCODER_SCI_SHANK

#if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
    #define SYSTEM_QEP_PULSES_PER_REV (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
    #define SYSTEM_QEP_QPOSMAX (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (10000)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK) || (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
    #define SYSTEM_QEP_PULSES_PER_REV (8388608)
    #define SYSTEM_QEP_REV_PER_PULSE (1.1920929e-7)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)

    #define SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 7365433//1051014 // 3494662 // ym tuned with id_cmd = 3A 2024-03-12
    #define HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 340755 // ym tuned with id_cmd = 3A 2024-03-12
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01)
    #define SYSTEM_QEP_PULSES_PER_REV (131072)
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 49476 // ym tuned with
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03)
    #define SYSTEM_QEP_PULSES_PER_REV (131072)
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 51203 // ym tuned with id_cmd = 4A 2024-02-27
#elif ENCODER_TYPE == RESOLVER_1
    #define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
    #define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
    #define SYSTEM_QEP_QPOSMAX (65535)                  // (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (65536)
    #define SYSTEM_QEP_PULSES_PER_REV (65536 * RESOLVER_NUMBER_OF_POLE_PAIRS)                     // (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (1.52587890625e-05 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#elif ENCODER_TYPE == RESOLVER_2
    #define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
    #define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
    #define SYSTEM_QEP_QPOSMAX (4095)                   // (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (4096)
    #define SYSTEM_QEP_PULSES_PER_REV (4096 * RESOLVER_NUMBER_OF_POLE_PAIRS)                   // (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (0.000244140625 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
    #define SYSTEM_QEP_PULSES_PER_REV (131072) // 2^17
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6) // 1 / 2^17
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 30190 // ym tuned with id_cmd = 3A, 20240308
    #define MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 41668 // cjh tuned with id_cmd = 3A, 20240715
    // 30144 wb tuned with id_cmd = 2A, 20240715
    // MOTOR1 30190 wb tuned with id_cmd = 3A, 20240719
    // MOTOR2 41668 wb tuned with id_cmd = 3A, 20240719
#endif


#endif
