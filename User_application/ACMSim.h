#ifndef ACMSIM_H
#define ACMSIM_H

/* Try to use Holtz03 developed for IM for PMSM */
// #define U_MOTOR_R       PMSM_RESISTANCE
// #define U_MOTOR_LD      PMSM_D_AXIS_INDUCTANCE
// #define U_MOTOR_LQ      PMSM_Q_AXIS_INDUCTANCE
// #define U_MOTOR_KE      PMSM_PERMANENT_MAGNET_FLUX_LINKAGE
// #define U_MOTOR_RREQ    0
// #define IM_FLUX_COMMAND_SINE_PART 0
// #define HOLTZ_2002_GAIN_OFFSET 20

#define PC_SIMULATION FALSE
#include "typedef.h"
#include "math.h"
#include "stdlib.h"

#define abs use_fabs_instead_or_you_will_regret
#define printf DoNotCallPrintFunctionInDSP

#if PC_SIMULATION == FALSE

/* SYSTEM Configuration */
#define NUMBER_OF_AXES 2
#define NUMBER_OF_DSP_CORES 2 // 1 or 2

/* ePWM CONFIGURATION */
#define SYSTEM_PROGRAM EPWM1ISR
//        #define SYSTEM_PWM_KILO_FREQUENCY               10 // 10kHz
//        #define SYSTEM_CARRIER_PERIOD              (100000/SYSTEM_PWM_KILO_FREQUENCY) // = TBCLK (100 MHz) / 10 (kHz) = 100e6/1e4 = 100e2 = 1e4 cnt 锟斤拷应一锟斤拷锟截诧拷锟斤拷锟斤拷
//        #define SYSTEM_TBPRD                       (SYSTEM_CARRIER_PERIOD/2)
#define SYSTEM_CARRIER_PERIOD 10000 // =100000/10, max is 100kHz
#define SYSTEM_TBPRD 5000           // =SYSTEM_CARRIER_PERIOD/2
// #define SYSTEM_PWM_DEADTIME_CNT            500 // 500 锟斤拷 TBCLK = EPWMCLK/1 = 100 MHz
#define SYSTEM_PWM_DEADTIME_CNT 500
#define SYSTEM_PWM_DEADTIME_COMPENSATION 500
// #define SYSTEM_PWM_DEADTIME_COMPENSATION   483
// Td = 5.0us; Ton = 0.15us; Toff = 0.32us;
// TM = Td + Ton -Toff;
// CJH: (5.0 + 0.15 - 0.32 )*1e-6 * TBCLK (100 MHz) = 483
#define SYSTEM_MAX_PWM_DUTY_LIMATATION 0.96
#define SYSTEM_MIN_PWM_DUTY_LIMATATION 0.04
#define SYSTEM_PWM_UDC_UTILIZATION SYSTEM_MAX_PWM_DUTY_LIMATATION

/* eCAP CONFIGURATION */ /* eCAP锟斤拷锟侥诧拷锟饺ｏ拷锟津波峰？锟叫讹拷锟斤拷锟节硷拷锟斤拷值锟斤拷20000锟斤拷锟斤拷锟揭伙拷锟斤拷锟街わ拷锟絋BCLK锟斤拷锟斤拷频锟剿ｏ拷锟斤拷锟斤拷200MHz锟斤拷锟斤拷锟斤拷100MHz锟斤拷 */
#define SYSTEM_PWM_INT_MAX_COUNT 20000
#define SYSTEM_PWM_INT_MAX_COUNT_INVERSE 5e-5 // = 1 / 20000
#define SYSTEM_HALF_PWM_MAX_COUNT 10000
#endif

// Everthing that is configurable is in here
#include "ACMConfig.h"
#include "pid_regulator.h"
#include "shared_flux_estimator.h"
#include "im_controller.h"
#include "im_observer.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"
#include "pmsm_comm.h"
#include "Bezier.h"
#include "regulator_speedInnerLoop.h"

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
inline double sign(double x) { return (x > 0) - (x < 0); }
inline REAL signf(REAL x) { return (x > 0) - (x < 0); }
double fabs(double x);
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);
REAL PostionSpeedMeasurement_MovingAvergage(int32 QPOSCNT, st_enc *p_enc);
extern REAL one_over_six;
REAL difference_between_two_angles(REAL first, REAL second);

/* Encoder QEP TODO: should read from excel */
#define ABSOLUTE_ENCODER_SCI_SHANK 1
#define ABSOLUTE_ENCODER_SCI_HIP 2
#define ABSOLUTE_ENCODER_CAN_ID0x01 3
#define ABSOLUTE_ENCODER_CAN_ID0x03 4
#define RESOLVER_1 5
#define RESOLVER_2 6
#define ABSOLUTE_ENCODER_MD1 7
#define INCREMENTAL_ENCODER_QEP 8

#define ENCODER_TYPE ABSOLUTE_ENCODER_MD1 // ABSOLUTE_ENCODER_SCI_SHANK

#define INIT_NPP d_sim.init.npp

#if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
#define SYSTEM_QEP_PULSES_PER_REV (10000)
#define SYSTEM_QEP_REV_PER_PULSE (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#define SYSTEM_QEP_QPOSMAX (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (10000)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK) || (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
#define SYSTEM_QEP_PULSES_PER_REV (8388608)
#define SYSTEM_QEP_REV_PER_PULSE (1.1920929e-7)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)

#define SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 7365433 // 1051014 // 3494662 // ym tuned with id_cmd = 3A 2024-03-12
#define HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 340755    // ym tuned with id_cmd = 3A 2024-03-12
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01)
#define SYSTEM_QEP_PULSES_PER_REV (131072)
#define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 49476 // ym tuned with
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03)
#define SYSTEM_QEP_PULSES_PER_REV (131072)
#define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
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
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#elif ENCODER_TYPE == RESOLVER_2
#define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
#define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
#define SYSTEM_QEP_QPOSMAX (4095)                   // (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (4096)
#define SYSTEM_QEP_PULSES_PER_REV (4096 * RESOLVER_NUMBER_OF_POLE_PAIRS)                   // (10000)
#define SYSTEM_QEP_REV_PER_PULSE (0.000244140625 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
#define SYSTEM_QEP_PULSES_PER_REV (131072)      // 2^17
#define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6) // 1 / 2^17
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
#define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
#define MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 30190 // ym tuned with id_cmd = 3A, 20240308
#define MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 30091 // cjh tuned with id_cmd = 3A, 20240715
// 30144 wb tuned with id_cmd = 2A, 20240715
// MOTOR1 30190 wb tuned with id_cmd = 3A, 20240719
// MOTOR2 41668 wb tuned with id_cmd = 3A, 20240719
#endif

#endif
