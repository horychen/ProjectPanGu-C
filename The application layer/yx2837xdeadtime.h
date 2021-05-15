/*
 * yx2837xdeadtime.h
 *
 *  Created on: 2021Äê1ÔÂ12ÈÕ
 *      Author: xinyuan
 */

#ifdef USE_YX_DEADTIME_COMPENSATION

#ifndef MOTOR_CONTROL_20170601_MPC_RAM_THE_APPLICATION_LAYER_YX2837XDEADTIME_H_
#define MOTOR_CONTROL_20170601_MPC_RAM_THE_APPLICATION_LAYER_YX2837XDEADTIME_H_

#include "main_config.h"
#define  UNIT_US_DEADTIME     0.000001
#define  UNIT_K_DEADTIME     1000
#define  M_PI_3              1.0471975511667
#define  SQRT_3              0.86602540378

typedef struct     { float  Ualpha;            // Input: reference alpha-axis phase voltage
                      float  Ubeta;         // Input: reference beta-axis phase voltage
                      float  Ta;                // Output: reference phase-a switching function
                      float  Tb;                // Output: reference phase-b switching function
                      float  Tc;                // Output: reference phase-c switching function
                } SVGENDQ;
typedef struct     {
                      float  Ualpha_compensation;                // Output: reference phase-a switching function
                      float  Ubeta_compensation;                // Output: reference phase-b switching function
                      float  Electrical_angle;                  //0-2pi for electrical angle
                      float  predictive_d_current;
                      float  predictive_q_current;
                      float  predictive_d_current_old;
                      float  predictive_q_current_old;
                      float  id_Fbk;
                      float  iq_Fbk;
                      int    bandwidth_deadtime;
                      float  wn_deadtime;
                      float  K_lo_1; //observer coefficient
                      float  K_lo_2; //observer coefficient
                      float  Ud_compensation;
                      float  Uq_compensation;
                      double elec_speed;
                      float  Sine;
                      float  Cosine;
                      float  Ud_p;
                      float  Uq_p;
                      float  Ud_i;
                      float  Uq_i;
} DEADTIME;
#define DEADTIME_COMPENSATION_INIT(v)  \
    v.Ualpha_compensation=0;                \
    v.Ubeta_compensation=0;                \
    v.predictive_d_current=0;                 \
    v.predictive_q_current=0;                 \
    v.predictive_d_current_old=0;                 \
    v.predictive_q_current_old=0;                 \
    v.Ud_compensation=0;                 \
    v.Uq_compensation=0;       \
    v.Ud_i=0;                   \
    v.Uq_i=0;                   \
    v.bandwidth_deadtime=1000;  //bandwidth is set to 1000hz initially;larger than 2000, the system is unstable!
void SVGEN_Drive(SVGENDQ* ptrV);
void Deadtime_compensation_observer(DEADTIME* ptrV);
void Deadtime_compensation(PARK* ptrV2,DEADTIME* ptrV);

#endif /* MOTOR_CONTROL_20170601_MPC_RAM_THE_APPLICATION_LAYER_YX2837XDEADTIME_H_ */
#endif
