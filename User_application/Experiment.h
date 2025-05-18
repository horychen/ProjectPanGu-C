/*
 * Experiment.h
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */

#ifndef THE_DRIVER_LAYER_EXPERIMENT_H_
#define THE_DRIVER_LAYER_EXPERIMENT_H_

#define AS_LOAD_MOTOR_CONST 1
#define AS_LOAD_MOTOR_RAMP  2
#define NSOAF_LOW_SPEED_OPERATION  3
#define NSOAF_HIGH_SPEED_OPERATION 4
#define NSOAF_RAMP_SPEED_OPERATION 41
#define XCUBE_TaTbTc_DEBUG_MODE    5
#define SLESSINV_CONST_LOAD_PAA  10

void init_ADC_scale_and_offset();
void init_experiment_overwrite();
void runtime_command_and_tuning();
void init_experiment_PLACE_gain_and_offset();

/* NSOAF */

#define SLOW_REVERSAL_RATE 50 // Park.Sul2014

void short_stopping_at_zero_speed();
void slow_speed_reversal();
void low_speed_operation_init();
void slow_speed_reversal_tuning();
void zero_speed_stopping_tuning();
void zero_speed_stopping();
void high_speed_operation_init();
void high_speed_operation_tuning();
void high_speed_operation();
void ramp_speed_operation_init();
void ramp_speed_operation();
void ramp_speed_operation_tuning();


#endif /* THE_DRIVER_LAYER_EXPERIMENT_H_ */
