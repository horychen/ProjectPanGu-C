/*
 * Experiment.h
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */

#ifndef THE_DRIVER_LAYER_EXPERIMENT_H_
#define THE_DRIVER_LAYER_EXPERIMENT_H_


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


#endif /* THE_DRIVER_LAYER_EXPERIMENT_H_ */
