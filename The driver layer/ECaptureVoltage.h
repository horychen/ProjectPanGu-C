/*
 * ECaptureVoltage.h
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */

#ifndef THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_
#define THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_

struct TestECapture {
    REAL TSt1, TSt2, TSt3, TSt4, Period1, Period2, Period3, DutyOnTime1, DutyOffTime1, DutyOnTime2, DutyOffTime2;
};
extern struct TestECapture ecapU, ecapV, ecapW;

void InitECapture();
void InitECaptureContinuousMode();
void do_enhanced_capture();

void ecap_moving_average();

#endif /* THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_ */
