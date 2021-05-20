/*
 * ECaptureVoltage.h
 *
 *  Created on: May 19, 2021
 *      Author: horyc
 */

#ifndef THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_
#define THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_

void InitECapture();
void InitECaptureContinuousMode();
void do_enhanced_capture();

void ecap_moving_average();

#endif /* THE_DRIVER_LAYER_ECAPTUREVOLTAGE_H_ */
