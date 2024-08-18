#ifndef REGULATOR_SPEEDINNERLOOP_H
#define REGULATOR_SPEEDINNERLOOP_H
#include "ACMSim.h"

// 吴波用：
void InnerLoopFeedback_calc(st_pid_regulator_handle);

/* WUBO */
void _user_wubo_WC_Tuner();
void _user_controller_wubo();

#endif
