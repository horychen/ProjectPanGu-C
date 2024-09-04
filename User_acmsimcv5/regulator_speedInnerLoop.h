#ifndef REGULATOR_SPEEDINNERLOOP_H
#define REGULATOR_SPEEDINNERLOOP_H
#include "ACMSim.h"

// 吴波用：
void InnerLoopFeedback_calc(st_pid_regulator_handle);

#define WUBO_INCREMENTAL_PID_CURRENT FALSE

/* WUBO */
void _user_wubo_WC_Tuner();
void _user_wubo_WC_Tuner_Online();
void _user_wubo_TI_Tuner_Online();
void _user_wubo_controller();
REAL _user_wubo_WC_Tuner_Part2(REAL zeta, REAL omega_n, REAL max_CLBW_PER_min_CLBW);

/* For Sweeping */
void _user_wubo_SpeedSweeping_command(); // 给定扫频的幅值



#endif
