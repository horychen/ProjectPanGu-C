#ifndef REGULATOR_SPEEDINNERLOOP_H
#define REGULATOR_SPEEDINNERLOOP_H
#include "ACMSim.h"
/* Here is a good example to show that we user WHO_IS_USER to make your own code to be compiled */
    #if WHO_IS_USER == USER_WB
        // 吴波用：
        void InnerLoopFeedback_calc(st_pid_regulator_handle);

        #define WUBO_INCREMENTAL_PID_CURRENT TRUE

        /* WUBO */
        void _user_wubo_WC_Tuner();
        void _user_wubo_WC_Tuner_Online();
        void _user_wubo_TI_Tuner_Online();
        void _user_wubo_SpeedInnerLoop_controller(st_pid_regulator *r);
        REAL _user_wubo_WC_Tuner_Part2(REAL zeta, REAL omega_n, REAL max_CLBW_PER_min_CLBW);

        /* Valina */
        void decoupling_current_loop();

        /* For Sweeping */
        void _user_wubo_Sweeping_Command(); // 给定扫频的幅值
    #endif
#endif
