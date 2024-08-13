#ifndef user_defined_functions_H
#define user_defined_functions_H
#include "ACMSim.h"

struct DebugExperiment{
    int  error;
    int  mode_select;
    // int  mode_PWM_test;
    // int  mode_position_loop;
    // int  mode_current_loop;
    // int  flag_overwrite_theta_d; 
    REAL Overwrite_Current_Frequency;
    REAL Overwrite_theta_d;
    REAL set_deg_position_command;
    REAL set_rpm_speed_command;
    REAL set_iq_command;
    REAL set_id_command;
    int INVERTER_NONLINEARITY_COMPENSATION_INIT;
    int INVERTER_NONLINEARITY;
    int SENSORLESS_CONTROL;
    int SENSORLESS_CONTROL_HFSI;
};
extern struct DebugExperiment debug;


/* USER: Macros */

#define USER_BEZIER 10
#define USER_WUBO   11
#define WHO_IS_USER USER_BEZIER
// #define WHO_IS_USER USER_WUBO


/* User Specified Functions */
void _user_init(); // 非常重要的初始化
void _user_time_varying_parameters();// 时变参数
void _user_observer(REAL iAB[2]);
void _user_controller(); // REAL Vdc, REAL theta_d_elec, REAL varOmega, REAL varTheta, REAL iDQ[2], REAL cmd_uDQ[ 

// 指令和负载
extern struct SweepFreq{
    REAL time;
    REAL freq_step_size;
    REAL current_freq;
    REAL      current_freq_end_time;
    REAL last_current_freq_end_time;
} sf;
void _user_commands();
    // REAL *p_set_rpm_speed_command, REAL *p_set_iq_cmd, REAL *p_set_id_cmd, int *p_set_current_loop, int *p_flag_overwrite_theta_d, REAL *p_Overwrite_Current_Frequency
REAL _user_load_model(void);
void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);

#endif
