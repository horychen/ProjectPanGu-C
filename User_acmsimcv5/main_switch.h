#ifndef user_defined_functions_H
#define user_defined_functions_H
#include "ACMSim.h"

#define USER_CJH    101976
#define USER_XM     102209
#define USER_BEZIER 224
#define USER_GZT    2021531030
#define USER_WB     2023231051
#define USER_YZZ    2023231060

#define MODE_SELECT_PWM_DIRECT         1
#define MODE_SELECT_VOLTAGE_OPEN_LOOP  11
#define MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE 2
#define MODE_SELECT_FOC                      3
#define MODE_SELECT_FOC_SENSORLESS           31
#define MODE_SELECT_INDIRECT_FOC             32
#define MODE_SELECT_VELOCITY_LOOP            4
#define MODE_SELECT_VELOCITY_LOOP_SENSORLESS 41
#define MODE_SELECT_TESTING_SENSORLESS       42
#define MODE_SELECT_VELOCITY_LOOP_WC_TUNER   43
#define MODE_SELECT_Marino2005               44
#define MODE_SELECT_POSITION_LOOP            5
#define MODE_SELECT_COMMISSIONING            9

struct DebugExperiment{
    int use_first_set_three_phase;
    long error;
    long who_is_user;
    long mode_select;
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
    REAL vvvf_voltage;
    REAL vvvf_frequency;
    int positionLoopType;
};
extern struct DebugExperiment debug;


/* USER: Macros */

/* User Specified Functions */
void _user_init(); // 非常重要的初始化
void _user_time_varying_parameters();// 时变参数
void _user_observer();
void _user_controller_wubo(); // REAL Vdc, REAL theta_d_elec, REAL varOmega, REAL varTheta, REAL iDQ[2], REAL cmd_uDQ[ 
void _user_onlyFOC(); // only current loop works
void _user_virtual_ENC(); // 使用内部给定角度代替码盘位置反馈

// 指令和负载
extern struct SweepFreq{
    REAL time;
    REAL freq_step_size;
    REAL current_freq;
    REAL      current_freq_end_time;
    REAL last_current_freq_end_time;
} sf;
void _user_pmsm_commands();
    // REAL *p_set_rpm_speed_command, REAL *p_set_iq_cmd, REAL *p_set_id_cmd, int *p_set_current_loop, int *p_flag_overwrite_theta_d, REAL *p_Overwrite_Current_Frequency
void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);

void main_switch(long mode_select);
void FOC_with_vecocity_control(REAL theta_d_elec,
         REAL varOmega,
         REAL cmd_varOmega,
         REAL cmd_iDQ[2],
         REAL iAB[2]);

#endif