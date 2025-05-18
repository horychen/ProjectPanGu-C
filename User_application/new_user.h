#ifndef NEW_USER_H
#define NEW_USER_H




/* Basic User Config */
// #define NUMBER_OF_AXES 4 // This marco should be canceled since we have AXIS_ENABLE?
// #define AXIS1_ENABLE TRUE
// #define AXIS2_ENABLE TRUE
// #define AXIS3_ENABLE TRUE
// #define AXIS4_ENABLE TRUE


/* Reset Operation */
//TODO: 当用户设置完数据后，soft reset应该记忆上一次user使用的controller数据，为了做实验方便
typedef enum{
    RESET_MODE_SOFT,  // 保持配置参数 _Reset_CTRL_except_Controller_Param();
    RESET_MODE_HARD,  // 恢复出厂设置 _Reset_CTRL_ALL();
    RESET_MODE_SAFE   // 进入安全状态后重置 _Reset_Controller_Param();
} st_reset_mode;
void _user_ResetController(st_reset_mode mode);
void _Reset_CTRL_except_Controller_Param();
void _Reset_CTRL_ALL();
void _Reset_Controller_Param();



/* Functional Function */
void user_init_axis(int axisCnt);
void measurement_position_count_axisCnt0();
void measurement_position_count_axisCnt1();
void measurement_enc();
void write_RPM_to_cpu02_dsp_cores_2();


/* Routine Function */
void user_routine_init_pwm_output();
void user_routine_read_from_cpu02();
void user_routine_debug_switch();
void user_routine_disable_pwm_output();
void user_routine_enable_pwm_output();

#endif
