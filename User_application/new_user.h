#ifndef NEW_USER_H  // 如果没有定义 NEW_USER_H
#define NEW_USER_H  // 定义 NEW_USER_H

typedef enum {
    RESET_MODE_SOFT,  // 保持配置参数 _Reset_CTRL_except_Controller_Param();
    RESET_MODE_HARD,  // 恢复出厂设置 _Reset_CTRL_ALL();
    RESET_MODE_SAFE   // 进入安全状态后重置 _Reset_Controller_Param();
} st_reset_mode;
st_reset_mode resetMode;
void _user_ResetController(st_reset_mode mode);
void _Reset_CTRL_except_Controller_Param();
void _Reset_CTRL_ALL();
void _Reset_Controller_Param();

void user_init_axis(int axisCnt);
void measurement_position_count_axisCnt0(); // 对四个电机采取不同的axisCnt函数，而不是循环——方便处理当驱动对象为4台不同电机时的情况
void measurement_position_count_axisCnt1();
void measurement_position_count_axisCnt2();
void measurement_position_count_axisCnt3();
void measurement_enc();
void write_RPM_to_cpu02_dsp_cores_2();

void user_routine_init_pwm_output();
void user_routine_read_from_cpu02();
void user_routine_debug_switch();
void user_routine_disable_pwm_output();
REAL user_routine_enable_pwm_output(); // 返回四台逆变器的三相占空比

#endif // NEW_USER_H

