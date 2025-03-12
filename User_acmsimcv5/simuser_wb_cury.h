#ifndef SIMUSER_WB_CURY_H
#define SIMUSER_WB_CURY_H

#include "ACMSim.h"

/*
Cury Code Copied from Pangu-C: AppCury.h, AppCury.h, tool.c
rearranged and modified by WuBo
*/
#if WHO_IS_USER == USER_WB

    // Files needed to be put in the Device_define.h ???
    // void EUREKA_GPIO_SETUP(); 这个函数是最开始的GPIO定义，在原始代码中存在，暂时不要动
    
    // tools
    extern REAL iq_command_from_PC;
        //当初这两个变量是认真的吗，我学完EE270是不是就知道这些在干嘛了
    #define ANGLE_SHIFT_FOR_FIRST_INVERTER 0.0  // Torque Inverter
    #define ANGLE_SHIFT_FOR_SECOND_INVERTER 0.0 // Suspension Inverter

    // void handle_interrupts();
    // void main_loop();
    // void DeadtimeCompensation(REAL Current_U, REAL Current_V, REAL Current_W, REAL CMPA[], REAL CMPA_DBC[]);
    // void test_pwm_output();

    // void voltage_commands_to_pwm();
    // void voltage_measurement_based_on_eCAP();
    // void measurement_position_count_axisCnt0();
    // void measurement_position_count_axisCnt1();
    // void measurement_enc();
    // void measurement_current_axisCnt0();
    // void measurement_current_axisCnt1();



    /* Step 1 : Cury 运动轨迹生成*/
    #define BEZIER_ORDER 4
    #define BEZIER_TRACE_SIZE 100
    void run_iecon_main(Uint64 t);
    void reset_position();
    void linear_controller(REAL t);
    void sinusoidal_controller(REAL t);
    void calc_theta_from_height(REAL height);
    REAL hip_shank_angle_to_can(REAL angle, int type);

    /* Step 2 : CAN编码器到实物位置的线性插值映射 */
    #define CAN_QMAX 131072 // 功望编码器的最大值
    #define CAN_QMAX_INV 7.62939453125e-06 // 功望编码器的最大值的倒数
    #define HIP_MIN   0.2269
    #define HIP_MAX   0.5585
    #define SHANK_MIN 0.4538
    #define SHANK_MAX 1.1170
    #define CAN01_MIN 48000
    #define CAN01_MAX 41500
    #define CAN03_MIN 58000
    #define CAN03_MAX 46000

    /* Step 3 : 位置环，纯控制算法代码 */
    #define HIP_TYPE 0
    #define SHANK_TYPE 1
    void Cury_call_position_loop_controller();
    void control_two_motor_position();
    void control_single_motor_position();
    void run_shank_loop();
    void run_hip_loop();
    void run_both_loop();
    void run_impedance_control();
    
    #define NO_POSITION_CONTROL 0
    #define TWOMOTOR_POSITION_CONTROL 1
    #define SINGLE_POSITION_CONTROL 2
    #define SHANK_LOOP_RUN 3
    #define HIP_LOOP_RUN 4
    #define BOTH_LOOP_RUN 5
    #define IMPEDANCE_CONTROL 6

    /* Cury Controller from old files*/
    #define INIT_THETA1 0.0
    #define INIT_THETA2 0.0
    typedef struct{
        int  CONTROLLER_TYPE;
        REAL IECON_HEIGHT;
        REAL L1;
        REAL L2;
        REAL theta1, theta2;  // unit [rad]   theta1: hip ; theta2: knee
        REAL dot_theta1, dot_theta2;
        REAL T;               // unit [s]
        REAL T_inv;           // unit [Hz]
        REAL height_limit[2]; // unit [m]
        REAL C[BEZIER_ORDER][2];
        int order;
        REAL bezier_trace[BEZIER_TRACE_SIZE + 1][2];
        REAL legBouncingSpeed;
        REAL hipBouncingFreq;
        REAL legBouncingIq;
        REAL hipBouncingIq;
        BOOL bool_TEMP;
    } CURYCONTROLLER;
    extern CURYCONTROLLER cury_controller;


#endif
#endif // SIMUSER_WB_CURY_H


