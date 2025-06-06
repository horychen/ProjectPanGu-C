#ifndef MAIN_TOOLS
#define MAIN_TOOLS

void EUREKA_GPIO_SETUP();
extern REAL iq_command_from_PC;
#define ANGLE_SHIFT_FOR_FIRST_INVERTER 0.0  // Torque Inverter
#define ANGLE_SHIFT_FOR_SECOND_INVERTER 0.0 // Suspension Inverter

void axis_basic_setup(int axisCnt);
void compute_CLA_task_vectors();
void init_spi();
void handle_interrupts();
void main_loop();
void DeadtimeCompensation(REAL Current_U, REAL Current_V, REAL Current_W, REAL CMPA[], REAL CMPA_DBC[]);
void test_pwm_output();

void voltage_commands_to_pwm();
void voltage_measurement_based_on_eCAP();
void measurement_position_count_axisCnt0();
void measurement_position_count_axisCnt1();
void measurement_enc();
void measurement_current_axisCnt0();
void measurement_current_axisCnt1();
#if WHO_IS_USER == USER_QIAN
    void measurement_sensor_coil();
#endif

void control_two_motor_position();
void control_single_motor_position();
void run_shank_loop();
void run_hip_loop();
void run_both_loop();
void run_impedance_control();

void DISABLE_PWM_OUTPUT();
void ENABLE_PWM_OUTPUT(int positionLoopType);

void read_count_from_cpu02_dsp_cores_2();
void write_RPM_to_cpu02_dsp_cores_2();
void cla_test_codes();
void test_ipc_tocpu02();

#endif
