// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 6;
    d_sim.init.R = 1.967709;
    d_sim.init.Ld = 0.002776;
    d_sim.init.Lq = 0.002776;
    d_sim.init.KE = 0.12618;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.0012000000000000001;
    d_sim.init.Vdc = 60;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 100000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 9.2;
    d_sim.FOC.CLBW_HZ = 372.42;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = FALSE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 6.495795228949103;
    d_sim.CL.SERIES_KI_D_AXIS = 708.8288904899135;
    d_sim.CL.SERIES_KP_Q_AXIS = 6.495795228949103;
    d_sim.CL.SERIES_KI_Q_AXIS = 708.8288904899135;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.95;

    d_sim.VL.SERIES_KP = 0.4031479997691052;
    d_sim.VL.SERIES_KI = 27.64631228851396;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 0.95;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231051;
    d_sim.user.mode_select_synchronous_motor = 4;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 200;
    d_sim.user.zeta = 1.0;
    d_sim.user.omega_n = 600;
    d_sim.user.max_CLBW_PER_min_CLBW = 0.95;
    d_sim.user.VL_FEEDBACK_KFB = 0;
    d_sim.user.bool_Null_D_Control = TRUE;
    d_sim.user.bool_apply_WC_tunner_for_speed_loop = TRUE;
    d_sim.user.bool_enable_Harnefors_back_calculation = TRUE;
    d_sim.user.Check_Harnerfors_1998_On = 0;
    d_sim.user.bool_apply_sweeping_frequency_excitation = TRUE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = TRUE;
    d_sim.user.bool_sweeping_frequency_for_current_loop_iD = FALSE;
    d_sim.user.CMD_CURRENT_SINE_AMPERE = 1.0;
    d_sim.user.CMD_SPEED_SINE_RPM = 80;
    d_sim.user.CMD_SPEED_SINE_HZ = 0;
    d_sim.user.CMD_SPEED_SINE_STEP_SIZE = 1;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_HZ_CEILING = 1000;
    d_sim.user.bool_Parameter_Mismatch_test = FALSE;
    d_sim.user.ParaMis_percent_max = 4;
    d_sim.user.ParaMis_percent_min = 0.4;
    d_sim.user.ParaMis_Mode_Select = 5;
    d_sim.user.ParaMis_OneCycleTime = 0.5;
    d_sim.user.bool_apply_HitWall_analysis = FALSE;
    d_sim.user.HitWall_max_limit_ratio = 2.0;
    d_sim.user.HitWall_min_limit_ratio = 0.1;
    d_sim.user.HitWall_high_RPM_command = 400;
    d_sim.user.HitWall_time_interval = 0.2;
    d_sim.user.Position_Loop_Kp = 10;
    d_sim.user.set_deg_position_command = 180;
    d_sim.user.Position_Output_Limit = 30;
    d_sim.user.Position_cmd_sine_frequency = 1;
    d_sim.user.Position_Loop_Ref_prev = 0;
    d_sim.user.Position_Loop_Ref_Diff = 0;
    d_sim.user.bool_use_position_feedforward_by_PosDiff = FALSE;
    d_sim.user.bool_compensation_of_Pos_CTRL_output_covergence = FALSE;
    d_sim.user.COMM_bool_comm_status = 5;
    d_sim.user.flag_Nyquist_one_cycle_DONE = FALSE;
    d_sim.user.Nyquist_plot_num_cycles = 1.0;
    d_sim.user.Nyquist_one_cycle_count = 0.0;
    d_sim.user.Nyquist_Input_Current_Amp = 0.35;
    d_sim.user.Nyquist_Input_Speed_Amp = 100.0;
    d_sim.user.Nyquist_sum_sin = 0.0;
    d_sim.user.Nyquist_sum_cos = 0.0;
    d_sim.user.Nyquist_Amp = 0.0;
    d_sim.user.Nyquist_Phase = 0.0;
    d_sim.user.Nyquist_Re = 0.0;
    d_sim.user.Nyquist_Im = 0.0;
    d_sim.user.Nyquist_Freq_Ceiling = 200;

}
