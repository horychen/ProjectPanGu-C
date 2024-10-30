// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 6;
    d_sim.init.R = 1.44;
    d_sim.init.Ld = 0.0032;
    d_sim.init.Lq = 0.0032;
    d_sim.init.KE = 0.11459;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.000182;
    d_sim.init.Vdc = 60;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 1200;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 10;
    d_sim.FOC.CLBW_HZ = 800;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = TRUE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_D_AXIS = 449.99999999999994;
    d_sim.CL.SERIES_KP_Q_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_Q_AXIS = 449.99999999999994;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.95;

    d_sim.VL.SERIES_KP = 0.1330586992357314;
    d_sim.VL.SERIES_KI = 50.26548245743669;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 0.95;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231051;
    d_sim.user.mode_select_synchronous_motor = 4;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 200;
    d_sim.user.zeta = 0.707;
    d_sim.user.omega_n = 600;
    d_sim.user.max_CLBW_PER_min_CLBW = 0.9;
    d_sim.user.VL_FEEDBACK_KFB = 0;
    d_sim.user.bool_Null_D_Control = TRUE;
    d_sim.user.bool_apply_WC_tunner_for_speed_loop = TRUE;
    d_sim.user.bool_enable_Harnefors_back_calculation = TRUE;
    d_sim.user.bool_apply_sweeping_frequency_excitation = TRUE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = FALSE;
    d_sim.user.bool_sweeping_frequency_for_current_loop_iD = TRUE;
    d_sim.user.CMD_CURRENT_SINE_AMPERE = 1;
    d_sim.user.CMD_SPEED_SINE_RPM = 100;
    d_sim.user.CMD_SPEED_SINE_HZ = 0;
    d_sim.user.CMD_SPEED_SINE_STEP_SIZE = 1;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_HZ_CEILING = 300;
    d_sim.user.bool_Parameter_Mismatch_test = FALSE;
    d_sim.user.ParaMis_percent_max = 4;
    d_sim.user.ParaMis_percent_min = 0.4;
    d_sim.user.ParaMis_Mode_Select = 5;
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

}
