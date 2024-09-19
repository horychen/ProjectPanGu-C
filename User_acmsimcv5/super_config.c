// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 5;
    d_sim.init.IN = 16.8;
    d_sim.init.R = 0.196;
    d_sim.init.Ld = 0.00019;
    d_sim.init.Lq = 0.00019;
    d_sim.init.KE = 0.01717;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.000755295972;
    d_sim.init.Vdc = 72;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CL_TS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 15000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 20;
    d_sim.FOC.CLBW_HZ = 800.0;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = FALSE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 2;

    d_sim.CL.SERIES_KP_D_AXIS = 0.9550441666912972;
    d_sim.CL.SERIES_KI_D_AXIS = 1031.578947368421;
    d_sim.CL.SERIES_KP_Q_AXIS = 0.9550441666912972;
    d_sim.CL.SERIES_KI_Q_AXIS = 1031.578947368421;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.95;

    d_sim.VL.SERIES_KP = 1.4740949885745913;
    d_sim.VL.SERIES_KI = 12.566370614359172;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 0.3;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231051;
    d_sim.user.mode_select_synchronous_motor = 99;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.bezier_NUMBER_OF_STEPS = 5000;
    d_sim.user.bezier_order = 4;
    d_sim.user.bezier_order_current = 0;
    d_sim.user.bezier_max_p_value = 0.0;
    d_sim.user.bezier_rpm_maximum_effective_speed_error = 500.0;
    d_sim.user.bezier_A_current_limit = 16.9;
    d_sim.user.bezier_seconds_step_command = 0.2;
    d_sim.user.bezier_seconds_load_disturbance = 0.4;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.zeta = 1;
    d_sim.user.omega_n = 400;
    d_sim.user.max_CLBW_PER_min_CLBW = 0.9;
    d_sim.user.VL_FEEDBACK_KFB = 0;
    d_sim.user.bool_apply_WC_tunner_for_speed_loop = TRUE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = FALSE;
    d_sim.user.bool_Null_D_Control = TRUE;
    d_sim.user.bool_apply_sweeping_frequency_excitation = TRUE;
    d_sim.user.CMD_CURRENT_SINE_AMPERE = 1;
    d_sim.user.CMD_SPEED_SINE_RPM = 100;
    d_sim.user.CMD_SPEED_SINE_HZ = 0;
    d_sim.user.CMD_SPEED_SINE_STEP_SIZE = 1;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_HZ_CEILING = 300;

}
