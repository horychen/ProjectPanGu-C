// This c file is automatically generated. Any modification to this file will get lost.
#include "ACMParam.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.96;
    d_sim.CL.SERIES_KI_D_AXIS = 231.578947368421;
    d_sim.CL.SERIES_KI_Q_AXIS = 231.578947368421;
    d_sim.CL.SERIES_KP_D_AXIS = 0.5969026041820606;
    d_sim.CL.SERIES_KP_Q_AXIS = 0.5969026041820606;

    d_sim.CTRL.CMD_CURRENT_SINE_AMPERE = 1.0;
    d_sim.CTRL.CMD_SPEED_SINE_RPM = 50;
    d_sim.CTRL.bool_apply_decoupling_voltages_to_current_regulation = FALSE;
    d_sim.CTRL.bool_apply_speed_closed_loop_control = TRUE;
    d_sim.CTRL.bool_apply_sweeping_frequency_excitation = FALSE;
    d_sim.CTRL.bool_overwrite_speed_commands = TRUE;
    d_sim.CTRL.bool_zero_id_control = TRUE;

    d_sim.FOC.CLBW_HZ = 500;
    d_sim.FOC.CL_KI_factor_when__bool_apply_decoupling_voltages_to_current_regulation__is_False = 10;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 5;
    d_sim.FOC.delta = 5;

    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 10;
    d_sim.VL.SERIES_KI = 125.66370614359171;
    d_sim.VL.SERIES_KP = 3.6852374714364777;

    d_sim.init.IN = 16.8;
    d_sim.init.Js = 0.000755295972;
    d_sim.init.KE = 0.01717;
    d_sim.init.Ld = 0.00019;
    d_sim.init.Lq = 0.00019;
    d_sim.init.R = 0.044;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Vdc = 48.0;
    d_sim.init.npp = 5;

    d_sim.sim.CL_TS = 0.0001;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;
    d_sim.sim.NUMBER_OF_STEPS = 5000;

    d_sim.user.VL_FEEDBACK_KFB = 0.0;
    d_sim.user.bezier_NUMBER_OF_STEPS = 5000;
    d_sim.user.bezier_max_p_value = 0.0;
    d_sim.user.bezier_order = 4;
    d_sim.user.bezier_order_current = 0;
    d_sim.user.bezier_rpm_maximum_effective_speed_error = 500;
    d_sim.user.bezier_seconds_load_disturbance = 0.4;
    d_sim.user.bezier_seconds_step_command = 0.2;
    d_sim.user.bool_apply_WC_tunner_for_speed_loop = TRUE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = TRUE;
    d_sim.user.max_CLBW_PER_min_CLBW = 0.5;
    d_sim.user.omega_n = 1000;
    d_sim.user.zeta = 0.5;

}