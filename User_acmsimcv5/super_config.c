// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 3;
    d_sim.init.R = 1.6469;
    d_sim.init.Ld = 0.0032;
    d_sim.init.Lq = 0.0032;
    d_sim.init.KE = 0.12618;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.00049836;
    d_sim.init.Vdc = 110;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CL_TS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 50000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 10;
    d_sim.FOC.CLBW_HZ = 800.0;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = TRUE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_D_AXIS = 514.65625;
    d_sim.CL.SERIES_KP_Q_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_Q_AXIS = 514.65625;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.96;

    d_sim.VL.SERIES_KP = 0.3308805652967738;
    d_sim.VL.SERIES_KI = 50.26548245743669;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 1.0;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231060;
    d_sim.user.mode_select_synchronous_motor = 41;
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
    d_sim.user.VL_FEEDBACK_KFB = 0;

}
