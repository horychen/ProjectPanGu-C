// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 3;
    d_sim.init.R = 1.44;
    d_sim.init.Ld = 0.0032;
    d_sim.init.Lq = 0.0032;
    d_sim.init.KE = 0.11459;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.000182;
    d_sim.init.Vdc = 60;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 5500;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 2;
    d_sim.FOC.CLBW_HZ = 800;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = FALSE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_D_AXIS = 449.99999999999994;
    d_sim.CL.SERIES_KP_Q_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_Q_AXIS = 449.99999999999994;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.95;

    d_sim.VL.SERIES_KP = 0.6652934961786571;
    d_sim.VL.SERIES_KI = 1256.6370614359173;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 0.95;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 102209;
    d_sim.user.mode_select_synchronous_motor = 4;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.VL_FEEDBACK_KFB = 0;

}
