// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 4;
    d_sim.init.IN = 6;
    d_sim.init.R = 1.967709;
    d_sim.init.Ld = 0.002776;
    d_sim.init.Lq = 0.002776;
    d_sim.init.KE = 0.086;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.0004624383;
    d_sim.init.Vdc = 60;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 65000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 5;
    d_sim.FOC.CLBW_HZ = 800;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = FALSE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 13.953697930184426;
    d_sim.CL.SERIES_KI_D_AXIS = 708.8288904899135;
    d_sim.CL.SERIES_KP_Q_AXIS = 13.953697930184426;
    d_sim.CL.SERIES_KI_Q_AXIS = 708.8288904899135;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.95;

    d_sim.VL.SERIES_KP = 0.9009567541200327;
    d_sim.VL.SERIES_KI = 201.06192982974676;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 0.95;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 102209;
    d_sim.user.mode_select_synchronous_motor = 3;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 500;
    d_sim.user.set_iq_command = 1;
    d_sim.user.bool_apply_sweeping_frequency_excitation = TRUE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = TRUE;
    d_sim.user.bool_sweeping_frequency_for_current_loop_iD = FALSE;
    d_sim.user.CMD_CURRENT_SINE_AMPERE = 1;
    d_sim.user.CMD_SPEED_SINE_RPM = 100;
    d_sim.user.CMD_SPEED_SINE_HZ = 0;
    d_sim.user.CMD_SPEED_SINE_STEP_SIZE = 1;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME = 1e-4;
    d_sim.user.CMD_SPEED_SINE_HZ_CEILING = 200;
    d_sim.user.timebase_for_Sweeping = 0.0;
    d_sim.user.VL_FEEDBACK_KFB = 0;

}
