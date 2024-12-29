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
    d_sim.init.Vdc = 48;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 20000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 15;
    d_sim.FOC.CLBW_HZ = 800;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = TRUE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_D_AXIS = 514.65625;
    d_sim.CL.SERIES_KP_Q_AXIS = 16.084954386379742;
    d_sim.CL.SERIES_KI_Q_AXIS = 514.65625;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.96;

    d_sim.VL.SERIES_KP = 0.22058704353118255;
    d_sim.VL.SERIES_KI = 22.340214425527417;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 1.0;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231060;
    d_sim.user.mode_select_synchronous_motor = 49;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 5;
    d_sim.user.set_iq_command = -0.5;
    d_sim.user.Set_SpeedLoop_KI_as_Zero = FALSE;
    d_sim.user.CAREFUL_ESOAF_OMEGA_OBSERVER = 3000;
    d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK = TRUE;
    d_sim.user.bool_ESO_SPEED_ON = TRUE;
    d_sim.user.VL_FEEDBACK_KFB = 0;
    d_sim.user.set_id_command = 0.0;
    d_sim.user.Variable_Parameters_percent = 3;
    d_sim.user.Variable_Parameters_time = 3;
    d_sim.user.Variable_Parameters_status = 4;
    d_sim.user.Variable_Parameters_time_num = 7;
    d_sim.user.VP_time_num_count = 0;
    d_sim.user.Variable_Parameters_timebase = 0.0;
    d_sim.user.sensorless_only_theta_on = 0;
    d_sim.user.sensorless_speed_observer = 0;
    d_sim.user.SENSORLESS_CONTROL = 0;

}
