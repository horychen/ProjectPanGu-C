// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 26;
    d_sim.init.IN = 10.4;
    d_sim.init.R = 0.1195;
    d_sim.init.Ld = 0.000132;
    d_sim.init.Lq = 0.000132;
    d_sim.init.KE = 0.0193;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 0.000364;
    d_sim.init.Vdc = 48;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 10000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 15;
    d_sim.FOC.CLBW_HZ = 800;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = TRUE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 1;

    d_sim.CL.SERIES_KP_D_AXIS = 0.6635043684381644;
    d_sim.CL.SERIES_KI_D_AXIS = 905.3030303030301;
    d_sim.CL.SERIES_KP_Q_AXIS = 0.6635043684381644;
    d_sim.CL.SERIES_KI_Q_AXIS = 905.3030303030301;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.96;

    d_sim.VL.SERIES_KP = 0.1620533688898362;
    d_sim.VL.SERIES_KI = 22.340214425527417;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 1.0;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 2023231060;
    d_sim.user.mode_select_synchronous_motor = 41;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 100;
    d_sim.user.set_iq_command = 8;
    d_sim.user.Set_SpeedLoop_KI_as_Zero = FALSE;
    d_sim.user.CAREFUL_ESOAF_OMEGA_OBSERVER = 3000;
    d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK = FALSE;
    d_sim.user.bool_ESO_SPEED_ON = TRUE;
    d_sim.user.Variable_Parameters_percent = 20;
    d_sim.user.Variable_Parameters_time = 3;
    d_sim.user.Variable_Parameters_status = 1;
    d_sim.user.Variable_Parameters_time_num = 7;
    d_sim.user.VP_time_num_count = 0;
    d_sim.user.Variable_Parameters_timebase = 0.0;
    d_sim.user.VL_FEEDBACK_KFB = 0;

}
