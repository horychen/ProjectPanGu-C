// This c file is automatically generated. Any modification to this file will get lost.
#include "super_config.h"
#include <stdio.h>

void init_d_sim() {
    d_sim.init.npp = 18;
    d_sim.init.IN = 3.0;
    d_sim.init.R = 0.077;
    d_sim.init.Ld = 0.000135;
    d_sim.init.Lq = 0.00019;
    d_sim.init.KE = 0.0056;
    d_sim.init.Rreq = 0.0;
    d_sim.init.Js = 7.900000000000001e-05;
    d_sim.init.Vdc = 30;

    d_sim.sim.INVERTER_NONLINEARITY = 0;
    d_sim.sim.CLTS = 0.0001;
    d_sim.sim.NUMBER_OF_STEPS = 5000;
    d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = 1;

    d_sim.FOC.delta = 8;
    d_sim.FOC.CLBW_HZ = 400;
    d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation = TRUE;
    d_sim.FOC.VL_EXE_PER_CL_EXE = 5;

    d_sim.CL.SERIES_KP_D_AXIS = 0.3392920065876977;
    d_sim.CL.SERIES_KI_D_AXIS = 570.3703703703703;
    d_sim.CL.SERIES_KP_Q_AXIS = 0.4775220833456486;
    d_sim.CL.SERIES_KI_Q_AXIS = 405.2631578947368;
    d_sim.CL.LIMIT_DC_BUS_UTILIZATION = 0.96;

    d_sim.VL.SERIES_KP = 0.16414406060422865;
    d_sim.VL.SERIES_KI = 39.269908169872416;
    d_sim.VL.LIMIT_OVERLOAD_FACTOR = 1.0;

    d_sim.user.verbose = TRUE;
    d_sim.user.who_is_user = 102209;
    d_sim.user.mode_select_synchronous_motor = 2;
    d_sim.user.mode_select_induction_motor = 32;
    d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD = 0;
    d_sim.user.Select_exp_operation = 0;
    d_sim.user.set_rpm_speed_command = 400;
    d_sim.user.set_iq_command = 1.0;
    d_sim.user.bool_apply_sweeping_frequency_excitation = FALSE;
    d_sim.user.bool_sweeping_frequency_for_speed_loop = FALSE;
    d_sim.user.bool_speed_sweeping_with_Load = FALSE;
    d_sim.user.bool_sweeping_frequency_for_current_loop_iD = FALSE;
    d_sim.user.bool_sweeping_frequency_for_Rejection_Load = FALSE;
    d_sim.user.CMD_CURRENT_SINE_AMPERE = 1.0;
    d_sim.user.CMD_SPEED_SINE_RPM = 100;
    d_sim.user.CMD_SPEED_SINE_HZ = 99;
    d_sim.user.CMD_SPEED_SINE_STEP_SIZE = 1;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME = 1e-4;
    d_sim.user.CMD_SPEED_SINE_HZ_CEILING = 150;
    d_sim.user.timebase_for_Sweeping = 0.0;
    d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point = 0.0;
    d_sim.user.Mark_Counter = 0.0;
    d_sim.user.Set_SpeedLoop_KI_as_Zero = FALSE;
    d_sim.user.Stable_Time_for_Sweeping = 1.5;
    d_sim.user.flag_clear_timebase_once = FALSE;
    d_sim.user.bool_ESO_SPEED_ON = TRUE;
    d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK = TRUE;
    d_sim.user.CAREFUL_ESOAF_OMEGA_OBSERVER = 4500;
    d_sim.user.VL_FEEDBACK_KFB = 0;

}
