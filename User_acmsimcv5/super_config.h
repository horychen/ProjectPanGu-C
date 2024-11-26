// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 2023231051 

typedef struct {
    long npp;
    long IN;
    REAL R;
    REAL Ld;
    REAL Lq;
    REAL KE;
    REAL Rreq;
    REAL Js;
    long Vdc;
} ST_init;

typedef struct {
    long INVERTER_NONLINEARITY;
    REAL CLTS;
    long NUMBER_OF_STEPS;
    long MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
} ST_sim;

typedef struct {
    long delta;
    long CLBW_HZ;
    BOOL bool_apply_decoupling_voltages_to_current_regulation;
    long VL_EXE_PER_CL_EXE;
} ST_FOC;

typedef struct {
    REAL SERIES_KP_D_AXIS;
    REAL SERIES_KI_D_AXIS;
    REAL SERIES_KP_Q_AXIS;
    REAL SERIES_KI_Q_AXIS;
    REAL LIMIT_DC_BUS_UTILIZATION;
} ST_CL;

typedef struct {
    REAL SERIES_KP;
    REAL SERIES_KI;
    REAL LIMIT_OVERLOAD_FACTOR;
} ST_VL;

typedef struct {
    BOOL verbose;
    long who_is_user;
    long mode_select_synchronous_motor;
    long mode_select_induction_motor;
    BOOL INVERTER_NONLINEARITY_COMPENSATION_METHOD;
    REAL Select_exp_operation;
    long set_rpm_speed_command;
    REAL zeta;
    long omega_n;
    REAL max_CLBW_PER_min_CLBW;
    long VL_FEEDBACK_KFB;
    BOOL bool_Null_D_Control;
    BOOL bool_apply_WC_tunner_for_speed_loop;
    BOOL bool_enable_Harnefors_back_calculation;
    REAL Check_Harnerfors_1998_On;
    BOOL bool_apply_sweeping_frequency_excitation;
    BOOL bool_sweeping_frequency_for_speed_loop;
    BOOL bool_sweeping_frequency_for_current_loop_iD;
    long CMD_CURRENT_SINE_AMPERE;
    long CMD_SPEED_SINE_RPM;
    long CMD_SPEED_SINE_HZ;
    long CMD_SPEED_SINE_STEP_SIZE;
    REAL CMD_SPEED_SINE_LAST_END_TIME;
    REAL CMD_SPEED_SINE_END_TIME;
    long CMD_SPEED_SINE_HZ_CEILING;
    REAL timebase_for_Sweeping;
    REAL Mark_Sweeping_Freq_ThreeDB_Point;
    REAL Mark_Counter;
    BOOL Set_SpeedLoop_KI_as_Zero;
    BOOL bool_ESO_SPEED_ON;
    BOOL bool_apply_ESO_SPEED_for_SPEED_FBK;
    BOOL bool_Parameter_Mismatch_test;
    long ParaMis_percent_max;
    REAL ParaMis_percent_min;
    long ParaMis_Mode_Select;
    REAL ParaMis_OneCycleTime;
    BOOL bool_apply_HitWall_analysis;
    REAL HitWall_max_limit_ratio;
    REAL HitWall_min_limit_ratio;
    long HitWall_high_RPM_command;
    REAL HitWall_time_interval;
    long Position_Loop_Kp;
    long set_deg_position_command;
    long Position_Output_Limit;
    long Position_cmd_sine_frequency;
    long Position_Loop_Ref_prev;
    long Position_Loop_Ref_Diff;
    BOOL bool_use_position_feedforward_by_PosDiff;
    BOOL bool_compensation_of_Pos_CTRL_output_covergence;
    long COMM_bool_comm_status;
    BOOL flag_Nyquist_one_cycle_DONE;
    REAL Nyquist_plot_num_cycles;
    REAL Nyquist_one_cycle_count;
    REAL Nyquist_Input_Current_Amp;
    REAL Nyquist_Input_Speed_Amp;
    REAL Nyquist_sum_sin;
    REAL Nyquist_sum_cos;
    REAL Nyquist_Amp;
    REAL Nyquist_Phase;
    REAL Nyquist_Re;
    REAL Nyquist_Im;
    long Nyquist_Freq_Ceiling;
} ST_user;


typedef struct {
    ST_init init;
    ST_sim sim;
    ST_FOC FOC;
    ST_CL CL;
    ST_VL VL;
    ST_user user;
} ST_D_SIM;



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OBSV.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.esoaf.xPos,(*CTRL).timebase,ACM.KA,ACM.iDQ[0],ACM.iDQ[1],wubo_ParaMis.percent_Ld,wubo_ParaMis.percent_Lq,wubo_ParaMis.percent_Rs,wubo_ParaMis.percent_Js,wubo_ParaMis.percent_KE,SIL_Controller.KFB,(*CTRL).o->cmd_uAB[0],(*CTRL).o->cmd_uAB[1],(*CTRL).o->cmd_uAB_to_inverter[0],(*CTRL).o->cmd_uAB_to_inverter[1],(*CTRL).s->iD->OutLimit,(*CTRL).s->iQ->OutLimit,(*CTRL).s->Speed->Kp,(*CTRL).s->Speed->Ki_CODE,(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).s->iD->Kp,(*CTRL).s->iD->Ki_CODE,(*CTRL).s->iQ->Kp,(*CTRL).s->iQ->Ki_CODE,ACM.theta_d/M_PI*180,(*CTRL).i->cmd_varTheta,(*CTRL).i->theta_d_elec * (*CTRL).motor->npp_inv,(*CTRL).i->varTheta,d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OBSV.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.esoaf.xPos,(*CTRL).timebase,ACM.KA,ACM.iDQ[0],ACM.iDQ[1],wubo_ParaMis.percent_Ld,wubo_ParaMis.percent_Lq,wubo_ParaMis.percent_Rs,wubo_ParaMis.percent_Js,wubo_ParaMis.percent_KE,SIL_Controller.KFB,(*CTRL).o->cmd_uAB[0],(*CTRL).o->cmd_uAB[1],(*CTRL).o->cmd_uAB_to_inverter[0],(*CTRL).o->cmd_uAB_to_inverter[1],(*CTRL).s->iD->OutLimit,(*CTRL).s->iQ->OutLimit,(*CTRL).s->Speed->Kp,(*CTRL).s->Speed->Ki_CODE,(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).s->iD->Kp,(*CTRL).s->iD->Ki_CODE,(*CTRL).s->iQ->Kp,(*CTRL).s->iQ->Ki_CODE,ACM.theta_d/M_PI*180,(*CTRL).i->cmd_varTheta,(*CTRL).i->theta_d_elec * (*CTRL).motor->npp_inv,(*CTRL).i->varTheta,d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point


#define DATA_FILE_NAME "../dat/SD80AEA07530-SC3-COMM-Eureka.dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
