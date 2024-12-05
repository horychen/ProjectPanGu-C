// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 224 

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
    long INVERTER_NONLINEARITY_COMPENSATION_METHOD;
    long Select_exp_operation;
    long set_rpm_speed_command;
    long set_iq_command;
    long bezier_NUMBER_OF_STEPS;
    long bezier_order;
    long bezier_order_current;
    REAL bezier_max_p_value;
    REAL bezier_rpm_maximum_effective_speed_error;
    REAL bezier_seconds_step_command;
    REAL bezier_seconds_load_disturbance;
    BOOL bezier_Give_Sweeping_Ref_in_Interrupt;
    BOOL BOOL_BEZIER_ADAPTIVE_GAIN;
    BOOL BOOL_USING_NEW_SOLVER_FOR_BEZIER;
    BOOL BOOL_BEZIER_RUN_IN_MAIN;
    REAL bezier_equivalent_Kp;
    long bezier_Speed_RPM_deadzone;
    BOOL bool_apply_sweeping_frequency_excitation;
    BOOL bool_sweeping_frequency_for_speed_loop;
    BOOL bool_sweeping_frequency_for_current_loop_iD;
    BOOL bool_speed_sweeping_with_Load;
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
    REAL Stable_Time_for_Sweeping;
    BOOL flag_clear_timebase_once;
    BOOL bool_ESO_SPEED_ON;
    BOOL bool_apply_ESO_SPEED_for_SPEED_FBK;
    long CAREFUL_ESOAF_OMEGA_OBSERVER;
    long VL_FEEDBACK_KFB;
} ST_user;


typedef struct {
    ST_init init;
    ST_sim sim;
    ST_FOC FOC;
    ST_CL CL;
    ST_VL VL;
    ST_user user;
} ST_D_SIM;



#define ARGS_PATH "../acmsimc_bezier_points/SD80AEA07530-SC3-COMM-Eureka-4-0.txt"



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OBSV.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.esoaf.xPos,ACM.KA,BezierVL_AdaptVersion.nonlinear_fake_disturbance_estimate,BezierVL_AdaptVersion.points[0].y,BezierVL_AdaptVersion.error,BezierVL_AdaptVersion.output,CTRL->s->Speed->Err*MECH_RAD_PER_SEC_2_RPM,CTRL->s->Speed->Out,ACM.TLoad / 1.5 / ACM.KA / ACM.npp,d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point,d_sim.user.Mark_Counter,d_sim.user.bezier_equivalent_Kp,d_sim.user.bezier_equivalent_Kp*MECH_RAD_PER_SEC_2_RPM\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OBSV.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.esoaf.xPos,ACM.KA,BezierVL_AdaptVersion.nonlinear_fake_disturbance_estimate,BezierVL_AdaptVersion.points[0].y,BezierVL_AdaptVersion.error,BezierVL_AdaptVersion.output,CTRL->s->Speed->Err*MECH_RAD_PER_SEC_2_RPM,CTRL->s->Speed->Out,ACM.TLoad / 1.5 / ACM.KA / ACM.npp,d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point,d_sim.user.Mark_Counter,d_sim.user.bezier_equivalent_Kp,d_sim.user.bezier_equivalent_Kp*MECH_RAD_PER_SEC_2_RPM


#define DATA_FILE_NAME "../dat/SD80AEA07530-SC3-COMM-Eureka.dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
