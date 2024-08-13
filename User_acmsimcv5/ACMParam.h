// This header file is automatically generated. Any modification to this file will get lost.
#ifndef ACMParam_H
#define ACMParam_H
#include "typedef.h"


typedef struct {
    REAL LIMIT_DC_BUS_UTILIZATION;
    REAL SERIES_KI_D_AXIS;
    REAL SERIES_KI_Q_AXIS;
    REAL SERIES_KP_D_AXIS;
    REAL SERIES_KP_Q_AXIS;
} ST_CL;

typedef struct {
    REAL CMD_CURRENT_SINE_AMPERE;
    long CMD_SPEED_SINE_RPM;
    BOOL bool_apply_decoupling_voltages_to_current_regulation;
    BOOL bool_apply_speed_closed_loop_control;
    BOOL bool_apply_sweeping_frequency_excitation;
    BOOL bool_overwrite_speed_commands;
    BOOL bool_zero_id_control;
} ST_CTRL;

typedef struct {
    long CLBW_HZ;
    long CL_KI_factor_when__bool_apply_decoupling_voltages_to_current_regulation__is_False;
    long VL_EXE_PER_CL_EXE;
    long delta;
} ST_FOC;

typedef struct {
    long LIMIT_OVERLOAD_FACTOR;
    REAL SERIES_KI;
    REAL SERIES_KP;
} ST_VL;

typedef struct {
    REAL IN;
    REAL Js;
    REAL KE;
    REAL Ld;
    REAL Lq;
    REAL R;
    REAL Rreq;
    REAL Vdc;
    long npp;
} ST_init;

typedef struct {
    REAL CL_TS;
    long MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
    long NUMBER_OF_STEPS;
} ST_sim;

typedef struct {
    REAL VL_FEEDBACK_KFB;
    long bezier_NUMBER_OF_STEPS;
    REAL bezier_max_p_value;
    long bezier_order;
    long bezier_order_current;
    long bezier_rpm_maximum_effective_speed_error;
    REAL bezier_seconds_load_disturbance;
    REAL bezier_seconds_step_command;
    BOOL bool_apply_WC_tunner_for_speed_loop;
    BOOL bool_sweeping_frequency_for_speed_loop;
    REAL max_CLBW_PER_min_CLBW;
    long omega_n;
    REAL zeta;
} ST_user;


typedef struct {
    ST_CL CL;
    ST_CTRL CTRL;
    ST_FOC FOC;
    ST_VL VL;
    ST_init init;
    ST_sim sim;
    ST_user user;
} ST_D_SIM;



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,debug.set_rpm_speed_command,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1]\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,debug.set_rpm_speed_command,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1]


#define DATA_FILE_NAME "../dat/MD1-08075AC30BB0L1.dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // ACMParam_H
