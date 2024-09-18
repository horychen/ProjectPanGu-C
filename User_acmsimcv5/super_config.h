// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 2023231060 

typedef struct {
    long npp;
    REAL IN;
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
    REAL CL_TS;
    long NUMBER_OF_STEPS;
    long MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
} ST_sim;

typedef struct {
    REAL delta;
    REAL CLBW_HZ;
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
    long bezier_NUMBER_OF_STEPS;
    long bezier_order;
    long bezier_order_current;
    REAL bezier_max_p_value;
    REAL bezier_rpm_maximum_effective_speed_error;
    REAL bezier_A_current_limit;
    REAL bezier_seconds_step_command;
    REAL bezier_seconds_load_disturbance;
    long INVERTER_NONLINEARITY_COMPENSATION_METHOD;
    long Select_exp_operation;
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



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,FE.AFEOE.psi_1[0],FE.AFEOE.psi_1[1],FE.AFEOE.u_offset[0],FE.AFEOE.u_offset[1],OBSV.nsoaf.xOmg,OBSV.nsoaf.xIq,OBSV.nsoaf.output_error,FE.AFEOE.theta_d/M_PI*180,OBSV.nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.nsoaf.xTL,OBSV.nsoaf.active_power_error,(*CTRL).motor->R,COMM.R,COMM.KE,COMM.Js,COMM.L,FE.no_sat.psi_2[0],FE.no_sat.psi_2[1],FE.no_sat.psi_1[0],FE.no_sat.psi_1[1],FE.no_sat.u_offset[0],FE.no_sat.u_offset[1],FE.no_sat.ell_1,FE.no_sat.psi_com[0],FE.no_sat.psi_com[1],FE.clfe4PMSM.psi_2[0],FE.clfe4PMSM.psi_2[1],FE.AFEOE.psi_2[0],FE.AFEOE.psi_2[1],FE.htz.psi_2[0],FE.htz.psi_2[1],FE.clfe4PMSM.u_offset[0],FE.clfe4PMSM.u_offset[1],FE.htz.psi_1[0],FE.htz.psi_1[1],FE.htz.psi_1_nonSat[0],FE.htz.psi_1_nonSat[1],FE.clfe4PMSM.psi_1[0],FE.clfe4PMSM.psi_1[1],FE.clfe4PMSM.current_error[0],FE.clfe4PMSM.current_error[1],FE.clfe4PMSM.hat_psi_d2,ACM.iAB[0],FE.clfe4PMSM.current_estimate[0],FE.lascu.psi_2[1],FE.lascu.psi_2[0],ACM.theta_d,FE.no_sat.theta_d,FE.no_sat.theta_e,FE.clfe4PMSM.theta_e,FE.htz.theta_e,FE.no_sat.psi_1_max[0],FE.no_sat.psi_1_max[1],FE.no_sat.psi_1_min[0],FE.no_sat.psi_1_min[1]\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,FE.AFEOE.psi_1[0],FE.AFEOE.psi_1[1],FE.AFEOE.u_offset[0],FE.AFEOE.u_offset[1],OBSV.nsoaf.xOmg,OBSV.nsoaf.xIq,OBSV.nsoaf.output_error,FE.AFEOE.theta_d/M_PI*180,OBSV.nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.nsoaf.xTL,OBSV.nsoaf.active_power_error,(*CTRL).motor->R,COMM.R,COMM.KE,COMM.Js,COMM.L,FE.no_sat.psi_2[0],FE.no_sat.psi_2[1],FE.no_sat.psi_1[0],FE.no_sat.psi_1[1],FE.no_sat.u_offset[0],FE.no_sat.u_offset[1],FE.no_sat.ell_1,FE.no_sat.psi_com[0],FE.no_sat.psi_com[1],FE.clfe4PMSM.psi_2[0],FE.clfe4PMSM.psi_2[1],FE.AFEOE.psi_2[0],FE.AFEOE.psi_2[1],FE.htz.psi_2[0],FE.htz.psi_2[1],FE.clfe4PMSM.u_offset[0],FE.clfe4PMSM.u_offset[1],FE.htz.psi_1[0],FE.htz.psi_1[1],FE.htz.psi_1_nonSat[0],FE.htz.psi_1_nonSat[1],FE.clfe4PMSM.psi_1[0],FE.clfe4PMSM.psi_1[1],FE.clfe4PMSM.current_error[0],FE.clfe4PMSM.current_error[1],FE.clfe4PMSM.hat_psi_d2,ACM.iAB[0],FE.clfe4PMSM.current_estimate[0],FE.lascu.psi_2[1],FE.lascu.psi_2[0],ACM.theta_d,FE.no_sat.theta_d,FE.no_sat.theta_e,FE.clfe4PMSM.theta_e,FE.htz.theta_e,FE.no_sat.psi_1_max[0],FE.no_sat.psi_1_max[1],FE.no_sat.psi_1_min[0],FE.no_sat.psi_1_min[1]


#define DATA_FILE_NAME "../dat/MD1-08075AC30BB0L1.dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
