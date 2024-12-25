// This header file is automatically generated. Any modification to this file will get lost.
#ifndef SUPER_CONFIG_H
#define SUPER_CONFIG_H
#include "typedef.h"

#define WHO_IS_USER 2023231060 

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
    BOOL Set_SpeedLoop_KI_as_Zero;
    long CAREFUL_ESOAF_OMEGA_OBSERVER;
    BOOL bool_apply_ESO_SPEED_for_SPEED_FBK;
    BOOL bool_ESO_SPEED_ON;
    long VL_FEEDBACK_KFB;
    long Variable_Parameters_percent;
    long Variable_Parameters_time;
    long Variable_Parameters_status;
    long Variable_Parameters_time_num;
    long VP_time_num_count;
    REAL Variable_Parameters_timebase;
    long sensorless_only_theta_on;
    long sensorless_speed_observer;
} ST_user;


typedef struct {
    ST_init init;
    ST_sim sim;
    ST_FOC FOC;
    ST_CL CL;
    ST_VL VL;
    ST_user user;
} ST_D_SIM;



#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OFSR.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OFSR.esoaf.xPos,ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,ACM.KE,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,OBSV.nsoaf.xOmg,OBSV.nsoaf.xIq,OBSV.nsoaf.output_error,OBSV.nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.nsoaf.xTL,OBSV.nsoaf.active_power_error,(*CTRL).motor->R,COMM.R,COMM.KE,COMM.Js,COMM.L,FE.clfe4PMSM.psi_2[0],FE.clfe4PMSM.psi_2[1],FE.htz.psi_2[0],FE.htz.psi_2[1],FE.clfe4PMSM.u_offset[0],FE.clfe4PMSM.u_offset[1],FE.clfe4PMSM.theta_e,FE.htz.psi_1[0],FE.htz.psi_1[1],FE.htz.psi_1_nonSat[0],FE.htz.psi_1_nonSat[1],FE.htz.u_offset[0],FE.htz.u_offset[1],FE.clfe4PMSM.psi_1[0],FE.clfe4PMSM.psi_1[1],FE.clfe4PMSM.hat_psi_d2,FE.clfe4PMSM.theta_d,ACM.iAB[0],FE.clfe4PMSM.current_estimate[0],FE.lascu.psi_2[1],FE.lascu.psi_2[0],FE.lascu.theta_d,FE.lascu.theta_e,ACM.theta_d,FE.htz.theta_e,FE.htz.theta_d,(*CTRL).motor->Lq,(*CTRL).i->TLoad,(*CTRL).s->xRho/M_PI*180,(*CTRL).motor->KE,(*CTRL).motor->Js,ACM.Js,CTRL->s->iD->Out,CTRL->s->iQ->Out,CTRL->s->iD->Err,CTRL->s->iQ->Err,CTRL->s->iD->I_Term,CTRL->s->iQ->I_Term,ACM.Lq,ACM.R,FE.htz.emf_stator[0],FE.htz.emf_stator[1],FE.Bernard.theta_d,FE.Bernard.psi_2[0],FE.Bernard.psi_2[1],FE.Bernard.psi_PM,FE.Bernard.error_correction,FE.Bernard.psi_2_ampl,FE.Bernard.theta_e,d_sim.user.VP_time_num_count,(*CTRL).enc->OffsetCountBetweenIndexAndUPhaseAxis\n"
#define DATA_DETAILS ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).o->dc_bus_utilization_ratio,(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,(*CTRL).timebase,OFSR.esoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OFSR.esoaf.xPos,ACM.x[0],ACM.x[1],ACM.x[2],ACM.x[3],ACM.x[4],ACM.theta_d/M_PI*180,ACM.varOmega * MECH_RAD_PER_SEC_2_RPM,ACM.uAB[0],ACM.uAB[1],ACM.uDQ[0],ACM.uDQ[1],ACM.iAB[0],ACM.iAB[1],ACM.iDQ[0],ACM.iDQ[1],ACM.TLoad,ACM.Tem,ACM.KA,ACM.KE,(*debug).set_rpm_speed_command,(*CTRL).s->xRho/M_PI*180,(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM,(*CTRL).i->cmd_varOmega,(*CTRL).i->varOmega,(*CTRL).i->cmd_iDQ[0],(*CTRL).i->cmd_iDQ[1],(*CTRL).o->cmd_uDQ[0],(*CTRL).o->cmd_uDQ[1],(*CTRL).i->iDQ[0],(*CTRL).i->iDQ[1],(*CTRL).i->iAB[0],(*CTRL).i->iAB[1],(*CTRL).i->cmd_psi,(*CTRL).i->Tem,(*CTRL).i->cmd_Tem,(*debug).Overwrite_theta_d / ACM.npp,OBSV.nsoaf.xOmg,OBSV.nsoaf.xIq,OBSV.nsoaf.output_error,OBSV.nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM,OBSV.nsoaf.xTL,OBSV.nsoaf.active_power_error,(*CTRL).motor->R,COMM.R,COMM.KE,COMM.Js,COMM.L,FE.clfe4PMSM.psi_2[0],FE.clfe4PMSM.psi_2[1],FE.htz.psi_2[0],FE.htz.psi_2[1],FE.clfe4PMSM.u_offset[0],FE.clfe4PMSM.u_offset[1],FE.clfe4PMSM.theta_e,FE.htz.psi_1[0],FE.htz.psi_1[1],FE.htz.psi_1_nonSat[0],FE.htz.psi_1_nonSat[1],FE.htz.u_offset[0],FE.htz.u_offset[1],FE.clfe4PMSM.psi_1[0],FE.clfe4PMSM.psi_1[1],FE.clfe4PMSM.hat_psi_d2,FE.clfe4PMSM.theta_d,ACM.iAB[0],FE.clfe4PMSM.current_estimate[0],FE.lascu.psi_2[1],FE.lascu.psi_2[0],FE.lascu.theta_d,FE.lascu.theta_e,ACM.theta_d,FE.htz.theta_e,FE.htz.theta_d,(*CTRL).motor->Lq,(*CTRL).i->TLoad,(*CTRL).s->xRho/M_PI*180,(*CTRL).motor->KE,(*CTRL).motor->Js,ACM.Js,CTRL->s->iD->Out,CTRL->s->iQ->Out,CTRL->s->iD->Err,CTRL->s->iQ->Err,CTRL->s->iD->I_Term,CTRL->s->iQ->I_Term,ACM.Lq,ACM.R,FE.htz.emf_stator[0],FE.htz.emf_stator[1],FE.Bernard.theta_d,FE.Bernard.psi_2[0],FE.Bernard.psi_2[1],FE.Bernard.psi_PM,FE.Bernard.error_correction,FE.Bernard.psi_2_ampl,FE.Bernard.theta_e,d_sim.user.VP_time_num_count,(*CTRL).enc->OffsetCountBetweenIndexAndUPhaseAxis


#define DATA_FILE_NAME "../dat/SD80AEA07530-SC3-COMM.dat"

extern ST_D_SIM d_sim;
void init_d_sim();
#endif // SUPER_CONFIG_H
