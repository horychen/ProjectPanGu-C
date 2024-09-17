/*
 * ShareMemory.c
 *
 *  Created on: 2021锟斤拷1锟斤拷15锟斤拷
 *      Author: JIAHAO
 *///cpu2 CONNECTION

#include <All_Definition.h>

extern REAL vvvf_voltage;
extern REAL vvvf_frequency;
extern Uint32 position_count_CAN_ID0x01_fromCPU2;
extern Uint32 position_count_CAN_ID0x03_fromCPU2;
extern Uint32 position_count_SCI_hip_fromCPU2;
extern Uint32 position_count_SCI_shank_fromCPU2;

st_axis *Axis4DAC;
#if NUMBER_OF_DSP_CORES == 2

struct IPC_MEMORY_WRITE Write;
struct IPC_MEMORY_READ Read;
#pragma DATA_SECTION(Write, "SHARERAMGS1"); // GS1 is write
#pragma DATA_SECTION( Read, "SHARERAMGS0");


REAL angle_error_limiter(REAL angle_error){
    if(fabs(angle_error)>M_PI){
        angle_error -= sign(angle_error) * 2*M_PI;
    }
    return angle_error;
}



extern REAL hall_sensor_read[3];
extern int current_pole[3];
extern REAL hall_qep_count;
extern int count_magnet[3];
extern REAL hall_theta_r_elec[3];
extern REAL hall_theta_r_elec_incremental[3];
extern REAL hall_theta_r_elec_local_absolute[3];
extern REAL normalizer[3];
extern REAL hall_rotating_direction;
extern REAL eddy_displacement[2];
//extern REAL used_theta_d_elec;
extern REAL target_position_cnt;
extern long long sci_pos;
long int counter=0;
int bool_test_dac_sinusoidal = FALSE;

//extern REAL target_position_cnt = 5000;
//extern REAL KP = 0.05;
//extern REAL direction



void write_DAC_buffer(){
    if(use_first_set_three_phase==2){
        Axis4DAC = &Axis_2;
    }else{
        Axis4DAC = &Axis_1;
    }

    if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤，锟角碉拷锟斤拷效锟斤拷围锟斤拷 [-1, 1]锟斤拷
        // wubo:我用的这套设备dac：dac_watch在dsp中输出[-1,1]V,通过dac板子输出[-3,3V]
        (*Axis4DAC).dac_watch[0] = Axis_1.iuvw[0]*0.2;
        (*Axis4DAC).dac_watch[1] = Axis_1.iuvw[1]*0.2;
        (*Axis4DAC).dac_watch[2] = Axis_1.iuvw[2]*0.2;
        (*Axis4DAC).dac_watch[3] = Axis_2.iuvw[3]*0.2;
        (*Axis4DAC).dac_watch[4] = Axis_2.iuvw[4]*0.2;
        (*Axis4DAC).dac_watch[5] = Axis_2.iuvw[5]*0.2;

        (*Axis4DAC).dac_watch[6] = (*CTRL).i->iAB[0]*0.25;      // +-6A ->0-3V
        (*Axis4DAC).dac_watch[7] = (*CTRL).i->iAB[1]*0.25;
        (*Axis4DAC).dac_watch[8] = (*CTRL).o->cmd_uAB[0]*0.125; // +-12V -> 0-3V
        (*Axis4DAC).dac_watch[9] = (*CTRL).o->cmd_uAB[1]*0.125;
#if WHO_IS_USER == USER_YZZ
        (*Axis4DAC).dac_watch[10] = FE.htz.psi_2_ampl*0.25;
        (*Axis4DAC).dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.25;
        (*Axis4DAC).dac_watch[12] = FE.htz.psi_2[0]*0.25;
        (*Axis4DAC).dac_watch[13] = FE.htz.psi_2[1]*0.25;
        (*Axis4DAC).dac_watch[14] = FE.htz.theta_d*0.1;
        (*Axis4DAC).dac_watch[15] = FE.htz.u_offset[0]*2;
        (*Axis4DAC).dac_watch[16] = FE.htz.u_offset[1]*2;
#endif
        (*Axis4DAC).dac_watch[17] = (*CTRL).svgen2.Ta;
        (*Axis4DAC).dac_watch[18] = (*CTRL).svgen2.Tb;
        (*Axis4DAC).dac_watch[19] = (*CTRL).svgen2.Tc;
        (*Axis4DAC).dac_watch[20] = (*CTRL).s->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;

        // (*Axis4DAC).dac_watch[21] = Axis_1.Set_manual_rpm *0.002;
        (*Axis4DAC).dac_watch[21] = Axis_1.pCTRL->i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM * 0.002; // 500RPM - 3V
        (*Axis4DAC).dac_watch[22] = Axis_1.pCTRL->i->varOmega     * MECH_RAD_PER_SEC_2_RPM * 0.002;
        (*Axis4DAC).dac_watch[23] = Axis_2.pCTRL->i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM * 0.002; // 500RPM - 3V
        (*Axis4DAC).dac_watch[24] = Axis_2.pCTRL->i->varOmega     * MECH_RAD_PER_SEC_2_RPM * 0.002;

        (*Axis4DAC).dac_watch[25] = Axis_1.pCTRL->i->cmd_iDQ[0]*0.2; // 0.6V -> 1A
        (*Axis4DAC).dac_watch[26] = Axis_1.pCTRL->i->iDQ[0]*0.2;
        (*Axis4DAC).dac_watch[27] = Axis_1.pCTRL->i->cmd_iDQ[1]*0.2;
        (*Axis4DAC).dac_watch[28] = Axis_1.pCTRL->i->iDQ[1]*0.2;

        (*Axis4DAC).dac_watch[63] = (*CTRL).o->cmd_uAB_to_inverter[0] * 0.125;
        (*Axis4DAC).dac_watch[64] = (*CTRL).o->cmd_uAB_to_inverter[1] * 0.125;


        // (*Axis4DAC).dac_watch[25] = (*CTRL).i->cmd_iDQ[0]*0.2;
        // (*Axis4DAC).dac_watch[26] = (*CTRL).i->iDQ[0]*0.2;
        // (*Axis4DAC).dac_watch[27] = (*CTRL).i->cmd_iDQ[1]*0.2;
        // (*Axis4DAC).dac_watch[28] = (*CTRL).i->iDQ[1]*0.2; // 0.6V->1A

    //    (*Axis4DAC).dac_watch[21] = marino.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
        // (*Axis4DAC).dac_watch[21] = curycontroller.theta1*0.1; // CTRL_2.s->spd->Ref*0.0001;//curycontroller.theta1*0.1; /// 131072.0 *2;
        // (*Axis4DAC).dac_watch[22] = curycontroller.dot_theta1*0.1; // CTRL_2.s->spd->Fbk*0.0001; /// 131072.0 *2;
        // (*Axis4DAC).dac_watch[23] = curycontroller.theta2*0.1; // CTRL_1.s->spd->Ref*0.0001; // * ELEC_RAD_PER_SEC_2_RPM *0.002;
        // (*Axis4DAC).dac_watch[24] = curycontroller.dot_theta2*0.1; // CTRL_1.s->spd->Fbk*0.0001; //i->varOmega  * ELEC_RAD_PER_SEC_2_RPM *0.002;
    //    (*Axis4DAC).dac_watch[25] = CTRL_1.i->cmd_iDQ[1] * 0.1;
    //    (*Axis4DAC).dac_watch[26] = CTRL_2.i->cmd_iDQ[1] * 0.1;

    //    (*Axis4DAC).dac_watch[22] = target_position_cnt*0.0003-((*CTRL).enc->encoder_abs_cnt)*0.0003;
    //    (*Axis4DAC).dac_watch[23] = GpioDataRegs.GPEDAT.bit.GPIO135;
    //    (*Axis4DAC).dac_watch[24] = GpioDataRegs.GPEDAT.bit.GPIO136;
    //    (*Axis4DAC).dac_watch[25] = GpioDataRegs.GPEDAT.bit.GPIO137;
    //    (*Axis4DAC).dac_watch[22] = marino.xTL*0.025;
    //    (*Axis4DAC).dac_watch[23] = marino.xRho*0.1;
    //    (*Axis4DAC).dac_watch[24] = marino.xAlpha*0.1;
    //    (*Axis4DAC).dac_watch[25] = marino.e_psi_Dmu;
    //    (*Axis4DAC).dac_watch[26] = marino.e_psi_Qmu;
        // (*Axis4DAC).dac_watch[27] = position_count_CAN_ID0x03_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;
        // (*Axis4DAC).dac_watch[28] = position_count_CAN_ID0x01_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;

        (*Axis4DAC).dac_watch[30] = (*CTRL).i->cmd_iDQ[0]*0.1;
        (*Axis4DAC).dac_watch[31] = (*CTRL).i->cmd_iDQ[1]*0.1;
        (*Axis4DAC).dac_watch[32] = yk;
        (*Axis4DAC).dac_watch[33] = uk;
#if WHO_IS_USER == USER_YZZ
        (*Axis4DAC).dac_watch[34] = FE.htz.sat_min_time[0]*100;
        (*Axis4DAC).dac_watch[35] = FE.htz.sat_min_time[1]*100;
        (*Axis4DAC).dac_watch[36] = vvvf_voltage * cos(vvvf_frequency*2*M_PI*(*CTRL).timebase);
        (*Axis4DAC).dac_watch[39] = FE.htz.theta_d*0.1;
#endif
        (*Axis4DAC).dac_watch[40] = (*CTRL).i->iDQ[0]*0.1;
        (*Axis4DAC).dac_watch[41] = (*CTRL).i->iDQ[1]*0.1;

        (*Axis4DAC).dac_watch[42] = CTRL_2.s->iQ->Ref; /// 131072.0 ;
        (*Axis4DAC).dac_watch[43] = CTRL_2.s->iQ->Fbk; /// 131072.0 ;
        (*Axis4DAC).dac_watch[44] = CTRL_2.s->Position->Ref *7.62939453125e-06; /// 131072.0 ;
        (*Axis4DAC).dac_watch[45] = CTRL_2.s->Position->Fbk *7.62939453125e-06; /// 131072.0 ;

        (*Axis4DAC).dac_watch[49] = CTRL_1.i->varOmega  * MECH_RAD_PER_SEC_2_RPM *0.002;
        (*Axis4DAC).dac_watch[50] = CTRL_1.i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM *0.002;
#if WHO_IS_USER == USER_YZZ
        (*Axis4DAC).dac_watch[51] = AFE_USED.theta_d *0.1;
        (*Axis4DAC).dac_watch[52] = (*Axis4DAC).used_theta_d_elec *0.1; // encoder angle
#endif
        // ZJL IMPEDENCE CONTROL
        (*Axis4DAC).dac_watch[53] = HIP_PREV_ANGLE;
        (*Axis4DAC).dac_watch[54] = SHANK_PREV_ANGLE;
        (*Axis4DAC).dac_watch[55] = IMPENDENCE_HIP_D_ANGULAR*0.1;
        (*Axis4DAC).dac_watch[56] = IMPENDENCE_SHANK_D_ANGULAR*0.1;
        (*Axis4DAC).dac_watch[57] = IQOUT_HIP*0.1;
        (*Axis4DAC).dac_watch[58] = IQOUT_SHANK*0.1;
        (*Axis4DAC).dac_watch[59] = position_count_SCI_hip_fromCPU2*1e-08;
        (*Axis4DAC).dac_watch[60] = position_count_SCI_shank_fromCPU2*1e-08;

        (*Axis4DAC).dac_watch[61] = INV.ual_comp *0.125;
        (*Axis4DAC).dac_watch[62] = INV.ube_comp *0.125;
#if WHO_IS_USER == USER_YZZ
        (*Axis4DAC).dac_watch[63] = FE.no_sat.psi_2[0];
        (*Axis4DAC).dac_watch[64] = FE.no_sat.psi_2[1];
#endif

        # if ENABLE_COMMISSIONING == TRUE
            (*Axis4DAC).dac_watch[61] = COMM.Js * 1e3;
            (*Axis4DAC).dac_watch[62] = COMM.current_command*0.2;
        # endif


        if((*Axis4DAC).channels_preset==1){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 23;
            (*Axis4DAC).channels[1] = 24;
            (*Axis4DAC).channels[2] = 25;
            (*Axis4DAC).channels[3] = 26;
            (*Axis4DAC).channels[4] = 32;
            (*Axis4DAC).channels[5] = 33;
            (*Axis4DAC).channels[6] = 21;
            (*Axis4DAC).channels[7] = 22;
        }else if((*Axis4DAC).channels_preset==2){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 26;//49; // 42; //12;
            (*Axis4DAC).channels[1] = 41;//50; // 43; //4; //13;
            (*Axis4DAC).channels[2] = 23 ; // 44; //28;
            (*Axis4DAC).channels[3] = 24;//52; // 45;
            (*Axis4DAC).channels[4] = 42;
            (*Axis4DAC).channels[5] = 43;
            (*Axis4DAC).channels[6] = 21;
            (*Axis4DAC).channels[7] = 22;
        }else if((*Axis4DAC).channels_preset==3){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 44;//42; //12;
            (*Axis4DAC).channels[1] = 45;//43; //4; //13;
            (*Axis4DAC).channels[2] = 42; //28;
            (*Axis4DAC).channels[3] = 43;
            (*Axis4DAC).channels[4] = 21;
            (*Axis4DAC).channels[5] = 22;
            (*Axis4DAC).channels[6] = 23;
            (*Axis4DAC).channels[7] = 24;
        }else if((*Axis4DAC).channels_preset==4){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 21;
            (*Axis4DAC).channels[1] = 22;
            (*Axis4DAC).channels[2] = 23;
            (*Axis4DAC).channels[3] = 24;
            (*Axis4DAC).channels[4] = 4;
            (*Axis4DAC).channels[5] = 5;
            (*Axis4DAC).channels[6] = 3;
            (*Axis4DAC).channels[7] = 4;
        }else if((*Axis4DAC).channels_preset==5){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 53;
            (*Axis4DAC).channels[1] = 54;
            (*Axis4DAC).channels[2] = 55;
            (*Axis4DAC).channels[3] = 56;
            (*Axis4DAC).channels[4] = 57;
            (*Axis4DAC).channels[5] = 58;
            (*Axis4DAC).channels[6] = 59;
            (*Axis4DAC).channels[7] = 60;
        }else if((*Axis4DAC).channels_preset==6){(*Axis4DAC).channels_preset=0;
            (*Axis4DAC).channels[0] = 21;
            (*Axis4DAC).channels[1] = 22;
            (*Axis4DAC).channels[2] = 23;
            (*Axis4DAC).channels[3] = 24;
            (*Axis4DAC).channels[4] = 27;
            (*Axis4DAC).channels[5] = 28;
            (*Axis4DAC).channels[6] = 63;
            (*Axis4DAC).channels[7] = 64;
        }
        if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

            // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤，锟角碉拷锟斤拷效锟斤拷围锟斤拷 [-1, 1]锟斤拷
            // wubo:我用的这套设备dac：dac_watch在dsp中输出[-1,1]V,通过dac板子输出[-3,3V]
            (*Axis4DAC).dac_watch[0] = Axis_1.iuvw[0]*0.2;
            (*Axis4DAC).dac_watch[1] = Axis_1.iuvw[1]*0.2;
            (*Axis4DAC).dac_watch[2] = Axis_1.iuvw[2]*0.2;
            (*Axis4DAC).dac_watch[3] = Axis_2.iuvw[3]*0.2;
            (*Axis4DAC).dac_watch[4] = Axis_2.iuvw[4]*0.2;
            (*Axis4DAC).dac_watch[5] = Axis_2.iuvw[5]*0.2;

            (*Axis4DAC).dac_watch[6] = (*CTRL).i->iAB[0]*0.25;      // +-6A ->0-3V
            (*Axis4DAC).dac_watch[7] = (*CTRL).i->iAB[1]*0.25;
            (*Axis4DAC).dac_watch[8] = (*CTRL).o->cmd_uAB[0]*0.125; // +-12V -> 0-3V
            (*Axis4DAC).dac_watch[9] = (*CTRL).o->cmd_uAB[1]*0.125;
#if WHO_IS_USER == USER_YZZ
            (*Axis4DAC).dac_watch[10] = FE.htz.psi_2_ampl*0.25;
            (*Axis4DAC).dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.25;
            (*Axis4DAC).dac_watch[12] = FE.htz.psi_2[0]*0.25;
            (*Axis4DAC).dac_watch[13] = FE.htz.psi_2[1]*0.25;
            (*Axis4DAC).dac_watch[14] = FE.htz.theta_d*0.1;
            (*Axis4DAC).dac_watch[15] = FE.htz.u_offset[0]*2;
            (*Axis4DAC).dac_watch[16] = FE.htz.u_offset[1]*2;
#endif
            (*Axis4DAC).dac_watch[17] = (*CTRL).svgen2.Ta;
            (*Axis4DAC).dac_watch[18] = (*CTRL).svgen2.Tb;
            (*Axis4DAC).dac_watch[19] = (*CTRL).svgen2.Tc;
            (*Axis4DAC).dac_watch[20] = (*CTRL).s->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;

            // (*Axis4DAC).dac_watch[21] = Axis_1.Set_manual_rpm *0.002;
            (*Axis4DAC).dac_watch[21] = Axis_1.pCTRL->i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM * 0.002;; // 500RPM - 3V
            (*Axis4DAC).dac_watch[22] = Axis_1.pCTRL->i->varOmega     * MECH_RAD_PER_SEC_2_RPM * 0.002;
            (*Axis4DAC).dac_watch[21] = Axis_2.pCTRL->i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM * 0.002;; // 500RPM - 3V
            (*Axis4DAC).dac_watch[22] = Axis_2.pCTRL->i->varOmega     * MECH_RAD_PER_SEC_2_RPM * 0.002;

            (*Axis4DAC).dac_watch[25] = Axis_1.pCTRL->i->cmd_iDQ[1]*0.02;
            (*Axis4DAC).dac_watch[26] = Axis_1.pCTRL->i->iDQ[1]*0.02;
            // 0.6V->1A
            (*Axis4DAC).dac_watch[27] = Axis_2.pCTRL->i->cmd_iDQ[1]*0.2;
            (*Axis4DAC).dac_watch[28] = Axis_2.pCTRL->i->iDQ[1]*0.2; // 0.6V->1A

            (*Axis4DAC).dac_watch[29] = Axis_2.pCTRL->i->iDQ[0]*0.2; // 0.6V->1A



            // (*Axis4DAC).dac_watch[25] = (*CTRL).i->cmd_iDQ[0]*0.2;
            // (*Axis4DAC).dac_watch[26] = (*CTRL).i->iDQ[0]*0.2;
            // (*Axis4DAC).dac_watch[27] = (*CTRL).i->cmd_iDQ[1]*0.2;
            // (*Axis4DAC).dac_watch[28] = (*CTRL).i->iDQ[1]*0.2; // 0.6V->1A

        //    (*Axis4DAC).dac_watch[21] = marino.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
            // (*Axis4DAC).dac_watch[21] = curycontroller.theta1*0.1; // CTRL_2.s->spd->Ref*0.0001;//curycontroller.theta1*0.1; /// 131072.0 *2;
            // (*Axis4DAC).dac_watch[22] = curycontroller.dot_theta1*0.1; // CTRL_2.s->spd->Fbk*0.0001; /// 131072.0 *2;
            // (*Axis4DAC).dac_watch[23] = curycontroller.theta2*0.1; // CTRL_1.s->spd->Ref*0.0001; // * ELEC_RAD_PER_SEC_2_RPM *0.002;
            // (*Axis4DAC).dac_watch[24] = curycontroller.dot_theta2*0.1; // CTRL_1.s->spd->Fbk*0.0001; //i->varOmega  * ELEC_RAD_PER_SEC_2_RPM *0.002;
        //    (*Axis4DAC).dac_watch[25] = CTRL_1.i->cmd_iDQ[1] * 0.1;
        //    (*Axis4DAC).dac_watch[26] = CTRL_2.i->cmd_iDQ[1] * 0.1;

        //    (*Axis4DAC).dac_watch[22] = target_position_cnt*0.0003-((*CTRL).enc->encoder_abs_cnt)*0.0003;
        //    (*Axis4DAC).dac_watch[23] = GpioDataRegs.GPEDAT.bit.GPIO135;
        //    (*Axis4DAC).dac_watch[24] = GpioDataRegs.GPEDAT.bit.GPIO136;
        //    (*Axis4DAC).dac_watch[25] = GpioDataRegs.GPEDAT.bit.GPIO137;
        //    (*Axis4DAC).dac_watch[22] = marino.xTL*0.025;
        //    (*Axis4DAC).dac_watch[23] = marino.xRho*0.1;
        //    (*Axis4DAC).dac_watch[24] = marino.xAlpha*0.1;
        //    (*Axis4DAC).dac_watch[25] = marino.e_psi_Dmu;
        //    (*Axis4DAC).dac_watch[26] = marino.e_psi_Qmu;
            // (*Axis4DAC).dac_watch[27] = position_count_CAN_ID0x03_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;
            // (*Axis4DAC).dac_watch[28] = position_count_CAN_ID0x01_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;

            (*Axis4DAC).dac_watch[30] = (*CTRL).i->cmd_iDQ[0]*0.1;
            (*Axis4DAC).dac_watch[31] = (*CTRL).i->cmd_iDQ[1]*0.1;
            (*Axis4DAC).dac_watch[32] = yk;
            (*Axis4DAC).dac_watch[33] = uk;
#if WHO_IS_USER == USER_YZZ
            (*Axis4DAC).dac_watch[34] = FE.htz.sat_min_time[0]*100;
            (*Axis4DAC).dac_watch[35] = FE.htz.sat_min_time[1]*100;
            (*Axis4DAC).dac_watch[36] = vvvf_voltage * cos(vvvf_frequency*2*M_PI*(*CTRL).timebase);
            (*Axis4DAC).dac_watch[39] = FE.htz.theta_d*0.1;
#endif
            (*Axis4DAC).dac_watch[40] = (*CTRL).i->iDQ[0]*0.1;
            (*Axis4DAC).dac_watch[41] = (*CTRL).i->iDQ[1]*0.1;

            (*Axis4DAC).dac_watch[42] = CTRL_2.s->iQ->Ref; /// 131072.0 ;
            (*Axis4DAC).dac_watch[43] = CTRL_2.s->iQ->Fbk; /// 131072.0 ;
            (*Axis4DAC).dac_watch[44] = CTRL_2.s->Position->Ref *7.62939453125e-06; /// 131072.0 ;
            (*Axis4DAC).dac_watch[45] = CTRL_2.s->Position->Fbk *7.62939453125e-06; /// 131072.0 ;

            (*Axis4DAC).dac_watch[49] = (*CTRL).i->cmd_varOmega  * MECH_RAD_PER_SEC_2_RPM *0.002;
            //(*Axis4DAC).dac_watch[50] = nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
            (*Axis4DAC).dac_watch[50] = (*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM *0.002;
#if WHO_IS_USER == USER_YZZ
            (*Axis4DAC).dac_watch[51] = AFE_USED.theta_d *0.1;

#endif
            (*Axis4DAC).dac_watch[52] = (*Axis4DAC).used_theta_d_elec *0.1;

            // ZJL IMPEDENCE CONTROL
            (*Axis4DAC).dac_watch[53] = HIP_PREV_ANGLE;
            (*Axis4DAC).dac_watch[54] = SHANK_PREV_ANGLE;
            (*Axis4DAC).dac_watch[55] = IMPENDENCE_HIP_D_ANGULAR*0.1;
            (*Axis4DAC).dac_watch[56] = IMPENDENCE_SHANK_D_ANGULAR*0.1;
            (*Axis4DAC).dac_watch[57] = IQOUT_HIP*0.1;
            (*Axis4DAC).dac_watch[58] = IQOUT_SHANK*0.1;
            (*Axis4DAC).dac_watch[59] = position_count_SCI_hip_fromCPU2*1e-08;
            (*Axis4DAC).dac_watch[60] = position_count_SCI_shank_fromCPU2*1e-08;

            (*Axis4DAC).dac_watch[61] = INV.ual_comp *0.125;
            (*Axis4DAC).dac_watch[62] = INV.ube_comp *0.125;


            # if ENABLE_COMMISSIONING == TRUE
                (*Axis4DAC).dac_watch[61] = COMM.Js * 1e3;
                (*Axis4DAC).dac_watch[62] = COMM.current_command*0.2;
            # endif


            if((*Axis4DAC).channels_preset==1){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 31;
                (*Axis4DAC).channels[1] = 41;
                (*Axis4DAC).channels[2] = 25;
                (*Axis4DAC).channels[3] = 26;
                (*Axis4DAC).channels[4] = 49;
                (*Axis4DAC).channels[5] = 50;
                (*Axis4DAC).channels[6] = 21;
                (*Axis4DAC).channels[7] = 22;
            }else if((*Axis4DAC).channels_preset==2){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 26;//49; // 42; //12;
                (*Axis4DAC).channels[1] = 41;//50; // 43; //4; //13;
                (*Axis4DAC).channels[2] = 23 ; // 44; //28;
                (*Axis4DAC).channels[3] = 24;//52; // 45;
                (*Axis4DAC).channels[4] = 42;
                (*Axis4DAC).channels[5] = 43;
                (*Axis4DAC).channels[6] = 21;
                (*Axis4DAC).channels[7] = 22;
            }else if((*Axis4DAC).channels_preset==3){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 44;//42; //12;
                (*Axis4DAC).channels[1] = 45;//43; //4; //13;
                (*Axis4DAC).channels[2] = 42; //28;
                (*Axis4DAC).channels[3] = 43;
                (*Axis4DAC).channels[4] = 21;
                (*Axis4DAC).channels[5] = 22;
                (*Axis4DAC).channels[6] = 23;
                (*Axis4DAC).channels[7] = 24;
            }else if((*Axis4DAC).channels_preset==4){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 21;
                (*Axis4DAC).channels[1] = 22;
                (*Axis4DAC).channels[2] = 23;
                (*Axis4DAC).channels[3] = 24;
                (*Axis4DAC).channels[4] = 4;
                (*Axis4DAC).channels[5] = 5;
                (*Axis4DAC).channels[6] = 3;
                (*Axis4DAC).channels[7] = 4;
            }else if((*Axis4DAC).channels_preset==5){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 53;
                (*Axis4DAC).channels[1] = 54;
                (*Axis4DAC).channels[2] = 55;
                (*Axis4DAC).channels[3] = 56;
                (*Axis4DAC).channels[4] = 57;
                (*Axis4DAC).channels[5] = 58;
                (*Axis4DAC).channels[6] = 59;
                (*Axis4DAC).channels[7] = 60;
            }else if((*Axis4DAC).channels_preset==6){(*Axis4DAC).channels_preset=0;
                (*Axis4DAC).channels[0] = 21;
                (*Axis4DAC).channels[1] = 22;
                (*Axis4DAC).channels[2] = 23;
                (*Axis4DAC).channels[3] = 24;
                (*Axis4DAC).channels[4] = 49;
                (*Axis4DAC).channels[5] = 50;
                (*Axis4DAC).channels[6] = 61;
                (*Axis4DAC).channels[7] = 62;
            }

            // 锟斤拷通锟斤拷DAC锟斤拷锟斤拷锟斤拷锟斤拷薷锟�(*Axis4DAC).channels锟斤拷锟斤拷锟斤拷确锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫�(*Axis4DAC).dac_watch锟斤拷锟斤拷锟叫的憋拷锟斤拷锟斤拷
            Write.dac_buffer[0] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[0]] + (*Axis4DAC).dac_offset[0];
            Write.dac_buffer[1] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[1]] + (*Axis4DAC).dac_offset[1];
            Write.dac_buffer[2] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[2]] + (*Axis4DAC).dac_offset[2];
            Write.dac_buffer[3] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[3]] + (*Axis4DAC).dac_offset[3];
            Write.dac_buffer[4] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[4]] + (*Axis4DAC).dac_offset[4];
            Write.dac_buffer[5] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[5]] + (*Axis4DAC).dac_offset[5];
            Write.dac_buffer[6] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[6]] + (*Axis4DAC).dac_offset[6];
            Write.dac_buffer[7] = (*Axis4DAC).dac_watch[(*Axis4DAC).channels[7]] + (*Axis4DAC).dac_offset[7];

            if(bool_test_dac_sinusoidal == TRUE){
                Write.dac_buffer[0] = sin(CL_TS*counter);
                Write.dac_buffer[1] = cos(CL_TS*counter);
                Write.dac_buffer[2] = sin(CL_TS*counter);
                Write.dac_buffer[3] = cos(CL_TS*counter);
                Write.dac_buffer[4] = sin(CL_TS*counter);
                Write.dac_buffer[5] = cos(CL_TS*counter);
                Write.dac_buffer[6] = sin(CL_TS*counter);
                Write.dac_buffer[7] = cos(CL_TS*counter);
            }

        ////    Write.dac_buffer[0] = AdcaResultRegs.ADCRESULT3 * 0.000244140625;
        ////    Write.dac_buffer[1] = AdccResultRegs.ADCRESULT2 * 0.000244140625;
        //    Write.dac_buffer[2] = AdcaResultRegs.ADCRESULT1 * 0.000244140625; //VSO3
        ////    Write.dac_buffer[3] = AdccResultRegs.ADCRESULT4 * 0.000244140625;
        ////    Write.dac_buffer[4] = AdccResultRegs.ADCRESULT5 * 0.000244140625;
        ////    Write.dac_buffer[5] = AdccResultRegs.ADCRESULT5 * 0.000244140625;
        //    Write.dac_buffer[6] = AdccResultRegs.ADCRESULT5 * 0.000244140625;
        ////    Write.dac_buffer[7] = AdccResultRegs.ADCRESULT2 * 0.000244140625;

        //    Write.dac_buffer[0] = (*Axis4DAC).iuvw[0];
        //    Write.dac_buffer[1] = (*Axis4DAC).iuvw[1];
        //    Write.dac_buffer[2] = (*Axis4DAC).iuvw[2];
        //    Write.dac_buffer[3] = (*Axis4DAC).iuvw[0];
        //    Write.dac_buffer[4] = (*Axis4DAC).iuvw[1];
        //    Write.dac_buffer[5] = (*Axis4DAC).iuvw[2];
        //    Write.dac_buffer[6] = (*Axis4DAC).iuvw[0];
        //    Write.dac_buffer[7] = (*Axis4DAC).iuvw[1];
            counter += 1;

            (*Axis4DAC).dac_time += CL_TS;
            if(((*Axis4DAC).dac_time)<3){
                (*Axis4DAC).dac_offset[0] = 0.0045;
                (*Axis4DAC).dac_offset[1] = 0.004;
                (*Axis4DAC).dac_offset[2] = 0.0055;
                (*Axis4DAC).dac_offset[3] = 0.0035;
                (*Axis4DAC).dac_offset[4] = 0.003;
                (*Axis4DAC).dac_offset[5] =-0.0045;
                (*Axis4DAC).dac_offset[6] = 0.002;
                (*Axis4DAC).dac_offset[7] =-0.001;

                Write.dac_buffer[0] = (*Axis4DAC).dac_offset[0];
                Write.dac_buffer[1] = (*Axis4DAC).dac_offset[1];
                Write.dac_buffer[2] = (*Axis4DAC).dac_offset[2];
                Write.dac_buffer[3] = (*Axis4DAC).dac_offset[3];
                Write.dac_buffer[4] = (*Axis4DAC).dac_offset[4];
                Write.dac_buffer[5] = (*Axis4DAC).dac_offset[5];
                Write.dac_buffer[6] = (*Axis4DAC).dac_offset[6];
                Write.dac_buffer[7] = (*Axis4DAC).dac_offset[7];

        //        G.test_integer += 1;
        //        G.test_float   +=CL_TS;
            }

            IPCLtoRFlagSet(IPC_FLAG7);
        }
    }

#elif NUMBER_OF_DSP_CORES == 1

    float temp = 0.0;
    void single_core_dac(){
        (*Axis4DAC).dac_time += CL_TS;
        (*Axis4DAC).DAC_MAX5307_FLAG++;

        temp = sin((*Axis4DAC).dac_time);
        DAC_MAX5307(1, temp ); //71us 10khz
        DAC_MAX5307(2, temp  ); //71us 10khz
        DAC_MAX5307(3, temp  ); //71us 10khz
        DAC_MAX5307(4, temp  ); //71us 10khz
        DAC_MAX5307(5, temp  ); //71us 10khz
        DAC_MAX5307(6, temp  ); //71us 10khz
        DAC_MAX5307(7, temp  ); //71us 10khz
        DAC_MAX5307(8, temp  ); //71us 10khz
    }

    //    void single_core_dac(){
    //        (*Axis4DAC).dac_time += CL_TS;
    //        (*Axis4DAC).DAC_MAX5307_FLAG++;
    //
    //
    //
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==1)
    //        {
    //            DAC_MAX5307(1, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==2)
    //        {
    //            DAC_MAX5307(2, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==3)
    //        {
    //            DAC_MAX5307(3, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==4)
    //        {
    //            DAC_MAX5307(4, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //            (*Axis4DAC).DAC_MAX5307_FLAG = 1;
    //        }
    //
    //
    //
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==5)
    //        {
    //            DAC_MAX5307(5, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==6)
    //        {
    //            DAC_MAX5307(6, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //            //(*Axis4DAC).DAC_MAX5307_FLAG=2;//4 means from 3 to 6;
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==7)
    //        {
    //            DAC_MAX5307(7, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis4DAC).DAC_MAX5307_FLAG==8)
    //        {
    //            DAC_MAX5307(8, sin(CL_TS*(*Axis4DAC).dac_time) ); //71us 10khz
    //            (*Axis4DAC).DAC_MAX5307_FLAG = 1;
    //        }
    //    }
#endif
}

