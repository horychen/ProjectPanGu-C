/*
 * ShareMemory.c
 *
 *  Created on: 2021锟斤拷1锟斤拷15锟斤拷
 *      Author: JIAHAO
 *///cpu2 CONNECTION

#include "All_Definition.h"

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

        (*Axis4DAC).dac_watch[17] = (*CTRL).svgen2.Ta;
        (*Axis4DAC).dac_watch[18] = (*CTRL).svgen2.Tb;
        (*Axis4DAC).dac_watch[19] = (*CTRL).svgen2.Tc;
        (*Axis4DAC).dac_watch[20] = (*CTRL).s->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;


        /* Speed Loop Info */
        /* !!! Noticing PID_Speed only works when ENABLE_PWM_OUT !!! */
        (*Axis4DAC).dac_watch[21] = Axis_1.Pdebug->set_rpm_speed_command * 0.002; // 500RPM - 3V
        (*Axis4DAC).dac_watch[22] = Axis_1.pCTRL->i->varOmega * MECH_RAD_PER_SEC_2_RPM * 0.002;
        (*Axis4DAC).dac_watch[23] = PID_Speed->Err * MECH_RAD_PER_SEC_2_RPM * 0.002; // 500RPM - 3V
        (*Axis4DAC).dac_watch[24] = (PID_Speed->Ref - PID_Speed->Fbk) * MECH_RAD_PER_SEC_2_RPM * 0.002;

        /* D-Current Info */
        (*Axis4DAC).dac_watch[25] = (*CTRL).i->cmd_iDQ[0] * 0.1; // 0.6V -> 2A
        (*Axis4DAC).dac_watch[26] = (*CTRL).i->iDQ[0] * 0.1;
        (*Axis4DAC).dac_watch[27] = PID_iD->Err * 0.1;
        (*Axis4DAC).dac_watch[28] = (PID_iD->Ref - PID_iD->Fbk) * 0.1;

        /* Q-Current Info */
        (*Axis4DAC).dac_watch[29] = (*CTRL).i->cmd_iDQ[1] * 0.1; // 0.6V -> 2A
        (*Axis4DAC).dac_watch[30] = (*CTRL).i->iDQ[1] * 0.1;
        (*Axis4DAC).dac_watch[31] = PID_iQ->Err * 0.1;
        (*Axis4DAC).dac_watch[32] = PID_iQ->Out * 0.1;

        /* UDQ given test */
        (*Axis4DAC).dac_watch[33] = (*CTRL).o->cmd_uDQ[0] * 0.2;
        (*Axis4DAC).dac_watch[34] = (*CTRL).o->cmd_uDQ[1] * 0.2;
        (*Axis4DAC).dac_watch[35] = (*CTRL).i->iDQ[0] * 0.5;
        (*Axis4DAC).dac_watch[36] = (*CTRL).i->iDQ[1] * 0.5;
        (*Axis4DAC).dac_watch[37] = (*CTRL).i->iAB[0] * 0.5;


//        #if WHO_IS_USER == USER_WB
//            (*Axis4DAC).dac_watch[0] = 0;
//            (*Axis4DAC).dac_watch[1] = 0;
//            (*Axis4DAC).dac_watch[2] = 0;
//            (*Axis4DAC).dac_watch[3] = 0;
//            (*Axis4DAC).dac_watch[4] = 0;
//            (*Axis4DAC).dac_watch[5] = 0;
//            (*Axis4DAC).dac_watch[6] = 0;
//            (*Axis4DAC).dac_watch[7] = 0;
//        #endif


        #if WHO_IS_USER == USER_WB
                //* Speed Controller Output
            (*Axis4DAC).dac_watch[64] = PID_Speed->P_Term * 1;
            (*Axis4DAC).dac_watch[65] = PID_Speed->I_Term * 0.04 * wubo_debug_tools[0];
            (*Axis4DAC).dac_watch[66] = PID_Speed->KFB_Term * 0.04 * wubo_debug_tools[0];

            (*Axis4DAC).dac_watch[67] = PID_Speed->Out * 0.1;
            (*Axis4DAC).dac_watch[68] = PID_Speed->OutPrev *0.1;
            (*Axis4DAC).dac_watch[69] = (*CTRL).svgen1.utilization_ratio * 0.5;
        #endif

        #if WHO_IS_USER == USER_YZZ
            (*Axis4DAC).dac_watch[10] = FE.htz.psi_2_ampl*0.25;
            (*Axis4DAC).dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.25;
            (*Axis4DAC).dac_watch[12] = FE.htz.psi_2[0]*0.25;
            (*Axis4DAC).dac_watch[13] = FE.htz.psi_2[1]*0.25;
            (*Axis4DAC).dac_watch[14] = FE.htz.theta_d*0.1;
            (*Axis4DAC).dac_watch[15] = FE.htz.u_offset[0]*2;
            (*Axis4DAC).dac_watch[16] = FE.htz.u_offset[1]*2;
            (*Axis4DAC).dac_watch[34] = FE.htz.sat_min_time[0]*100;
            (*Axis4DAC).dac_watch[35] = FE.htz.sat_min_time[1]*100;
            (*Axis4DAC).dac_watch[51] = AFE_USED.theta_d *0.1;
            (*Axis4DAC).dac_watch[63] = FE.no_sat.psi_2[0];
            (*Axis4DAC).dac_watch[64] = FE.no_sat.psi_2[1];
        #endif

        # if ENABLE_COMMISSIONING == TRUE
            (*Axis4DAC).dac_watch[61] = COMM.Js * 1e3;
            (*Axis4DAC).dac_watch[62] = COMM.current_command*0.2;
        # endif

        
        // ZJL IMPEDENCE CONTROL
        // FOR LAOZHU change zero to the student ID
        #if WHO_IS_USER == 0
            (*Axis4DAC).dac_watch[53] = HIP_PREV_ANGLE;
            (*Axis4DAC).dac_watch[54] = SHANK_PREV_ANGLE;
            (*Axis4DAC).dac_watch[55] = IMPENDENCE_HIP_D_ANGULAR*0.1;
            (*Axis4DAC).dac_watch[56] = IMPENDENCE_SHANK_D_ANGULAR*0.1;
            (*Axis4DAC).dac_watch[57] = IQOUT_HIP*0.1;
            (*Axis4DAC).dac_watch[58] = IQOUT_SHANK*0.1;
            (*Axis4DAC).dac_watch[59] = position_count_SCI_hip_fromCPU2*1e-08;
            (*Axis4DAC).dac_watch[60] = position_count_SCI_shank_fromCPU2*1e-08;
        #endif


        if((*Axis4DAC).channels_preset==1){(*Axis4DAC).channels_preset=0;
            /* Speed Loop info with iQ current info */
            (*Axis4DAC).channels[0] = 21; // PID_Speed->Ref
            (*Axis4DAC).channels[1] = 22; // PID_Speed->Fbk
            (*Axis4DAC).channels[2] = 23; // PID_Speed->Err
            (*Axis4DAC).channels[3] = 24; // PID_Speed->Ref - PID_Speed->Fbk
            (*Axis4DAC).channels[4] = 29; // PID_iQ->Ref
            (*Axis4DAC).channels[5] = 30; // PID_iQ->Fbk
            (*Axis4DAC).channels[6] = 31; // PID_iQ->Err
            (*Axis4DAC).channels[7] = 32; // PID_iQ->Out
        }else if((*Axis4DAC).channels_preset==2){(*Axis4DAC).channels_preset=0;
            /* iD current and iQ current info */
            (*Axis4DAC).channels[0] = 25; // PID_iD->Ref
            (*Axis4DAC).channels[1] = 26; // PID_iD->Fbk
            (*Axis4DAC).channels[2] = 27; // PID_iD->Err
            (*Axis4DAC).channels[3] = 28; // PID_iD->Out
            (*Axis4DAC).channels[4] = 29;
            (*Axis4DAC).channels[5] = 30;
            (*Axis4DAC).channels[6] = 31;
            (*Axis4DAC).channels[7] = 32;
        }else if((*Axis4DAC).channels_preset==3){(*Axis4DAC).channels_preset=0;
            /* DAC offset tune */
            (*Axis4DAC).channels[0] = 29; // 
            (*Axis4DAC).channels[1] = 30; // 
            (*Axis4DAC).channels[2] = 2; // 0
            (*Axis4DAC).channels[3] = 3; // 0
            (*Axis4DAC).channels[4] = 4; // 0
            (*Axis4DAC).channels[5] = 5; // 0
            (*Axis4DAC).channels[6] = 6; // 0
            (*Axis4DAC).channels[7] = 7; // 0
        }else if((*Axis4DAC).channels_preset==4){(*Axis4DAC).channels_preset=0;
            /* uDQ given test */
            (*Axis4DAC).channels[0] = 33; // 0
            (*Axis4DAC).channels[1] = 34; // 0
            (*Axis4DAC).channels[2] = 35; // 0
            (*Axis4DAC).channels[3] = 36; // 0
            (*Axis4DAC).channels[4] = 37; // 0
            (*Axis4DAC).channels[5] = 5; // 0
            (*Axis4DAC).channels[6] = 6; // 0
            (*Axis4DAC).channels[7] = 7; // 0
        }
        if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){
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
            /* for DAC test */
//            Write.dac_buffer[0] = sin(CL_TS*counter);
//            Write.dac_buffer[1] = cos(CL_TS*counter);
//            Write.dac_buffer[2] = sin(CL_TS*counter);
//            Write.dac_buffer[3] = cos(CL_TS*counter);
//            Write.dac_buffer[4] = sin(CL_TS*counter);
//            Write.dac_buffer[5] = cos(CL_TS*counter);
//            Write.dac_buffer[6] = sin(CL_TS*counter);
//            Write.dac_buffer[7] = cos(CL_TS*counter);



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

