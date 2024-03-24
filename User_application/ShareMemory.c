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

//extern REAL target_position_cnt = 5000;
//extern REAL KP = 0.05;
//extern REAL direction


extern int use_first_set_three_phase;

void write_DAC_buffer(){
if(use_first_set_three_phase==2){
    Axis4DAC = &Axis_2;
}else{
    Axis4DAC = &Axis_1;
}
if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤，锟角碉拷锟斤拷效锟斤拷围锟斤拷 [-1, 1]锟斤拷
    (*Axis4DAC).dac_watch[0] = Axis_1.iuvw[0]*0.2;
    (*Axis4DAC).dac_watch[1] = Axis_1.iuvw[1]*0.2;
    (*Axis4DAC).dac_watch[2] = Axis_1.iuvw[2]*0.2;
    (*Axis4DAC).dac_watch[3] = Axis_2.iuvw[3]*0.2;
    (*Axis4DAC).dac_watch[4] = Axis_2.iuvw[4]*0.2;
    (*Axis4DAC).dac_watch[5] = Axis_2.iuvw[5]*0.2;

    (*Axis4DAC).dac_watch[6] = (*CTRL).i->iab[0]*0.25;      // +-6A ->0-3V
    (*Axis4DAC).dac_watch[7] = (*CTRL).i->iab[1]*0.25;
    (*Axis4DAC).dac_watch[8] = (*CTRL).o->uab_cmd[0]*0.125; // +-12V -> 0-3V
    (*Axis4DAC).dac_watch[9] = (*CTRL).o->uab_cmd[1]*0.125;
    (*Axis4DAC).dac_watch[10] = FE.htz.psi_2_ampl*0.25;
    (*Axis4DAC).dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.25;
    (*Axis4DAC).dac_watch[12] = FE.htz.psi_2[0]*0.25;
    (*Axis4DAC).dac_watch[13] = FE.htz.psi_2[1]*0.25;
    (*Axis4DAC).dac_watch[14] = FE.htz.theta_d*0.1;
    (*Axis4DAC).dac_watch[15] = FE.htz.u_offset[0]*2;
    (*Axis4DAC).dac_watch[16] = FE.htz.u_offset[1]*2;

    (*Axis4DAC).dac_watch[17] = (*CTRL).svgen2.Ta;
    (*Axis4DAC).dac_watch[18] = (*CTRL).svgen2.Tb;
    (*Axis4DAC).dac_watch[19] = (*CTRL).svgen2.Tc;
    (*Axis4DAC).dac_watch[20] = (*CTRL).s->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;

//    (*Axis4DAC).dac_watch[21] = marino.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis4DAC).dac_watch[21] = CTRL_1.s->pos->Ref *1.52587890625e-05; /// 131072.0 *2;
    (*Axis4DAC).dac_watch[22] = CTRL_2.s->pos->Ref *1.52587890625e-05; /// 131072.0 *2;
    (*Axis4DAC).dac_watch[23] = CTRL_2.s->spd->Ref *0.0001; // * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis4DAC).dac_watch[24] = CTRL_2.s->spd->Fbk *0.0001; //i->omg_elec  * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis4DAC).dac_watch[25] = CTRL_1.i->idq_cmd[1] * 0.1;
    (*Axis4DAC).dac_watch[26] = CTRL_2.i->idq_cmd[1] * 0.1;

//    (*Axis4DAC).dac_watch[22] = target_position_cnt*0.0003-((*CTRL).enc->encoder_abs_cnt)*0.0003;
//    (*Axis4DAC).dac_watch[23] = GpioDataRegs.GPEDAT.bit.GPIO135;
//    (*Axis4DAC).dac_watch[24] = GpioDataRegs.GPEDAT.bit.GPIO136;
//    (*Axis4DAC).dac_watch[25] = GpioDataRegs.GPEDAT.bit.GPIO137;
//    (*Axis4DAC).dac_watch[22] = marino.xTL*0.025;
//    (*Axis4DAC).dac_watch[23] = marino.xRho*0.1;
//    (*Axis4DAC).dac_watch[24] = marino.xAlpha*0.1;
//    (*Axis4DAC).dac_watch[25] = marino.e_psi_Dmu;
//    (*Axis4DAC).dac_watch[26] = marino.e_psi_Qmu;
    (*Axis4DAC).dac_watch[27] = position_count_CAN_ID0x03_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;
    (*Axis4DAC).dac_watch[28] = position_count_CAN_ID0x01_fromCPU2 *1.52587890625e-05; // / 131072.0 *2;

    (*Axis4DAC).dac_watch[30] = (*CTRL).o->iab_cmd[0]*0.2;
    (*Axis4DAC).dac_watch[31] = (*CTRL).o->iab_cmd[1]*0.2;
    (*Axis4DAC).dac_watch[32] = FE.htz.sat_max_time[0]*100;
    (*Axis4DAC).dac_watch[33] = FE.htz.sat_max_time[1]*100;
    (*Axis4DAC).dac_watch[34] = FE.htz.sat_min_time[0]*100;
    (*Axis4DAC).dac_watch[35] = FE.htz.sat_min_time[1]*100;
    (*Axis4DAC).dac_watch[36] = vvvf_voltage * cos(vvvf_frequency*2*M_PI*(*CTRL).timebase);
    (*Axis4DAC).dac_watch[39] = FE.htz.theta_d*0.1;

    (*Axis4DAC).dac_watch[40] = (*CTRL).i->idq[0]*0.1;
    (*Axis4DAC).dac_watch[41] = (*CTRL).i->idq[1]*0.1;

    (*Axis4DAC).dac_watch[42] = CTRL_1.s->pos->Ref *7.62939453125e-06; /// 131072.0 ;
    (*Axis4DAC).dac_watch[43] = CTRL_1.s->pos->Fbk *7.62939453125e-06; /// 131072.0 ;
    (*Axis4DAC).dac_watch[44] = CTRL_2.s->pos->Ref *7.62939453125e-06; /// 131072.0 ;
    (*Axis4DAC).dac_watch[45] = CTRL_2.s->pos->Fbk *7.62939453125e-06; /// 131072.0 ;

    (*Axis4DAC).dac_watch[49] = CTRL_1.i->omg_elec  * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis4DAC).dac_watch[50] = nsoaf.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis4DAC).dac_watch[51] = AFE_USED.theta_d *0.1;
    (*Axis4DAC).dac_watch[52] = (*Axis4DAC).used_theta_d_elec *0.1;

    if((*Axis4DAC).channels_preset==1){(*Axis4DAC).channels_preset=0;
        (*Axis4DAC).channels[0] = 23;
        (*Axis4DAC).channels[1] = 24;
        (*Axis4DAC).channels[2] = 25;
        (*Axis4DAC).channels[3] = 26;
        (*Axis4DAC).channels[4] = 27;
        (*Axis4DAC).channels[5] = 28;
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
        (*Axis4DAC).channels[6] = 41;
        (*Axis4DAC).channels[7] = 41;
    }else if((*Axis4DAC).channels_preset==4){(*Axis4DAC).channels_preset=0;
        (*Axis4DAC).channels[0] = 42;
        (*Axis4DAC).channels[1] = 43;
        (*Axis4DAC).channels[2] = 2;
        (*Axis4DAC).channels[3] = 4;
        (*Axis4DAC).channels[4] = 4;
        (*Axis4DAC).channels[5] = 5;
        (*Axis4DAC).channels[6] = 3;
        (*Axis4DAC).channels[7] = 4;
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

//    Write.dac_buffer[0] = sin(CL_TS*counter);
//    Write.dac_buffer[1] = cos(CL_TS*counter);
//    Write.dac_buffer[2] = sin(CL_TS*counter);
//    Write.dac_buffer[3] = cos(CL_TS*counter);
//    Write.dac_buffer[4] = sin(CL_TS*counter);
//    Write.dac_buffer[5] = cos(CL_TS*counter);
//    Write.dac_buffer[6] = sin(CL_TS*counter);
//    Write.dac_buffer[7] = cos(CL_TS*counter);
////
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


