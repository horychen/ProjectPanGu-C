/*
 * ShareMemory.c
 *
 *  Created on: 2021锟斤拷1锟斤拷15锟斤拷
 *      Author: JIAHAO
 *///cpu2 CONNECTION

#include <All_Definition.h>

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


void write_DAC_buffer(){
if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤，锟角碉拷锟斤拷效锟斤拷围锟斤拷 [-1, 1]锟斤拷
    (*Axis).dac_watch[0] = (*Axis).iuvw[0]*0.2;
    (*Axis).dac_watch[1] = (*Axis).iuvw[1]*0.2;
    (*Axis).dac_watch[2] = (*Axis).iuvw[2]*0.2;
    (*Axis).dac_watch[3] = (*Axis).iuvw[3]*0.2;
    (*Axis).dac_watch[4] = (*Axis).iuvw[4]*0.2;
    (*Axis).dac_watch[5] = (*Axis).iuvw[5]*0.2;

    (*Axis).dac_watch[6] = (*CTRL).I->iab[0]*0.25;      // +-6A ->0-3V
    (*Axis).dac_watch[7] = (*CTRL).I->iab[1]*0.25;
    (*Axis).dac_watch[8] = (*CTRL).O->uab_cmd[0]*0.125; // +-12V -> 0-3V
    (*Axis).dac_watch[9] = (*CTRL).O->uab_cmd[1]*0.125;
    (*Axis).dac_watch[10] = FE.htz.psi_2_ampl*0.25;
    (*Axis).dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.25;
    (*Axis).dac_watch[12] = FE.htz.psi_2[0]*0.25;
    (*Axis).dac_watch[13] = FE.htz.psi_2[1]*0.25;
    (*Axis).dac_watch[14] = FE.htz.theta_d*0.1;
    (*Axis).dac_watch[15] = FE.htz.u_offset[0]*2;
    (*Axis).dac_watch[16] = FE.htz.u_offset[1]*2;

    (*Axis).dac_watch[17] = (*CTRL).svgen2.Ta;
    (*Axis).dac_watch[18] = (*CTRL).svgen2.Tb;
    (*Axis).dac_watch[19] = (*CTRL).svgen2.Tc;
    (*Axis).dac_watch[20] = (*CTRL).S->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;

//    (*Axis).dac_watch[21] = marino.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis).dac_watch[21] = (*Axis).Set_manual_rpm *0.002;
    (*Axis).dac_watch[22] = (*CTRL).I->omg_elec * ELEC_RAD_PER_SEC_2_RPM *0.002;
    (*Axis).dac_watch[23] = (*CTRL).I->idq_cmd[0]*0.2;
    (*Axis).dac_watch[24] = (*CTRL).I->idq[0]*0.2;

//    (*Axis).dac_watch[22] = target_position_cnt*0.0003-((*CTRL).enc->encoder_abs_cnt)*0.0003;
//    (*Axis).dac_watch[23] = GpioDataRegs.GPEDAT.bit.GPIO135;
//    (*Axis).dac_watch[24] = GpioDataRegs.GPEDAT.bit.GPIO136;
//    (*Axis).dac_watch[25] = GpioDataRegs.GPEDAT.bit.GPIO137;
//    (*Axis).dac_watch[22] = marino.xTL*0.025;
//    (*Axis).dac_watch[23] = marino.xRho*0.1;
//    (*Axis).dac_watch[24] = marino.xAlpha*0.1;
//    (*Axis).dac_watch[25] = marino.e_psi_Dmu;
//    (*Axis).dac_watch[26] = marino.e_psi_Qmu;
    (*Axis).dac_watch[27] = (*CTRL).I->idq_cmd[1]*0.1;
    (*Axis).dac_watch[28] = (*CTRL).I->idq[1]*0.1;

    (*Axis).dac_watch[30] = (*CTRL).O->iab_cmd[0]*0.2;
    (*Axis).dac_watch[31] = (*CTRL).O->iab_cmd[1]*0.2;
    (*Axis).dac_watch[32] = FE.htz.sat_max_time[0]*100;
    (*Axis).dac_watch[33] = FE.htz.sat_max_time[1]*100;
    (*Axis).dac_watch[34] = FE.htz.sat_min_time[0]*100;
    (*Axis).dac_watch[35] = FE.htz.sat_min_time[1]*100;
    (*Axis).dac_watch[39] = FE.htz.theta_d*0.1;

    (*Axis).dac_watch[40] = (*CTRL).I->idq[0]*0.1;
    (*Axis).dac_watch[41] = (*CTRL).I->idq[1]*0.1;

    if((*Axis).channels_preset==1){(*Axis).channels_preset=0;
        /* Marino 2005 Sensorless Control */
        (*Axis).channels[0] = 0;
        (*Axis).channels[1] = 1;
        (*Axis).channels[2] = 2;
        (*Axis).channels[3] = 28;
        (*Axis).channels[4] = 21;
        (*Axis).channels[5] = 22;
        (*Axis).channels[6] = 21;
        (*Axis).channels[7] = 22;
    }else if((*Axis).channels_preset==2){(*Axis).channels_preset=0;
        /* Marino 2005 Sensorless Control */
        (*Axis).channels[0] = 3; //12;
        (*Axis).channels[1] = 4;//4; //13;
        (*Axis).channels[2] = 5;//28;
        (*Axis).channels[3] = 41;
        (*Axis).channels[4] = 21;
        (*Axis).channels[5] = 22;
        (*Axis).channels[6] = 21;
        (*Axis).channels[7] = 22;
    }else if((*Axis).channels_preset==3){(*Axis).channels_preset=0;
        /* Marino 2005 Sensorless Control */
        (*Axis).channels[0] = 2; //12;
        (*Axis).channels[1] = 3;//4; //13;
        (*Axis).channels[2] = 12;//28;
        (*Axis).channels[3] = 13;
        (*Axis).channels[4] = 21;
        (*Axis).channels[5] = 22;
        (*Axis).channels[6] = 41;
        (*Axis).channels[7] = 41;
    }else if((*Axis).channels_preset==4){(*Axis).channels_preset=0;
        (*Axis).channels[0] = 0;
        (*Axis).channels[1] = 1;
        (*Axis).channels[2] = 2;
        (*Axis).channels[3] = 4;
        (*Axis).channels[4] = 4;
        (*Axis).channels[5] = 5;
        (*Axis).channels[6] = 3;
        (*Axis).channels[7] = 4;
    }

    // 锟斤拷通锟斤拷DAC锟斤拷锟斤拷锟斤拷锟斤拷薷锟�(*Axis).channels锟斤拷锟斤拷锟斤拷确锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫�(*Axis).dac_watch锟斤拷锟斤拷锟叫的憋拷锟斤拷锟斤拷
    Write.dac_buffer[0] = (*Axis).dac_watch[(*Axis).channels[0]] + (*Axis).dac_offset[0];
    Write.dac_buffer[1] = (*Axis).dac_watch[(*Axis).channels[1]] + (*Axis).dac_offset[1];
    Write.dac_buffer[2] = (*Axis).dac_watch[(*Axis).channels[2]] + (*Axis).dac_offset[2];
    Write.dac_buffer[3] = (*Axis).dac_watch[(*Axis).channels[3]] + (*Axis).dac_offset[3];
    Write.dac_buffer[4] = (*Axis).dac_watch[(*Axis).channels[4]] + (*Axis).dac_offset[4];
    Write.dac_buffer[5] = (*Axis).dac_watch[(*Axis).channels[5]] + (*Axis).dac_offset[5];
    Write.dac_buffer[6] = (*Axis).dac_watch[(*Axis).channels[6]] + (*Axis).dac_offset[6];
    Write.dac_buffer[7] = (*Axis).dac_watch[(*Axis).channels[7]] + (*Axis).dac_offset[7];

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

//    Write.dac_buffer[0] = (*Axis).iuvw[0];
//    Write.dac_buffer[1] = (*Axis).iuvw[1];
//    Write.dac_buffer[2] = (*Axis).iuvw[2];
//    Write.dac_buffer[3] = (*Axis).iuvw[0];
//    Write.dac_buffer[4] = (*Axis).iuvw[1];
//    Write.dac_buffer[5] = (*Axis).iuvw[2];
//    Write.dac_buffer[6] = (*Axis).iuvw[0];
//    Write.dac_buffer[7] = (*Axis).iuvw[1];
    counter += 1;

    (*Axis).dac_time += CL_TS;
    if(((*Axis).dac_time)<3){
        (*Axis).dac_offset[0] = 0.0045;
        (*Axis).dac_offset[1] = 0.004;
        (*Axis).dac_offset[2] = 0.0055;
        (*Axis).dac_offset[3] = 0.0035;
        (*Axis).dac_offset[4] = 0.003;
        (*Axis).dac_offset[5] =-0.0045;
        (*Axis).dac_offset[6] = 0.002;
        (*Axis).dac_offset[7] =-0.001;

        Write.dac_buffer[0] = (*Axis).dac_offset[0];
        Write.dac_buffer[1] = (*Axis).dac_offset[1];
        Write.dac_buffer[2] = (*Axis).dac_offset[2];
        Write.dac_buffer[3] = (*Axis).dac_offset[3];
        Write.dac_buffer[4] = (*Axis).dac_offset[4];
        Write.dac_buffer[5] = (*Axis).dac_offset[5];
        Write.dac_buffer[6] = (*Axis).dac_offset[6];
        Write.dac_buffer[7] = (*Axis).dac_offset[7];

//        G.test_integer += 1;
//        G.test_float   +=CL_TS;
    }

    IPCLtoRFlagSet(IPC_FLAG7);
}
}

#elif NUMBER_OF_DSP_CORES == 1

    float temp = 0.0;
    void single_core_dac(){
        (*Axis).dac_time += CL_TS;
        (*Axis).DAC_MAX5307_FLAG++;

        temp = sin((*Axis).dac_time);
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
    //        (*Axis).dac_time += CL_TS;
    //        (*Axis).DAC_MAX5307_FLAG++;
    //
    //
    //
    //        if ((*Axis).DAC_MAX5307_FLAG==1)
    //        {
    //            DAC_MAX5307(1, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==2)
    //        {
    //            DAC_MAX5307(2, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==3)
    //        {
    //            DAC_MAX5307(3, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==4)
    //        {
    //            DAC_MAX5307(4, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //            (*Axis).DAC_MAX5307_FLAG = 1;
    //        }
    //
    //
    //
    //        if ((*Axis).DAC_MAX5307_FLAG==5)
    //        {
    //            DAC_MAX5307(5, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==6)
    //        {
    //            DAC_MAX5307(6, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //            //(*Axis).DAC_MAX5307_FLAG=2;//4 means from 3 to 6;
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==7)
    //        {
    //            DAC_MAX5307(7, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //        }
    //        if ((*Axis).DAC_MAX5307_FLAG==8)
    //        {
    //            DAC_MAX5307(8, sin(CL_TS*(*Axis).dac_time) ); //71us 10khz
    //            (*Axis).DAC_MAX5307_FLAG = 1;
    //        }
    //    }
#endif


