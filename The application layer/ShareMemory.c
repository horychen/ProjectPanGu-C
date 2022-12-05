/*
 * ShareMemory.c
 *
 *  Created on: 2021��1��15��
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
extern REAL used_theta_d_elec;

void write_DAC_buffer(){
if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

    // �������������ı���������������ǵ���Ч��Χ�� [-1, 1]��
    Axis.dac_watch[0] = Axis.iuvw[2]*0.2;
    Axis.dac_watch[1] = Axis.iuvw[1]*0.2;
    Axis.dac_watch[2] = CTRL.I->iab[0]*0.2;
    Axis.dac_watch[3] = CTRL.I->iab[1]*0.2;
    Axis.dac_watch[4] = CTRL.O->uab_cmd[0]*0.01;
    Axis.dac_watch[5] = CTRL.O->uab_cmd[1]*0.01;
    Axis.dac_watch[10] = FE.htz.psi_2_ampl*0.5;
    Axis.dac_watch[11] = FE.htz.psi_2_ampl_lpf*0.5;
    Axis.dac_watch[12] = FE.htz.psi_2[0]*0.5;
    Axis.dac_watch[13] = FE.htz.psi_2[1]*0.5;
    Axis.dac_watch[14] = FE.htz.theta_d*0.1;
    Axis.dac_watch[15] = FE.htz.u_offset[0]*2;
    Axis.dac_watch[16] = FE.htz.u_offset[1]*2;

    Axis.dac_watch[17] = marino.xOmg * ELEC_RAD_PER_SEC_2_RPM *0.002;
    Axis.dac_watch[18] = marino.xTL*0.1;
    Axis.dac_watch[19] = marino.xRho*0.1;
    Axis.dac_watch[20] = marino.xAlpha*0.1;

    Axis.dac_watch[21] = CTRL.svgen2.Ta;
    Axis.dac_watch[22] = CTRL.svgen2.Tb;
    Axis.dac_watch[23] = CTRL.svgen2.Tc;
    Axis.dac_watch[24] = CTRL.S->omega_syn * ELEC_RAD_PER_SEC_2_RPM *0.002;

    if(Axis.channels_preset==1){Axis.channels_preset=0;
        /* Marino 2005 Sensorless Control */
        Axis.channels[0] = 15; //12;
        Axis.channels[1] = 16; //13;
        Axis.channels[2] = 24;
        //        Axis.channels[3] = 14;
        //        Axis.channels[4] = 15;
        //        Axis.channels[5] = 16;
        Axis.channels[3] = 17;
        Axis.channels[4] = 18;
        Axis.channels[5] = 21;
    }else if(Axis.channels_preset==8){Axis.channels_preset=0;
    }

    // ��ͨ��DAC��������޸�Axis.channels������ȷ�����������ЩAxis.dac_watch�����еı�����
    Write.dac_buffer[0] = Axis.dac_watch[Axis.channels[0]] + Axis.dac_offset[0];
    Write.dac_buffer[1] = Axis.dac_watch[Axis.channels[1]] + Axis.dac_offset[1];
    Write.dac_buffer[2] = Axis.dac_watch[Axis.channels[2]] + Axis.dac_offset[2];
    Write.dac_buffer[3] = Axis.dac_watch[Axis.channels[3]] + Axis.dac_offset[3];
    Write.dac_buffer[4] = Axis.dac_watch[Axis.channels[4]] + Axis.dac_offset[4];
    Write.dac_buffer[5] = Axis.dac_watch[Axis.channels[5]] + Axis.dac_offset[5];
    Write.dac_buffer[6] = Axis.dac_watch[Axis.channels[6]] + Axis.dac_offset[6];
    Write.dac_buffer[7] = Axis.dac_watch[Axis.channels[7]] + Axis.dac_offset[7];

    Axis.dac_time += CL_TS;
    if((Axis.dac_time)<3){
        Axis.dac_offset[0] = 0.0045;
        Axis.dac_offset[1] = 0.004;
        Axis.dac_offset[2] = 0.0055;
        Axis.dac_offset[3] = 0.0035;
        Axis.dac_offset[4] = 0.003;
        Axis.dac_offset[5] =-0.0045;
        Axis.dac_offset[6] = 0.002;
        Axis.dac_offset[7] =-0.001;

        Write.dac_buffer[0] = Axis.dac_offset[0];
        Write.dac_buffer[1] = Axis.dac_offset[1];
        Write.dac_buffer[2] = Axis.dac_offset[2];
        Write.dac_buffer[3] = Axis.dac_offset[3];
        Write.dac_buffer[4] = Axis.dac_offset[4];
        Write.dac_buffer[5] = Axis.dac_offset[5];
        Write.dac_buffer[6] = Axis.dac_offset[6];
        Write.dac_buffer[7] = Axis.dac_offset[7];

        G.test_integer += 1;
        G.test_float   +=CL_TS;
    }

    IPCLtoRFlagSet(IPC_FLAG7);
}
}

#elif NUMBER_OF_DSP_CORES == 1

    void single_core_dac(){
        G.DAC_MAX5307_FLAG++;
        if (G.DAC_MAX5307_FLAG==1)
        {
            DAC_MAX5307(1, G.Current_V*0.1 );//71us 10khz
        }
        if (G.DAC_MAX5307_FLAG==2)
        {
            DAC_MAX5307(2, G.Current_W*0.1 );
        }
        if (G.DAC_MAX5307_FLAG==3)
        {
            DAC_MAX5307(3, G.Current_W*0.1 );//phase w
        }
        if (G.DAC_MAX5307_FLAG==4)
        {
            DAC_MAX5307(4, 0.1 );//Id
        }
        if (G.DAC_MAX5307_FLAG==5)
        {
            DAC_MAX5307(5, 0.1 );
        }
        if (G.DAC_MAX5307_FLAG==6)
        {
            DAC_MAX5307(6, 0.1 );//iq
            G.DAC_MAX5307_FLAG=2;//4 means from 3 to 6;
        }
        if (G.DAC_MAX5307_FLAG==7)
        {
            DAC_MAX5307(7, 0.1 );
        }
        if (G.DAC_MAX5307_FLAG==8)
        {
            DAC_MAX5307(8, 0.1 );
        }
    }
#endif


