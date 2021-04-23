/*
 * ShareMemory.c
 *
 *  Created on: 2021年1月15日
 *      Author: JIAHAO
 *///cpu2 CONNECTION

#include <All_Definition.h>

#if NUMBER_OF_DSP_CORES == 2

struct IPC_MEMORY_WRITE Write;
struct IPC_MEMORY_READ Read;
#pragma DATA_SECTION(Write, "SHARERAMGS1"); // GS1 is write
#pragma DATA_SECTION( Read, "SHARERAMGS0");

REAL dac_watch[30];
#define NO_OF_CHANNELS 6
//int channels[4]={2,3,14,15}; // 自整定
//int channels[4]={2,3,14,16}; // 自整定
//int channels[4]={4,5,6,7};
//int channels[4]={10,12,6,7};
//int channels[4]={10,11,20,21}; // 磁链和位置观测实验
//int channels[4]={6,7,20,21}; // 低速运行实验

//int channels[NO_OF_CHANNELS]={5,23,4,21,6,7}; // 低速运行实验
//int channels[NO_OF_CHANNELS]={5,23,20,21,6,7}; // 低速运行实验


//int channels[4]={26,27,5,22}; // 高速运行实验
//int channels[NO_OF_CHANNELS]={5,23,26,27,22,19}; // 高速运行实验
int channels[NO_OF_CHANNELS]={5,23,26,27,12,22}; // 高速运行实验

REAL dac_time = 0;
REAL temp_xOmg = 0.0;
REAL temp_xOmg_filtered = 0.0;

Uint32 e;
Uint32 b;
Uint32 c;
REAL d;

//REAL temp_xOmg2 = 0.0;
//REAL temp_xOmg_filtered2 = 0.0;

void write_DAC_buffer(){
    dac_time += 1e-4;
    if(IPCRtoLFlagBusy(IPC_FLAG7) == 0){

        // 所有曾经看过的变量都在列在这里，记得有效范围是 [-1, 1]。
        dac_watch[0] = Current_U*0.2;
        dac_watch[1] = Current_V*0.2;
        //dac_watch[10] = Current_W*0.2;
        dac_watch[2] = CTRL.I->iab[0]*0.2;
        dac_watch[3] = CTRL.I->iab[1]*0.2;
        dac_watch[4] = CTRL.I->idq[0]*0.2;
        //dac_watch[5] = CTRL.I->idq[1]*0.2; // 5 A for low speed
        dac_watch[5] = CTRL.I->idq[1]*0.05; // 20 A for high speed reversal
        dac_watch[6] = CTRL.I->rpm*0.002;
        dac_watch[7] = nsoaf.xOmg*ELEC_RAD_PER_SEC_2_RPM*0.002;
        dac_watch[8] = EQep1Regs.QPOSCNT*0.0001; // [0, 10000]
        dac_watch[9] = Current_U - CTRL.I->iab[0];

        //dac_watch[11] = COMM.current_sum*0.05/(float32)COMM.counterSS;
        //dac_watch[12] = COMM.voltage_sum*0.05/(float32)COMM.counterSS;
        //dac_watch[13] = COMM.counterSS*0.001;

        dac_watch[10] = nsoaf.active_flux_ab[0];
        dac_watch[11] = nsoaf.active_flux_ab[1];
        dac_watch[12] = nsoaf.afest_states[0];
        dac_watch[13] = nsoaf.afest_states[1];

        dac_watch[14] = CTRL.O->uab_cmd_to_inverter[0]*0.01;
        dac_watch[15] = CTRL.O->uab_cmd_to_inverter[1]*0.01;
        dac_watch[16] = CTRL.O->uab_cmd[0]*0.01;
        dac_watch[17] = CTRL.O->uab_cmd[1]*0.01;

        dac_watch[18] = CTRL.O->udq_cmd[0]*0.01;
        dac_watch[19] = CTRL.O->udq_cmd[1]*0.01;

        dac_watch[20] = ENC.theta_d_elec*0.1;
        dac_watch[21] = ELECTRICAL_POSITION_FEEDBACK*0.1;

        dac_watch[22] = nsoaf.LoadTorquePI*0.25;
        //dac_watch[23] = nsoaf.xIq*0.2; // 5 A for low speed
        dac_watch[23] = nsoaf.xIq*0.05; // 20 A for high speed reversal

        dac_watch[26] = CTRL.I->rpm*0.0005;
        dac_watch[27] = nsoaf.xOmg*ELEC_RAD_PER_SEC_2_RPM*0.0005;


        // information from d-axis voltage equation
        //temp_xOmg = (CTRL.O->udq_cmd[0] - CTRL.motor->R * CTRL.I->idq[0] ) / -CTRL.motor->Lq*CTRL.I->idq[1];
        //temp_xOmg_filtered = _lpf(temp_xOmg, temp_xOmg_filtered, 20.0);


        // 四通道DAC输出，请修改channels数组来确定具体输出哪些dac_watch数组中的变量。
        Write.dac_buffer[0] = dac_watch[channels[0]] + G.dac_offset[0];
        Write.dac_buffer[1] = dac_watch[channels[1]] + G.dac_offset[1];
        Write.dac_buffer[2] = dac_watch[channels[2]] + G.dac_offset[2];
        Write.dac_buffer[3] = dac_watch[channels[3]] + G.dac_offset[3];
        Write.dac_buffer[4] = dac_watch[channels[4]] + G.dac_offset[4];
        Write.dac_buffer[5] = dac_watch[channels[5]] + G.dac_offset[5];

        if(dac_time<10){
            G.dac_offset[0] =-0.003;
            G.dac_offset[1] = 0.003;
            G.dac_offset[2] = 0.0055;
            G.dac_offset[3] = 0.0087;
            G.dac_offset[4] = 0.006;
            G.dac_offset[5] = 0.002;

            Write.dac_buffer[0] = G.dac_offset[0];
            Write.dac_buffer[1] = G.dac_offset[1];
            Write.dac_buffer[2] = G.dac_offset[2];
            Write.dac_buffer[3] = G.dac_offset[3];
            Write.dac_buffer[4] = G.dac_offset[4];
            Write.dac_buffer[5] = G.dac_offset[5];

            e+=1;
            b+=1;c+=1;d=dac_time;
        }

        IPCLtoRFlagSet(IPC_FLAG7);
    }
}


extern int DAC_MAX5307_FLAG;
void single_core_dac(){
    DAC_MAX5307_FLAG++;
    if (DAC_MAX5307_FLAG==1)
    {
        DAC_MAX5307(1, Current_V*0.1 );//71us 10khz
    }
    if (DAC_MAX5307_FLAG==2)
    {
        DAC_MAX5307(2, Current_W*0.1 );
    }
    if (DAC_MAX5307_FLAG==3)
    {
        DAC_MAX5307(3, Current_W*0.1 );//phase w
    }
    if (DAC_MAX5307_FLAG==4)
    {
        DAC_MAX5307(4, 0.1 );//Id
    }
    if (DAC_MAX5307_FLAG==5)
    {
        DAC_MAX5307(5, 0.1 );
    }
    if (DAC_MAX5307_FLAG==6)
    {
        DAC_MAX5307(6, 0.1 );//iq
        DAC_MAX5307_FLAG=2;//4 means from 3 to 6;
    }
    if (DAC_MAX5307_FLAG==7)
    {
        DAC_MAX5307(7, 0.1 );
    }
    if (DAC_MAX5307_FLAG==8)
    {
        DAC_MAX5307(8, 0.1 );
    }    
}

#endif


