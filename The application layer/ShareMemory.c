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


//int Axis.channels[4]={2,3,14,15}; // 自整定
//int Axis.channels[4]={2,3,14,16}; // 自整定
//int Axis.channels[4]={4,5,6,7};
//int Axis.channels[4]={10,12,6,7};
//int Axis.channels[4]={10,11,20,21}; // 磁链和位置观测实验
//int Axis.channels[4]={6,7,20,21}; // 低速运行实验

//int Axis.channels[NO_OF_DAC_Axis.channels]={5,23,4,21,6,7}; // 低速运行实验
//int Axis.channels[NO_OF_DAC_Axis.channels]={5,23,20,21,6,7}; // 低速运行实验


//int Axis.channels[4]={26,27,5,22}; // 高速运行实验
//int Axis.channels[NO_OF_DAC_Axis.channels]={5,23,26,27,22,19}; // 高速运行实验
//int Axis.channels[NO_OF_DAC_Axis.channels]={5,23,26,27,12,22}; // 高速运行实验


// ECAP
//int Axis.channels[NO_OF_DAC_Axis.channels]={0,1,4,5,6,7};
//int Axis.channels[NO_OF_DAC_Axis.channels]={0,1,32,34,33,35};
//int Axis.channels[NO_OF_DAC_Axis.channels]={0,1,4,38,32,33};

// NSOAF-Rev1
//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,20,21,5,23};
//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,30,32,31,33};
//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,20,21,32,33};

// ECAP
//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,40,42,41,43};
//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,44,45,46,47};
//int Axis.channels[NO_OF_DAC_Axis.channels]={46,47,20,21,6,7};

//int Axis.channels[NO_OF_DAC_Axis.channels]={46,47,20,21,6,7,59,59}; // ecap dq
//int Axis.channels[NO_OF_DAC_Axis.channels]={10,11,20,21,6,7,59,59}; // psi_2
//int Axis.channels[NO_OF_DAC_Axis.channels]={10,11,20,21,6,39,39,59}; // park trapezoidal
//int Axis.channels[NO_OF_DAC_Axis.channels]={10,11,20,21,6,7,39,59}; // park trapezoidal
//int Axis.channels[NO_OF_DAC_Axis.channels]={10,11,20,21,6,7,5,23}; // rev1 sensorless slow reversal

//int Axis.channels[NO_OF_DAC_Axis.channels]={51,52,20,21,6,7,54,59}; // test huwu 1998
//int Axis.channels[NO_OF_DAC_Axis.channels]={10,11,20,21,6,7,55,59}; // Compare to CM-VM fusion with only KP

//int Axis.channels[NO_OF_DAC_Axis.channels]={50,9,20,21,6,7,52,59}; // Chi.Xu 2009 SSR
//int Axis.channels[NO_OF_DAC_Axis.channels]={50,9,52,53,26,27,49,59}; // Chi.Xu 2009 High speed
//int Axis.channels[NO_OF_DAC_Axis.channels]={56,57,3,5,26,27,49,59}; // Chi.Xu 2009 High speed
//int Axis.channels[NO_OF_DAC_Axis.channels]={56,57,3,5,26,27,23,59}; // NSOAF High Speed

//int Axis.channels[NO_OF_DAC_Axis.channels]={6,7,49,59,20,21,47,48}; // ESOAF

//int Axis.channels[NO_OF_DAC_Axis.channels]={3,5,49,59,20,21,47,48}; // ESOAF

//int Axis.channels_preset = 9; //6

//REAL dac_time_that_cannot_be_modified = 0;
//REAL if_you_define_an_extra_global_variable_here_you_cannot_modify_dac_time_anymore = 0;


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

    // 所有曾经看过的变量都在列在这里，记得有效范围是 [-1, 1]。
    Axis.dac_watch[0] = Axis.iuvw[2]*0.2;
    Axis.dac_watch[1] = Axis.iuvw[1]*0.2;
    //Axis.dac_watch[2] = G.Current_U*0.2;
    //Axis.dac_watch[3] = G.Current_Not_Used*0.2;
    Axis.dac_watch[2] = CTRL.I->idq_cmd[0]*0.1;
    Axis.dac_watch[3] = CTRL.I->idq_cmd[1]*0.1; // 5 A for low speed
    Axis.dac_watch[4] = CTRL.I->idq[0]*0.1;
    Axis.dac_watch[5] = CTRL.I->idq[1]*0.1; // 5 A for low speed
    //Axis.dac_watch[5] = CTRL.I->idq[1]*0.05; // 20 A for high speed reversal
    Axis.dac_watch[6] = CTRL.I->rpm*0.002;
    Axis.dac_watch[7] = ELECTRICAL_SPEED_FEEDBACK*ELEC_RAD_PER_SEC_2_RPM*0.002;

    Axis.dac_watch[8] = EQep1Regs.QPOSCNT*0.0001; // [0, 10000]

    //Axis.dac_watch[2] = CTRL.I->iab[1]*0.2;
    Axis.dac_watch[9] = CTRL.I->iab[0]*0.2;

    //Axis.dac_watch[11] = COMM.current_sum*0.05/(float32)COMM.counterSS;
    //Axis.dac_watch[12] = COMM.voltage_sum*0.05/(float32)COMM.counterSS;
    //Axis.dac_watch[13] = COMM.counterSS*0.001;

    Axis.dac_watch[10] = AFE_USED.psi_2[0];
    Axis.dac_watch[11] = AFE_USED.psi_2[1]; // MOTOR.KActive; //AFEOE.psi_2[1];
    Axis.dac_watch[12] = AFEOE.psi_2[0];
    Axis.dac_watch[13] = AFEOE.psi_2[1];

    Axis.dac_watch[14] = CTRL.O->uab_cmd_to_inverter[0]*0.01;
    Axis.dac_watch[15] = CTRL.O->uab_cmd_to_inverter[1]*0.01;
    Axis.dac_watch[16] = CTRL.O->uab_cmd[0]*0.01;
    Axis.dac_watch[17] = CTRL.O->uab_cmd[1]*0.01;

    Axis.dac_watch[18] = CTRL.O->udq_cmd[0]*0.01;
    Axis.dac_watch[19] = CTRL.O->udq_cmd[1]*0.01;

    Axis.dac_watch[20] = ENC.theta_d_elec*0.1;
    Axis.dac_watch[21] = ELECTRICAL_POSITION_FEEDBACK*0.1;

    #if SELECT_ALGORITHM == ALG_NSOAF
        Axis.dac_watch[22] = nsoaf.LoadTorquePI*0.25;
        Axis.dac_watch[23] = nsoaf.xIq*0.2; // 5 A for low speed
        //Axis.dac_watch[23] = nsoaf.xIq*0.05; // 20 A for high speed reversal
    #endif

    //Axis.dac_watch[26] = CTRL.I->rpm*0.0005;
    //Axis.dac_watch[27] = ELECTRICAL_SPEED_FEEDBACK*ELEC_RAD_PER_SEC_2_RPM*0.0005;
    Axis.dac_watch[26] = CTRL.I->rpm*0.02;
    Axis.dac_watch[27] = ELECTRICAL_SPEED_FEEDBACK*ELEC_RAD_PER_SEC_2_RPM*0.02;

    Axis.dac_watch[30] = htz.psi_2_ampl;
    Axis.dac_watch[31] = htz.psi_2_ampl_lpf;
    Axis.dac_watch[32] = (MOTOR.KActive - htz.psi_2_ampl_lpf)*10;
    Axis.dac_watch[33] = htz.psi_2[0];
    Axis.dac_watch[34] = htz.psi_2[1];
    Axis.dac_watch[35] = htz.theta_d*0.1;

    Axis.dac_watch[36] = INV.sig_a2*0.02;
    Axis.dac_watch[37] = INV.sig_a3*0.02;

    Axis.dac_watch[38] = htz.u_offset[0]*0.2;
    Axis.dac_watch[39] = htz.u_offset[1]*0.2;


    #if FALSE
        REAL ual_dist = MT2A(pid1_iM.Out, pid1_iT.Out, CTRL.S->cosT, CTRL.S->sinT);
        REAL ube_dist = MT2B(pid1_iM.Out, pid1_iT.Out, CTRL.S->cosT, CTRL.S->sinT);
        Axis.dac_watch[28] = pid1_iM.Out*0.02;
        Axis.dac_watch[29] = pid1_iT.Out*0.02;

        Axis.dac_watch[30] = ual_dist*0.02;
        Axis.dac_watch[31] = ube_dist*0.02;

        Axis.dac_watch[32] = INV.ual_comp*0.02;
        Axis.dac_watch[33] = INV.ube_comp*0.02;

        Axis.dac_watch[34] = (CTRL.O->uab_cmd_to_inverter[0] - Axis.dac_watch_stator_resistance * CTRL.I->iab[0])*0.02;
        Axis.dac_watch[35] = (CTRL.O->uab_cmd_to_inverter[1] - Axis.dac_watch_stator_resistance * CTRL.I->iab[1])*0.02;

        Axis.dac_watch[36] = Axis.dac_watch[34] - Axis.dac_watch[32];
        Axis.dac_watch[37] = Axis.dac_watch[35] - Axis.dac_watch[33];

        Axis.dac_watch[38] = CTRL.inv->theta_trapezoidal / M_PI_OVER_180 * 0.025;
        Axis.dac_watch[39] = CTRL.inv->Vsat* 0.05;
        //Axis.dac_watch[39] = INV.I5_plus_I7_LPF * 0.2;
        Axis.dac_watch[40] = (CTRL.O->udq_cmd_to_inverter[0] - CTRL.O->udq_cmd[0])*0.02;
        Axis.dac_watch[41] = (CTRL.O->udq_cmd_to_inverter[1] - CTRL.O->udq_cmd[1])*0.02;
        Axis.dac_watch[42] = (CAP.dq[0]             - CTRL.O->udq_cmd[0])*0.02;
        Axis.dac_watch[43] = (CAP.dq[1]             - CTRL.O->udq_cmd[1])*0.02;
        //        Axis.dac_watch[38] = (CAP.ecapU.Period1) * 0.25e-4;
        //        Axis.dac_watch[39] = (CAP.ecapV.Period1) * 0.25e-4;
        //        Axis.dac_watch[40] = (CAP.ecapW.Period1) * 0.25e-4;
        //        Axis.dac_watch[41] = (CAP.ecapU.Period2) * 0.25e-4;
        //        Axis.dac_watch[42] = (CAP.ecapV.Period2) * 0.25e-4;
        //        Axis.dac_watch[43] = (CAP.ecapW.Period2) * 0.25e-4;

        Axis.dac_watch[44] = (CTRL.O->udq_cmd_to_inverter[0])*0.02;
        Axis.dac_watch[45] = (CTRL.O->udq_cmd_to_inverter[1])*0.02;
        Axis.dac_watch[46] = (CAP.dq[0]            )*0.02;
        Axis.dac_watch[47] = (CAP.dq[1]            )*0.02;
    #else

        Axis.dac_watch[40] = Axis.vdc*0.0025;

        Axis.dac_watch[41] = INV.ual_comp*0.05;
        Axis.dac_watch[42] = INV.ube_comp*0.05;

        Axis.dac_watch[43] = CTRL.inv->theta_trapezoidal / M_PI_OVER_180 * 0.025;

        Axis.dac_watch[47] = esoaf.xPos*0.1;
        Axis.dac_watch[48] = angle_error_limiter(ELECTRICAL_POSITION_FEEDBACK - esoaf.xPos);
        Axis.dac_watch[49] = angle_error_limiter(ENC.theta_d_elec - esoaf.xPos);

    #endif

    #if SELECT_ALGORITHM == ALG_Chi_Xu
    // Chi Xu 2009
    Axis.dac_watch[49] = sin(ENC.theta_d_elec - chixu.xTheta_d);
    Axis.dac_watch[50] = chixu.xIab[0]*0.2;
    Axis.dac_watch[51] = chixu.output_error[0]*0.2;
    Axis.dac_watch[52] = chixu.xZeq[0]*0.1;
    Axis.dac_watch[53] = chixu.xZeq[1]*0.1;
    Axis.dac_watch[54] = chixu.xEmf_raw[0]*0.01;
    Axis.dac_watch[55] = chixu.xEmf_raw[1]*0.01;
    #endif

    // HU WU 1998
    //    Axis.dac_watch[50] = sin(CTRL.I->theta_d_elec-huwu.theta_d);
    //    Axis.dac_watch[51] = huwu.x[0];
    //    Axis.dac_watch[52] = huwu.x[1];
    //    Axis.dac_watch[53] = huwu.x[2];
    //    Axis.dac_watch[54] = huwu.limiter_KE;

//    Axis.dac_watch[55] = MOTOR.KE;
//
//    //        Axis.dac_watch[48] = (CAP.ecapU.DutyOnTime1)  * 0.25e-4;
//    //        Axis.dac_watch[49] = (CAP.ecapU.DutyOffTime1) * 0.25e-4;
//    //        Axis.dac_watch[50] = (CAP.ecapU.DutyOnTime2)  * 0.25e-4;
//    //        Axis.dac_watch[51] = (CAP.ecapU.DutyOffTime2) * 0.25e-4;
//
//    Axis.dac_watch[56] = (CTRL.I->rpm - CTRL.I->cmd_speed_rpm)*0.01;
//    Axis.dac_watch[57] = (ELECTRICAL_SPEED_FEEDBACK*ELEC_RAD_PER_SEC_2_RPM - CTRL.I->cmd_speed_rpm)*0.01;
//
//    Axis.dac_watch[58] = angle_error_limiter(ENC.theta_d_elec - AFEOE.theta_d); // VM CM Fusion
//    Axis.dac_watch[59] = angle_error_limiter(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK);
//    //    Axis.dac_watch[58] = sin(ENC.theta_d_elec - AFEOE.theta_d);
//    //    Axis.dac_watch[59] = sin(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK);


    // information from d-axis voltage equation
    //temp_xOmg = (CTRL.O->udq_cmd[0] - CTRL.motor->R * CTRL.I->idq[0] ) / -CTRL.motor->Lq*CTRL.I->idq[1];
    //temp_xOmg_filtered = _lpf(temp_xOmg, temp_xOmg_filtered, 20.0);

    Axis.dac_watch[25] = hall_sensor_read[2] * 0.0005;


//    Axis.dac_watch[50] = hall_sensor_read[0] * 0.0005;
//    Axis.dac_watch[51] = hall_sensor_read[1] * 0.0005;
//    Axis.dac_watch[52] = hall_theta_r_elec_local_absolute[0] * 0.1;
//    Axis.dac_watch[53] = hall_rotating_direction;
//    Axis.dac_watch[54] = 1.0/normalizer[0]*0.0005;
//    Axis.dac_watch[55] = hall_qep_count*0.01;
    Axis.dac_watch[58] = hall_theta_r_elec[0] * 0.1;
    Axis.dac_watch[59] = hall_theta_r_elec_incremental[0] * 0.1;

//    Axis.dac_watch[56] = CTRL.I->theta_d_elec*0.1;
//    Axis.dac_watch[57] = CTRL.I->rpm*0.01;

    //Axis.dac_watch[50] = used_theta_d_elec * 0.1;
//    Axis.dac_watch[50] = Axis.angle_shift_for_second_inverter*0.1;
//    Axis.dac_watch[51] = Axis.iuvw[0] * 0.02;

    Axis.dac_watch[50] = pid1_dispX.out*0.02;
    Axis.dac_watch[51] = pid1_dispY.out*0.02;


//    Axis.dac_watch[52] = pid1_dispX.prevError * 0.001;
//    Axis.dac_watch[53] = pid1_dispY.prevError * 0.001;

//    Axis.dac_watch[52] = pid2_ix.Ref * 0.02;
//    Axis.dac_watch[53] = pid2_ix.Fbk * 0.02;
    Axis.dac_watch[52] = pid2_ix.Fbk * 0.02;
    Axis.dac_watch[53] = pid2_iy.Fbk * 0.02;

//    Axis.dac_watch[54] = eddy_displacement[0]*0.0004;
//    Axis.dac_watch[55] = eddy_displacement[1]*0.0004;
    Axis.dac_watch[54] = pid1_dispX.setpoint*0.001;
    Axis.dac_watch[55] = pid1_dispY.setpoint*0.001;
    Axis.dac_watch[56] = pid1_dispX.measurement*0.001;
    Axis.dac_watch[57] = pid1_dispY.measurement*0.001;
    Axis.dac_watch[58] = pid1_dispX.prevError*0.001;
    Axis.dac_watch[59] = pid1_dispY.prevError*0.001;



//    Axis.dac_watch[58] = current_pole[0]*0.5;
//    Axis.dac_watch[59] = current_pole[1]*0.5;


    if(Axis.channels_preset==101){Axis.channels_preset=0;
        /* Slice motor + sensorless */
        Axis.channels[0] = 27; // ESOAF theta_d
        Axis.channels[1] = 21; // AFEOE theta_d
        Axis.channels[2] = 58; // Hall sensor theta_d
        Axis.channels[3] = 10; // AFEOE psi_2[0]
        Axis.channels[4] = 26;
        Axis.channels[5] = 27;
        Axis.channels[6] = 50;
    }else if(Axis.channels_preset==102){Axis.channels_preset=0;
        /* Slice motor + sensorless */
        Axis.channels[0] = 26;
        Axis.channels[1] = 27;
        Axis.channels[2] = 10;
        Axis.channels[3] = 33;
        Axis.channels[4] = 50;
        Axis.channels[5] = 51;
        Axis.channels[6] = 52;
    }else if(Axis.channels_preset==1){Axis.channels_preset=0;
        /*Sensorless using ECAP*/
        Axis.channels[0] = 46;
        Axis.channels[1] = 47;
        Axis.channels[2] = 20;
        Axis.channels[3] = 21;
        Axis.channels[4] =  6;
        Axis.channels[5] =  7;
        Axis.channels[6] = 59;
    }else if(Axis.channels_preset==9){Axis.channels_preset=0;
        /* Slice motor Y */
        Axis.channels[0] = 53;
        Axis.channels[1] = 55;
        Axis.channels[2] = 57;
        Axis.channels[3] = 51;
        Axis.channels[4] = 50;
        Axis.channels[5] = 51;
    }else if(Axis.channels_preset==91){Axis.channels_preset=0;
        /* Slice motor X */
        Axis.channels[0] = 52;
        Axis.channels[1] = 54;
        Axis.channels[2] = 56;
        Axis.channels[3] = 50;
        Axis.channels[4] = 50;
        Axis.channels[5] = 51;
    }else if(Axis.channels_preset==2){Axis.channels_preset=0;
        /*ECAP Periods*/
        Axis.channels[0] = 38;
        Axis.channels[1] = 39;
        Axis.channels[2] = 40;
        Axis.channels[3] = 41;
        Axis.channels[4] = 42;
        Axis.channels[5] = 43;
    }else if(Axis.channels_preset==3){Axis.channels_preset=0;
        /*ECAP U phase on/off 1/2*/
        Axis.channels[0] = 48;
        Axis.channels[1] = 49;
        Axis.channels[2] = 50;
        Axis.channels[3] = 51;
        Axis.channels[4] = 46;
        Axis.channels[5] = 47;
    }else if(Axis.channels_preset==4){Axis.channels_preset=0;
        /* Test Holtz03 for online invnonl id. */
        Axis.channels[0] = 30;
        Axis.channels[1] = 31;
        Axis.channels[2] = 32;
        Axis.channels[3] = 33;
        Axis.channels[4] = 34;
        Axis.channels[5] = 59; //35;
        Axis.channels[7] = 55;
    }else if(Axis.channels_preset==5){Axis.channels_preset=0;
        /* Test Holtz03 for online invnonl id. */
        Axis.channels[0] = 36; // sig_a2
        Axis.channels[1] = 37; // sig_a3
        Axis.channels[2] = 32; // ampl error
        Axis.channels[3] = 59; // angle error
        Axis.channels[4] = 33; // htz.psi_2[0]
        Axis.channels[5] = 12; // AFEOE.psi_2[0]
        Axis.channels[6] = 38; // htz.u_offset[0]
        Axis.channels[7] = 39; // htz.u_offset[1]
    }else if(Axis.channels_preset==6){Axis.channels_preset=0;
        /* Test Holtz03 for online invnonl id. */
        Axis.channels[0] = 36; // sig_a2
        Axis.channels[1] = 37; // sig_a3
        Axis.channels[2] = 59; // angle error
        Axis.channels[3] = 40; // Vdc
        //Axis.channels[3] = 5; // iq
        //Axis.channels[4] = 30; // htz.psi_2_ampl
        //Axis.channels[5] = 31; // htz.psi_2_ampl_lpf
        //Axis.channels[4] = 33; // htz.psi_2[0]
        //Axis.channels[5] = 12; // AFEOE.psi_2[0]
        Axis.channels[4] = 41; // Compensation Voltage alpha
        Axis.channels[5] = 42; // Compensation Voltage beta
        Axis.channels[6] = 26; // Speed
        Axis.channels[7] = 27; // Speed Estimate
        //Axis.channels[6] = 38; // htz.u_offset[0]
        //Axis.channels[7] = 39; // htz.u_offset[1]
    }else if(Axis.channels_preset==7){Axis.channels_preset=0;
        /* TIE.R1 Slessinv SS */
        Axis.channels[0] = 3; // iq cmd
        Axis.channels[1] = 5; // iq
        Axis.channels[2] = 59; // angle error
        Axis.channels[3] = 39; // u offset beta
        Axis.channels[4] = 38; // psi_2 alpha
        Axis.channels[5] = 1; // dc bus
        Axis.channels[6] = 26; // Speed
        Axis.channels[7] = 27; // Speed Estimate
    }else if(Axis.channels_preset==8){Axis.channels_preset=0;
        /* TIE.R1 Slessinv SS */
        Axis.channels[0] = 59; // angle error
        Axis.channels[1] = 5;  // iq
        Axis.channels[2] = 20; // theta_d
        Axis.channels[3] = 21; // hat theta_d
        Axis.channels[4] = 36; //a2 //41; // ual_comp
        Axis.channels[5] = 37; //a3 //42; // ube_comp
        Axis.channels[6] = 26; // Speed
        //Axis.channels[7] = 27; // Speed Estimate
        Axis.channels[7] = 43; // theta_trapezoidal

    }

    // 八通道DAC输出，请修改Axis.channels数组来确定具体输出哪些Axis.dac_watch数组中的变量。
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


