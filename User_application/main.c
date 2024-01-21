#include <All_Definition.h>
st_axis Axis;
//int digital_virtual_button = 0;
REAL target_position_cnt = 5000;
        REAL KP = 0.05;
        REAL error_pos;

#ifdef _MMDv1 // mmlab drive version 1
    #define OFFSET_VDC 7   // ADC0
//    #define OFFSET_U   2023 // 2
//    #define OFFSET_V   2060 // 3
//    #define OFFSET_W   2020 // 1
//Lem
     #define OFFSET_U   2040 // 2
    #define OFFSET_V   2070 // 3
    #define OFFSET_W   2075 // 1
    #define OFFSET_R   2058 // 11
    #define OFFSET_S   2062 // 9
    #define OFFSET_T   2059 // 8

//Lem 1
    #define OFFSET_VDC_BUS_IPM1 -1.01456189
    #define OFFSET_LEM1_L   2024 //2023.89473684 // ADCB7
    #define OFFSET_LEM1_R   2041 //2042.33333333 // ADCB8
    #define OFFSET_LEM1_M   2044 //2043.43859649 // ADCB9


    #define OFFSET_LEM2_L   2024 //2023.89473684 // ADCB7
    #define OFFSET_LEM2_R   2041 //2042.33333333 // ADCB8
    #define OFFSET_LEM2_M   2044 //2043.43859649 // ADCB9

    // Sensor Board 6 Phase SiC MOSFET Inverter
    #define SCALE_VDC 0.0949
    #define SCALE_U  0.0325 // 0.03367
    #define SCALE_V  0.0320 // 0.03388
    #define SCALE_W  0.0320 // 0.0340136
    #define SCALE_R -0.0295 //-0.032
    #define SCALE_S -0.0295 //-0.032
    #define SCALE_T -0.0290 //-0.034


//Lem 1
// 令逆变器输出端指向电机为正方向，若LEM上的箭头与正方向相同，则SCALE为正数，若LEM上的箭头与正方向相反，则SCALE为负数，
    #define SCALE_VDC_BUS_IPM1 0.17604031
    #define SCALE_LEM1_L   -0.03076297 // ADCB7
    #define SCALE_LEM1_R   -0.03038256 // ADCB8
    #define SCALE_LEM1_M   -0.03039058 // ADCB9

    #define SCALE_LEM2_L   0.03076297 // ADCB7
    #define SCALE_LEM2_R   0.03038256 // ADCB8
    #define SCALE_LEM2_M   0.03039058 // ADCB9

#else
    scale and offset...
#endif

#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19
#define ANGLE_SHIFT_FOR_FIRST_INVERTER  0.0 // Torque Inverter
#define ANGLE_SHIFT_FOR_SECOND_INVERTER 0.0 // Suspension Inverter
int USE_3_CURRENT_SENSORS = TRUE;

void init_experiment_AD_gain_and_offset(){
    /* ADC OFFSET */
    // dc bus sensor
    Axis.adc_offset[0] = OFFSET_VDC;
    // LEM2
    Axis.adc_offset[1] = OFFSET_U; // 2
    Axis.adc_offset[2] = OFFSET_V; // 3
    Axis.adc_offset[3] = OFFSET_W; // 1
    // LEM1
    Axis.adc_offset[0] = OFFSET_VDC_BUS_IPM1;
    Axis.adc_offset[4] = OFFSET_LEM1_L; // b7
    Axis.adc_offset[5] = OFFSET_LEM1_R; // b8
    Axis.adc_offset[6] = OFFSET_LEM1_M; // b9

    Axis.adc_offset[1] = OFFSET_LEM2_L; // b7
    Axis.adc_offset[2] = OFFSET_LEM2_R; // b8
    Axis.adc_offset[3] = OFFSET_LEM2_M; // b9

//    Axis.adc_offset[1] = OFFSET_U; // 2
//    Axis.adc_offset[2] = OFFSET_V; // 3
//    Axis.adc_offset[3] = OFFSET_W; // 1
//    Axis.adc_offset[4] = OFFSET_R; // 11
//    Axis.adc_offset[5] = OFFSET_S; // 9
//    Axis.adc_offset[6] = OFFSET_T; // 7

    /* ADC SCALE */
    Axis.adc_scale[0] = SCALE_VDC;
    Axis.adc_scale[1] = SCALE_U;
    Axis.adc_scale[2] = SCALE_V;
    Axis.adc_scale[3] = SCALE_W;
    Axis.adc_scale[4] = SCALE_R;
    Axis.adc_scale[5] = SCALE_S;
    Axis.adc_scale[6] = SCALE_T;

    // LEM1
    Axis.adc_scale[0] = SCALE_VDC_BUS_IPM1;
    Axis.adc_scale[4] = SCALE_LEM1_L;
    Axis.adc_scale[5] = SCALE_LEM1_R;
    Axis.adc_scale[6] = SCALE_LEM1_M;

    Axis.adc_scale[1] = SCALE_LEM2_L;
    Axis.adc_scale[2] = SCALE_LEM2_R;
    Axis.adc_scale[3] = SCALE_LEM2_M;

    /* eQEP OFFSET */
    CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
    CTRL.enc->theta_d_offset = CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis * CNT_2_ELEC_RAD;
}
void main(void){

    Axis.pCTRL = &CTRL;
    Axis.pAdcaResultRegs = &AdcaResultRegs;
    Axis.pAdcbResultRegs = &AdcbResultRegs;
    Axis.pAdccResultRegs = &AdccResultRegs;
    Axis.use_first_set_three_phase = 1; // -1;
    Axis.Set_current_loop = FALSE;
    Axis.Set_x_suspension_current_loop = FALSE;
    Axis.Set_y_suspension_current_loop = FALSE;
    Axis.Set_manual_rpm = 50.0;
    Axis.Set_manual_current_iq = 0.0;
    Axis.Set_manual_current_id = 0.0; // 20
    Axis.Select_exp_operation = 0; //200; //202; //200; //101;
    Axis.pFLAG_INVERTER_NONLINEARITY_COMPENSATION = &G.FLAG_INVERTER_NONLINEARITY_COMPENSATION;
    Axis.flag_overwrite_theta_d = FALSE;
    Axis.Overwrite_Current_Frequency = 0;
    Axis.Overwrite_Suspension_Current_Frequency = 0.5;
    Axis.used_theta_d_elec = 0.0;
    Axis.angle_shift_for_first_inverter  = ANGLE_SHIFT_FOR_FIRST_INVERTER;
    Axis.angle_shift_for_second_inverter = ANGLE_SHIFT_FOR_SECOND_INVERTER;
    Axis.OverwriteSpeedOutLimitDuringInit = 6; // 30; // A
    Axis.FLAG_ENABLE_PWM_OUTPUT = FALSE;
    Axis.channels_preset = 1; // 9; // 101;

    InitSysCtrl();        // 1. Initialize System Control: PLL, WatchDog, enable Peripheral Clocks.
    Gpio_initialize();    // 2. Initialize GPIO and assign GPIO to peripherals.
    DINT;                 // 3.1 Clear all interrupts and initialize PIE vector table.
    InitPieCtrl();        // 3.2 Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;         // 3.3 Disable CPU __interrupts,
    IFR = 0x0000;         // 3.4 and clear all CPU __interrupt flags.
    InitPieVectTable();   // 3.5 Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). At end, ENPIE = 1.

    // for Slessinv TIE.R1 for measuring the execution time
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 200, 1000000); // 200MHz, INTERRUPT_period = 1e6 us

    // 4.1 IPC
    #if NUMBER_OF_DSP_CORES == 2
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #endif
    #ifdef _STANDALONE
    #ifdef _FLASH
        // 当你需要离线断电再上电运行时用这个：
        //  Send boot command to allow the CPU02 application to begin execution
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #else
        //  Send boot command to allow the CPU02 application to begin execution
        // 这句话我不知道什么意义，可能还是不要比较好。
        //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
    #endif
    #endif

    // 4.2 Initialize peripherals
    ePWM_initialize();
    ADC_initialize();
    eQEP_initialize(0);
    InitECaptureContinuousMode();

    // 4.3 Assign peripherals to CPU02
    /* SPI and SCI */
    #if NUMBER_OF_DSP_CORES == 1
        // 同步！！！！！
        InitHighSpeedSpiGpio();
        //InitSpiaGpio();
        //InitSpicGpio();
        InitSpi();

        InitSciGpio();
        InitSci();
    #elif NUMBER_OF_DSP_CORES == 2
        /* 双核配置*/
        // 初始化SPI，用于与DAC芯片MAX5307通讯。
        EALLOW;
        DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
        DevCfgRegs.CPUSEL6.bit.SPI_C = 1; // assign spi-c to cpu2

        DevCfgRegs.CPUSEL5.bit.SCI_C = 1; // assign sci-c to cpu2
        EDIS;

        // 同步！！！！！
        InitHighSpeedSpiGpio();
        //InitSpicGpio();
        //InitSpiaGpio();
        //InitSpi(); // this is moved to CPU02


        // 异步！！！！！
        InitScicGpio();
        //InitSci(); // this is moved to CPU02

        // 在此之前，已经把GPIO和外设的权限转给CPU2了。
        // 这里再把部分共享内存权限给CPU2，同时告诉CPU2，你可以继续运行代码了。
        while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS0))
        {
            EALLOW;
            // Give Memory Access to GS0/ GS14 SARAM to CPU02
            MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
            EDIS;
        }
    #endif

    // 4.4 Initialize algorithms
    init_experiment();
    init_experiment_AD_gain_and_offset();

    // 5. Handle Interrupts
    /* Re-map PIE Vector Table to user defined ISR functions. */
        EALLOW; // This is needed to write to EALLOW protected registers
        PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM主中断 10kKHz
        #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
        PieVectTable.ECAP1_INT = &ecap1_isr;
        PieVectTable.ECAP2_INT = &ecap2_isr;
        PieVectTable.ECAP3_INT = &ecap3_isr;
        #endif
        #if SYSTEM_PROGRAM_MODE != 223
        PieVectTable.EQEP1_INT = &EQEP_UTO_INT;      // eQEP
        #endif
        #if NUMBER_OF_DSP_CORES == 1
        //PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;   // SCI Receive (Part 1/3)
        //PieVectTable.SCIC_TX_INT = &scicTxFifoIsr;   // SCI Transmit
        #endif
        EDIS; // This is needed to disable write to EALLOW protected registers
    /* PIE Control */
        /* ePWM */
        PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // PWM1 interrupt (Interrupt 3.1)
        /* SCI */
        #if NUMBER_OF_DSP_CORES == 1
            //PieCtrlRegs.PIEIER8.bit.INTx5 = 1;   // PIE Group 8, INT5, SCI-C Rx (Part 2/3)
            //PieCtrlRegs.PIEIER8.bit.INTx6 = 1;   // PIE Group 8, INT6, SCI-C Tx
        #endif
        /* eCAP */
        #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
            PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
            PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // 1 Enable for Interrupt 4.2
            PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // 2 Enable for Interrupt 4.3
        #endif
        /* eQEP */
        #if SYSTEM_PROGRAM_MODE != 223
            PieCtrlRegs.PIEIER5.bit.INTx1 = 1;      // QEP interrupt
        #endif
    /* CPU Interrupt Enable Register (IER) */
        IER |= M_INT3;  // EPWM1_INT
        #if NUMBER_OF_DSP_CORES == 1
            //IER |= M_INT8; // SCI-C (Part 3/3)
        #endif
        #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
            IER |= M_INT4;  // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
        #endif
        #if SYSTEM_PROGRAM_MODE != 223 // used by yuanxin
        IER |= M_INT5;  // EQEP1_INT???
        #endif
    EINT;   // Enable Global __interrupt INTM
    ERTM;   // Enable Global realtime __interrupt DBGM

    // 6. Pre main loop
    DSP_STOP_LED1
    DSP_STOP_LED2
    DSP_PWM_DISABLE
    DSP_2PWM_DISABLE
    /* Test IPC to CPU02 */
    #if NUMBER_OF_DSP_CORES == 2
        Write.dac_buffer[0] = 0.5;
        Write.dac_buffer[1] = 0.5;
        Write.dac_buffer[2] = 0.5;
        Write.dac_buffer[3] = 0.5;
        Write.dac_buffer[4] = 0.5;
        Write.dac_buffer[5] = 0.5;
        Write.dac_buffer[6] = 0.5;
        Write.dac_buffer[7] = 0.5;
        Write.test = 200.0;
        IPCLtoRFlagSet(IPC_FLAG7);
    #endif

    if(FALSE){
        SendChar = 24;
        for(;;)
        {
            //SendChar = 0;
            ScicRegs.SCITXBUF.all = (SendChar);

            //
            // wait for RRDY/RXFFST =1 for 1 data available in FIFO
            //
            while(ScicRegs.SCIFFRX.bit.RXFFST == 0) {
            }
            CTRL.S->go_sensorless = 100;

            //
            // Check received character
            //
            ReceivedChar = ScicRegs.SCIRXBUF.all;
            if(ReceivedChar != SendChar)
            {
               //error();
            }

            //
            // Move to the next character and repeat the test
            //
            SendChar++;

            //
            // Limit the character to 8-bits
            //
            SendChar &= 0x00F;
            LoopCount++;
        }
    }

    // 7. Main loop
    while(1){

        if (Motor_mode_START==1){
            Axis.FLAG_ENABLE_PWM_OUTPUT = 1;
            DSP_START_LED1
            DSP_START_LED2
        }else if (Motor_mode_START==0){
            Axis.FLAG_ENABLE_PWM_OUTPUT = 0;
            DSP_STOP_LED1
            DSP_STOP_LED2
        }

        #if NUMBER_OF_DSP_CORES == 2
        /* CPU02 (Remote) to CPU01 (Local)
         * The register to check is IPCSTS.
         * */
            if(IPCRtoLFlagBusy(IPC_FLAG10) == 1) // if flag
            {
                sci_poll(Read.SCI_char);
                IPCRtoLFlagAcknowledge (IPC_FLAG10);
            }
        #endif
        #if NUMBER_OF_DSP_CORES == 1
            single_core_dac();
        #endif
    }
}

/* Below is moved from PanGuMainISR.c */
#if SYSTEM_PROGRAM_MODE==223
void DeadtimeCompensation(REAL Current_U, REAL Current_V, REAL Current_W, REAL CMPA[], REAL CMPA_DBC[]){

    // TODO: Add transitional linear range to dead time compensation

    int temp = 0;
    // ------------U--------------
    if (Current_U>=0){
        temp = (int)(CMPA[0] + SYSTEM_PWM_DEADTIME_COMPENSATION);
        if(temp>=SYSTEM_TBPRD)
            temp =  (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }else{
        temp = (int)(CMPA[0] - SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp<=0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[0] = (Uint16)temp;
    temp = 0;

    // --------------V--------------
    if (Current_V>=0)
    {
        temp = (int)(CMPA[1] + SYSTEM_PWM_DEADTIME_COMPENSATION);
        if(temp>=SYSTEM_TBPRD)
            temp =  (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    else
    {
        temp = (int)(CMPA[1] - SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp<=0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[1] = (Uint16)temp;
    temp = 0;

    // --------------W--------------
    if (Current_W>=0)
    {
        temp = (int)(CMPA[2] + SYSTEM_PWM_DEADTIME_COMPENSATION);
        if(temp>=SYSTEM_TBPRD)
            temp =  (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    else
    {
        temp = (int)(CMPA[2] - SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp<=0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[2] = (Uint16)temp;
}

REAL vvvf_voltage = 4;
REAL vvvf_frequency = 5;
REAL enable_vvvf = TRUE;


void voltage_commands_to_pwm(){
    if(Axis.use_first_set_three_phase==1){
        // SVPWM of the motor 3-phase
        CTRL.svgen1.Ualpha= CTRL.O->uab_cmd_to_inverter[0];
        CTRL.svgen1.Ubeta = CTRL.O->uab_cmd_to_inverter[1];

//        CTRL.svgen1.Ualpha= vvvf_voltage * cos(vvvf_frequency*2*M_PI*CTRL.timebase);
//        CTRL.svgen1.Ubeta = vvvf_voltage * sin(vvvf_frequency*2*M_PI*CTRL.timebase);

        // SVPWM of the suspension 3-phase
        CTRL.svgen2.Ualpha = 0;
        CTRL.svgen2.Ubeta  = 0;
        if(enable_vvvf){
            CTRL.svgen2.Ualpha= vvvf_voltage * cos(vvvf_frequency*2*M_PI*CTRL.timebase);
            CTRL.svgen2.Ubeta = vvvf_voltage * sin(vvvf_frequency*2*M_PI*CTRL.timebase);
        }

        //    svgen2.Ualpha = svgen1.Ualpha*0.5        + svgen1.Ubeta*0.8660254; // rotate 60 deg
        //    svgen2.Ubeta  = svgen1.Ualpha*-0.8660254 + svgen1.Ubeta*0.5;
    }else if(Axis.use_first_set_three_phase==2){
        // SVPWM of the motor 3-phase
        CTRL.svgen1.Ualpha= 0;
        CTRL.svgen1.Ubeta = 0;

        // SVPWM of the suspension 3-phase
        //CTRL.svgen2.Ualpha = CTRL.O->uab_cmd_to_inverter[0+2];
        //CTRL.svgen2.Ubeta  = CTRL.O->uab_cmd_to_inverter[1+2]; // uab_cmd的第零个和第一个分量传给svgen2第二套逆变器！！！
        CTRL.svgen2.Ualpha = CTRL.O->uab_cmd_to_inverter[0];
        CTRL.svgen2.Ubeta  = CTRL.O->uab_cmd_to_inverter[1];

        if(enable_vvvf){
            CTRL.svgen2.Ualpha= vvvf_voltage * cos(vvvf_frequency*2*M_PI*CTRL.timebase);
            CTRL.svgen2.Ubeta = vvvf_voltage * sin(vvvf_frequency*2*M_PI*CTRL.timebase);
        }

    }else if(Axis.use_first_set_three_phase==-1){

        // SVPWM of the motor 3-phase 第二套逆变器控制转矩
        CTRL.svgen2.Ualpha = CTRL.O->uab_cmd_to_inverter[0]; // uab_cmd的第零个和第一个分量传给svgen2第二套逆变器！！！
        CTRL.svgen2.Ubeta  = CTRL.O->uab_cmd_to_inverter[1];

        // SVPWM of the suspension 3-phase
        CTRL.svgen1.Ualpha = CTRL.O->uab_cmd_to_inverter[0+2];
        CTRL.svgen1.Ubeta  = CTRL.O->uab_cmd_to_inverter[1+2];
    }
    SVGEN_Drive(&CTRL.svgen1);
    SVGEN_Drive(&CTRL.svgen2); //, -ctrl.UNot);

    CTRL.svgen1.CMPA[0] = CTRL.svgen1.Ta*SYSTEM_TBPRD;
    CTRL.svgen1.CMPA[1] = CTRL.svgen1.Tb*SYSTEM_TBPRD;
    CTRL.svgen1.CMPA[2] = CTRL.svgen1.Tc*SYSTEM_TBPRD;
    CTRL.svgen2.CMPA[0] = CTRL.svgen2.Ta*SYSTEM_TBPRD;
    CTRL.svgen2.CMPA[1] = CTRL.svgen2.Tb*SYSTEM_TBPRD;
    CTRL.svgen2.CMPA[2] = CTRL.svgen2.Tc*SYSTEM_TBPRD;

    #if USE_DEATIME_PRECOMP
        DeadtimeCompensation(Axis.iuvw[0], Axis.iuvw[1], Axis.iuvw[2], CTRL.svgen1.CMPA, CTRL.svgen1.CMPA_DBC);
        EPwm1Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA_DBC[0];
        EPwm2Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA_DBC[1];
        EPwm3Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA_DBC[2];
        DeadtimeCompensation(Axis.iuvw[3], Axis.iuvw[4], Axis.iuvw[5], CTRL.svgen2.CMPA, CTRL.svgen2.CMPA_DBC);
        EPwm4Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA_DBC[0];
        EPwm5Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA_DBC[1];
        EPwm6Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA_DBC[2];

    #else
        EPwm1Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA[0];
        EPwm2Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA[1];
        EPwm3Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen1.CMPA[2];
        EPwm4Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA[0];
        EPwm5Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA[1];
        EPwm6Regs.CMPA.bit.CMPA = (Uint16)CTRL.svgen2.CMPA[2];
    #endif
}
void voltage_measurement_based_on_eCAP(){
    CAP.terminal_voltage[0] = (CAP.terminal_DutyOnRatio[0]) * Axis.vdc - Axis.vdc * 0.5; // -0.5 is due to duty ratio calculation; - vdc * 0.5 is referring to the center of dc bus capacitor.
    CAP.terminal_voltage[1] = (CAP.terminal_DutyOnRatio[1]) * Axis.vdc - Axis.vdc * 0.5;
    CAP.terminal_voltage[2] = (CAP.terminal_DutyOnRatio[2]) * Axis.vdc - Axis.vdc * 0.5;

    CAP.line_to_line_voltage[0] = CAP.terminal_voltage[0] - CAP.terminal_voltage[1];
    CAP.line_to_line_voltage[1] = CAP.terminal_voltage[1] - CAP.terminal_voltage[2];
    CAP.line_to_line_voltage[2] = CAP.terminal_voltage[2] - CAP.terminal_voltage[0];

    if(CAP.flag_bad_U_capture==FALSE && CAP.flag_bad_V_capture==FALSE && CAP.flag_bad_W_capture==FALSE){
        // Use ecap feedback
        CAP.uab0[0] = 0.33333 * (2*CAP.terminal_voltage[0] - CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[1] = 0.57735 * (                            CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[2] = 0.33333 * (  CAP.terminal_voltage[0] + CAP.terminal_voltage[1] + CAP.terminal_voltage[2]);
        CAP.dq[0] =  CTRL.S->cosT*CAP.uab0[0] + CTRL.S->sinT*CAP.uab0[1];
        CAP.dq[1] = -CTRL.S->sinT*CAP.uab0[0] + CTRL.S->cosT*CAP.uab0[1];

    }else{
        // Assume the voltage vector is rtoating at a constant speed when ecap measurement is disturbed.
        CAP.uab0[0] = CTRL.S->cosT*CAP.dq[0] - CTRL.S->sinT*CAP.dq[1];
        CAP.uab0[1] = CTRL.S->sinT*CAP.dq[0] + CTRL.S->cosT*CAP.dq[1];
    }

    // 电压测量
    if(G.flag_use_ecap_voltage==2 || G.flag_use_ecap_voltage==1){
        /*Use original ecap measured voltage*/
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];
    }
    //    else if(G.flag_use_ecap_voltage==3){
    //        ecap_moving_average();
    //    }
    if(G.flag_use_ecap_voltage==10){
        /*Use lpf ecap measured voltage*/
        CAP.dq_lpf[0] = _lpf(CAP.dq[0], CAP.dq_lpf[0], 800);
        CAP.dq_lpf[1] = _lpf(CAP.dq[1], CAP.dq_lpf[1], 800);
        CAP.uab0[0] = CTRL.S->cosT*CAP.dq_lpf[0] - CTRL.S->sinT*CAP.dq_lpf[1];
        CAP.uab0[1] = CTRL.S->sinT*CAP.dq_lpf[0] + CTRL.S->cosT*CAP.dq_lpf[1];
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];

    }else if(G.flag_use_ecap_voltage==0){
        /*Use command voltage for feedback*/
        US_P(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
        US_P(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current
        US_C(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current
    }

    // (for watch only) Mismatch between ecap measurement and command to inverter
    CTRL.O->udq_cmd_to_inverter[0] = CTRL.S->cosT*CTRL.O->uab_cmd_to_inverter[0] + CTRL.S->sinT*CTRL.O->uab_cmd_to_inverter[1];
    CTRL.O->udq_cmd_to_inverter[1] =-CTRL.S->sinT*CTRL.O->uab_cmd_to_inverter[0] + CTRL.S->cosT*CTRL.O->uab_cmd_to_inverter[1];
    CAP.dq_mismatch[0] = CTRL.O->udq_cmd_to_inverter[0] - CAP.dq[0];
    CAP.dq_mismatch[1] = CTRL.O->udq_cmd_to_inverter[1] - CAP.dq[1];
}


void measurement(){

    // 转子位置和转速接口 以及 转子位置和转速测量
    Uint32 QPOSCNT   = EQep1Regs.QPOSCNT;
    ENC.rpm          = PostionSpeedMeasurement_MovingAvergage(QPOSCNT, CTRL.enc);

    if(CTRL.S->go_sensorless == FALSE){
        CTRL.I->omg_elec     = CTRL.enc->omg_elec;
        CTRL.I->theta_d_elec = CTRL.enc->theta_d_elec;
    }
    // Convert adc results
    //Axis.vdc    =((REAL)(AdcaResultRegs.ADCRESULT0 ) - Axis.adc_offset[0]) * Axis.adc_scale[0];
    Axis.vdc    =((REAL)(AdcbResultRegs.ADCRESULT6 ) - Axis.adc_offset[0]) * Axis.adc_scale[0];
    if(G.flag_overwite_vdc) Axis.vdc = G.overwrite_vdc;

//    Axis.iuvw[0]=((REAL)(AdcaResultRegs.ADCRESULT1 ) - Axis.adc_offset[1]) * Axis.adc_scale[1];
//    Axis.iuvw[1]=((REAL)(AdcaResultRegs.ADCRESULT2 ) - Axis.adc_offset[2]) * Axis.adc_scale[2];
//    Axis.iuvw[2]=((REAL)(AdcaResultRegs.ADCRESULT3 ) - Axis.adc_offset[3]) * Axis.adc_scale[3];
//    Axis.iuvw[3]=((REAL)(AdcbResultRegs.ADCRESULT11) - Axis.adc_offset[4]) * Axis.adc_scale[4]; // AD_scale_U2; offsetD2
//    Axis.iuvw[4]=((REAL)(AdcbResultRegs.ADCRESULT9 ) - Axis.adc_offset[5]) * Axis.adc_scale[5]; // AD_scale_V2; offsetB2
//    Axis.iuvw[5]=((REAL)(AdcbResultRegs.ADCRESULT8 ) - Axis.adc_offset[6]) * Axis.adc_scale[6]; // AD_scale_W2; offsetA2

    //LEM1传感器用于测量第一套绕组的电流，因此，adcb7\b8\b9的ad值给iuvw[0][1][2]
//    Axis.iuvw[0]=((REAL)(AdccResultRegs.ADCRESULT2 ) - Axis.adc_offset[1]) * Axis.adc_scale[1];
//    Axis.iuvw[1]=((REAL)(AdccResultRegs.ADCRESULT4 ) - Axis.adc_offset[2]) * Axis.adc_scale[2];
//    Axis.iuvw[2]=((REAL)(AdccResultRegs.ADCRESULT5 ) - Axis.adc_offset[3]) * Axis.adc_scale[3];
    Axis.iuvw[0]=((REAL)(AdcbResultRegs.ADCRESULT7 ) - Axis.adc_offset[4]) * Axis.adc_scale[4]; // AD_scale_U2; offsetD2
    Axis.iuvw[1]=((REAL)(AdcbResultRegs.ADCRESULT8 ) - Axis.adc_offset[5]) * Axis.adc_scale[5]; // AD_scale_V2; offsetB2
    Axis.iuvw[2]=((REAL)(AdcbResultRegs.ADCRESULT9 ) - Axis.adc_offset[6]) * Axis.adc_scale[6]; // AD_scale_W2; offsetA2
    Axis.iuvw[3]=((REAL)(AdcaResultRegs.ADCRESULT1 ) - Axis.adc_offset[1]) * Axis.adc_scale[1]; // AD_scale_U2; offsetD2
    Axis.iuvw[4]=((REAL)(AdcaResultRegs.ADCRESULT2 ) - Axis.adc_offset[2]) * Axis.adc_scale[2]; // AD_scale_V2; offsetB2
    Axis.iuvw[5]=((REAL)(AdcaResultRegs.ADCRESULT3 ) - Axis.adc_offset[3]) * Axis.adc_scale[3]; // AD_scale_W2; offsetA2

    // 线电压测量（基于占空比和母线电压）
    //voltage_measurement_based_on_eCAP();

    // Vdc用于实时更新电流环限幅
    pid1_iM.OutLimit = Axis.vdc * 0.5773672;
    pid1_iT.OutLimit = Axis.vdc * 0.5773672;
    pid2_ix.OutLimit = Axis.vdc * 0.5773672;
    pid2_iy.OutLimit = Axis.vdc * 0.5773672;

    // 电流接口
    if(USE_3_CURRENT_SENSORS){
        Axis.iabg[0] = UVW2A_AI(Axis.iuvw[0], Axis.iuvw[1], Axis.iuvw[2]);
        Axis.iabg[1] = UVW2B_AI(Axis.iuvw[0], Axis.iuvw[1], Axis.iuvw[2]);
        Axis.iabg[2] = UVW2G_AI(Axis.iuvw[0], Axis.iuvw[1], Axis.iuvw[2]);
        Axis.iabg[3] = UVW2A_AI(Axis.iuvw[3], Axis.iuvw[4], Axis.iuvw[5]);
        Axis.iabg[4] = UVW2B_AI(Axis.iuvw[3], Axis.iuvw[4], Axis.iuvw[5]);
        Axis.iabg[5] = UVW2G_AI(Axis.iuvw[3], Axis.iuvw[4], Axis.iuvw[5]);
    }else{
        REAL phase_V_current = -Axis.iuvw[0] - Axis.iuvw[2];
        Axis.iabg[0] = UV2A_AI(Axis.iuvw[0], phase_V_current);
        Axis.iabg[1] = UV2B_AI(Axis.iuvw[0], phase_V_current);
        phase_V_current = -Axis.iuvw[3] - Axis.iuvw[5];
        Axis.iabg[3] = UV2A_AI(Axis.iuvw[3], phase_V_current);
        Axis.iabg[4] = UV2B_AI(Axis.iuvw[3], phase_V_current);
    }

    if(Axis.use_first_set_three_phase==1){
        // 只用第一套三相
        IS_C(0)        = Axis.iabg[0];
        IS_C(1)        = Axis.iabg[1];
        CTRL.I->iab[0] = Axis.iabg[0];
        CTRL.I->iab[1] = Axis.iabg[1];

        US_C(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current

        US_P(0) = US_C(0);
        US_P(1) = US_C(1);

    }else if(Axis.use_first_set_three_phase==2){
        // 只用第二套三相
        IS_C(0)        = Axis.iabg[3];
        IS_C(1)        = Axis.iabg[4];
        CTRL.I->iab[0] = Axis.iabg[3];
        CTRL.I->iab[1] = Axis.iabg[4];

        US_C(0) = CTRL.O->uab_cmd[0+2]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = CTRL.O->uab_cmd[1+2]; // 后缀_C表示当前步的电压，C = Current

        US_P(0) = US_C(0);
        US_P(1) = US_C(1);

    }else if(Axis.use_first_set_three_phase==-1){
        IS_C(0)        = Axis.iabg[3];
        IS_C(1)        = Axis.iabg[4];
        CTRL.I->iab[0] = Axis.iabg[3]; // 第二套逆变器控制转矩
        CTRL.I->iab[1] = Axis.iabg[4];
        CTRL.I->iab[0+2] = Axis.iabg[0];
        CTRL.I->iab[1+2] = Axis.iabg[1];
    }


    //    这样不能形成保护，必须设置故障状态才行。
    //    if(fabs(G.Current_W)>8 || fabs(G.Current_V)>8){
    //        DSP_PWM_DISABLE
    //        DSP_2PWM_DISABLE
    //    }

    // 电流采样ADC温飘校准 // TODO 改成用ADC Raw Results校准。
//    if(Axis.AD_offset_flag2==FALSE)
//    {
//        Axis.offset_counter += 1;
//        Axis.iuvw_offset_online[0] += (REAL)(AdcaResultRegs.ADCRESULT1 ) ;
//        Axis.iuvw_offset_online[1] += (REAL)(AdcaResultRegs.ADCRESULT2 ) ;
//        Axis.iuvw_offset_online[2] += (REAL)(AdcaResultRegs.ADCRESULT3 ) ;
//        Axis.iuvw_offset_online[3] += (REAL)(AdcaResultRegs.ADCRESULT11 ) ;
//        Axis.iuvw_offset_online[4] += (REAL)(AdcaResultRegs.ADCRESULT9 ) ;
//        Axis.iuvw_offset_online[5] += (REAL)(AdcaResultRegs.ADCRESULT8 ) ;
//        if(Axis.offset_counter>=5000){
//            Axis.iuvw_offset_online[0] = Axis.iuvw_offset_online[0] / 5000;
//            Axis.iuvw_offset_online[1] = Axis.iuvw_offset_online[1] / 5000;
//            Axis.iuvw_offset_online[2] = Axis.iuvw_offset_online[2] / 5000;
//            Axis.iuvw_offset_online[3] = Axis.iuvw_offset_online[3] / 5000;
//            Axis.iuvw_offset_online[4] = Axis.iuvw_offset_online[4] / 5000;
//            Axis.iuvw_offset_online[5] = Axis.iuvw_offset_online[5] / 5000;
//            Axis.AD_offset_flag2 = TRUE;
//            Axis.offset_counter = 0;
//        }
//
//        // 来不及完成偏置检测（比如刚上电数字开关就是开的），采用默认值
//        /* 427-1401：添加开关信号滤波。今天发现在刚上电的时候，XCUBE-II的前两个中断里，数字开关是打开的，然后才变成关闭。*/
//        if(Axis.FLAG_ENABLE_PWM_OUTPUT && Axis.offset_counter>100){
//            Axis.iuvw_offset_online[0] = 0.0;
//            Axis.iuvw_offset_online[1] = 0.0;
//            Axis.iuvw_offset_online[2] = 0.0;
//            Axis.iuvw_offset_online[3] = 0.0;
//            Axis.iuvw_offset_online[4] = 0.0;
//            Axis.iuvw_offset_online[5] = 0.0;
//            Axis.AD_offset_flag2 = TRUE;
//        }
//
//        // 上电的时候，电机可能在转，此时根据电流判断是否还要额外进行偏置补偿。
//        if( fabs(Axis.iuvw[0])>0.05 || fabs(Axis.iuvw[1])>0.05 || fabs(Axis.iuvw[2])>0.05 || \
//            fabs(Axis.iuvw[3])>0.05 || fabs(Axis.iuvw[4])>0.05 || fabs(Axis.iuvw[5])>0.05){
//            Axis.iuvw_offset_online[0] = 0.0;
//            Axis.iuvw_offset_online[1] = 0.0;
//            Axis.iuvw_offset_online[2] = 0.0;
//            Axis.iuvw_offset_online[3] = 0.0;
//            Axis.iuvw_offset_online[4] = 0.0;
//            Axis.iuvw_offset_online[5] = 0.0;
//            Axis.AD_offset_flag2 = TRUE;
//        }
//    }
}
// int down_freq_ecap_counter = 1;
Uint64 timebase_counter = 0;
extern REAL imife_realtime_gain_off;
void PanGuMainISR(void){
    #if NUMBER_OF_DSP_CORES == 2
        write_DAC_buffer();
    #endif


//    static long int ii = 0;
//    if(++ii%5000 == 0){
//        //        EPwm1Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
//        //        EPwm2Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
//        //        EPwm3Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;
//        //        EPwm4Regs.CMPA.bit.CMPA = CTRL.svgen2.Ta*50000000*CL_TS;
//        //        EPwm5Regs.CMPA.bit.CMPA = CTRL.svgen2.Tb*50000000*CL_TS;
//
//        if(EPwm1Regs.CMPA.bit.CMPA==5000){
//            EPwm1Regs.CMPA.bit.CMPA = 0;
//        }else{
//            EPwm1Regs.CMPA.bit.CMPA = 5000;
//        }
//    }
//
//    static long int jj = 0;
//    if(++jj%10000 == 0){
//        if(EPwm2Regs.CMPA.bit.CMPA==5000){
//            EPwm2Regs.CMPA.bit.CMPA = 0;
//        }else{
//            EPwm2Regs.CMPA.bit.CMPA = 5000;
//        }
//    }
//    return;


    #if ENABLE_ECAP
        do_enhanced_capture();
    #endif

    // 采样，包括DSP中的ADC采样等
    // DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?
    measurement();


    if(!Axis.FLAG_ENABLE_PWM_OUTPUT){

        DSP_PWM_DISABLE
        DSP_2PWM_DISABLE

        /* Only init once for easy debug */
        if(!G.flag_experimental_initialized){

            init_experiment();
            //G.Select_exp_operation = 3; // fixed
            init_experiment_overwrite();

            FE.htz.rs_est = 3.8; // 通过调节电阻值等效补偿Vdc=80V时的死区电压误差（按慢反转实验效果调节获得）

            // 电机铭牌处摸着烫，电阻变大了？？
            //FE.htz.rs_est = 4.2; // Ohm 通过调节电阻值等效补偿Vdc=120V时的死区电压误差（按慢反转实验效果调节获得）

            //CTRL.motor->Lmu_inv = 1.55; // H^-1 使得实测alpha-beta电流李萨茹图为圆形且不会发生偏心转动
            //CTRL.I->m0 = 2.5; // Wb
            // load motor is iq=-10 A

            if(CTRL.g->overwrite_vdc<5){
                //CTRL.g->overwrite_vdc = 80;
//                CTRL.g->overwrite_vdc = 120;
                //CTRL.g->overwrite_vdc = 200;
                //正弦实验
                CTRL.g->overwrite_vdc = 28;
//                FE.htz.rs_est=4.45;
//                marino.gamma_inv=150000;
//                marino.lambda_inv=2000;
//                marino.xAlpha=6.25; // 使得正弦转速的波头跟踪变好
//                imife_realtime_gain_off=0.01;
//                CTRL.motor->Js_inv=30; // 使得负载转矩估计值变成常数而不是正弦波（正弦波是惯量误差*加速度形成的惯性负载）
            }
            CTRL.g->flag_overwite_vdc = 0;

            G.flag_experimental_initialized = TRUE;
        }

        DELAY_US(11);
        GpioDataRegs.GPDCLEAR.bit.GPIO106=1; // TODO: What is this doing?

    }else{
        G.flag_experimental_initialized = FALSE;
        DSP_PWM_ENABLE
        DSP_2PWM_ENABLE
        if (FE.htz.u_offset[0] > 0.1){
            FE.htz.u_offset[0] = 0;
        }
        // DSP中控制器的时间
        timebase_counter += 1;
        CTRL.timebase = CL_TS * timebase_counter; //CTRL.timebase += CL_TS; // 2048 = float/double max

        // 根据指令，产生控制输出（电压）
        #if ENABLE_COMMISSIONING == FALSE
            //CTRL.S->Motor_or_Gnerator = sign(CTRL.I->idq_cmd[1]) == sign(ENC.rpm); // sign(CTRL.I->idq_cmd[1]) != sign(CTRL.I->cmd_speed_rpm))
            runtime_command_and_tuning(Axis.Select_exp_operation);

            error_pos = target_position_cnt - CTRL.enc->encoder_abs_cnt;
            if (error_pos > (SYSTEM_QEP_QPOSMAX_PLUS_1/2))
            {
                error_pos -= SYSTEM_QEP_QPOSMAX_PLUS_1;
            }
            if (error_pos < -(SYSTEM_QEP_QPOSMAX_PLUS_1/2))
            {
                error_pos += SYSTEM_QEP_QPOSMAX_PLUS_1;
            }
            Axis.Set_manual_rpm = error_pos*KP;
            Axis.used_theta_d_elec = controller(Axis.Set_manual_rpm, Axis.Set_current_loop, Axis.Set_manual_current_iq, Axis.Set_manual_current_id,
                Axis.flag_overwrite_theta_d, Axis.Overwrite_Current_Frequency,
                //Axis.used_theta_d_elec,
                Axis.angle_shift_for_first_inverter,
                Axis.angle_shift_for_second_inverter);
        #else
            commissioning();
        #endif

        if(Axis.Select_exp_operation == XCUBE_TaTbTc_DEBUG_MODE){
            EPwm1Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
            EPwm2Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
            EPwm3Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;

            EPwm4Regs.CMPA.bit.CMPA = CTRL.svgen2.Ta*50000000*CL_TS;
            EPwm5Regs.CMPA.bit.CMPA = CTRL.svgen2.Tb*50000000*CL_TS;
            EPwm6Regs.CMPA.bit.CMPA = CTRL.svgen2.Tc*50000000*CL_TS;

            // 20240119 test，观察4、5、6通道的IPM输出电压是否正常
//            EPwm4Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
//            EPwm5Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
//            EPwm6Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;
        }
        else
            voltage_commands_to_pwm();


        //        #if NUMBER_OF_DSP_CORES == 1
        //            single_core_dac();
        //        #endif

    }
}

Uint64 EPWM1IntCount=0;
__interrupt void EPWM1ISR(void){
    EPWM1IntCount += 1;


#if USE_ECAP_CEVT2_INTERRUPT == 1
    CAP.password_isr_nesting = 178; // only if you can stop EPWM ISR, or else you won't know the value of password_isr_nesting.
    /* Step 1. [eCAP] Set the global priority */
    // Set global priority by adjusting IER, so as to allow PIE group 4 to send interrupt flag to CPU stage.
    IER |= M_INT4; // Modify IER to allow CPU interrupts from PIE group 4 to be serviced. Part 1
    IER &= M_INT4; // Modify IER to allow CPU interrupts from PIE group 4 to be serviced. Part 2

    /* Step 2. [eCAP] Set the group priority */
    uint16_t TempPIEIER4;
    TempPIEIER4 = PieCtrlRegs.PIEIER4.all; // Save PIEIER register for later
    PieCtrlRegs.PIEIER4.all &= 0x7;        // Set group priority by adjusting PIEIER4 to allow INT4.1, 4.2, 4.3 to interrupt current ISR

    /* Step 3. [eCAP] Enable interrupts */
    PieCtrlRegs.PIEACK.all = 0xFFFF;      // Enable PIE interrupts by writing all 1’s to the PIEACK register
    asm("       NOP");                    // Wait at least one cycle
    EINT;                                 // Enable global interrupts by clearing INTM
#endif

    /* Step 4. [ePWM] Execute EPWM ISR */
    PanGuMainISR();

#if USE_ECAP_CEVT2_INTERRUPT == 1
    /* Step 5. [eCAP] Disable interrupts */
    DINT;

    /* Step 6. [eCAP] Restore the PIEIERx register */
    PieCtrlRegs.PIEIER4.all = TempPIEIER4;
    CAP.password_isr_nesting = 0;
#endif

    /* Step 7. [ePWM] Exit EPWM1 ISR */
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

#endif



/*
声明全局变量
#if PC_SIMULATION==FALSE
double CpuTimer_Delta = 0;
Uint32 CpuTimer_Before = 0;
Uint32 CpuTimer_After = 0;
#endif


这段放需要测时间的代码前面
#if PC_SIMULATION==FALSE
EALLOW;
CpuTimer1.RegsAddr->TCR.bit.TRB = 1; // reset cpu timer to period value
CpuTimer1.RegsAddr->TCR.bit.TSS = 0; // start/restart
CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
EDIS;
#endif

这段放需要测时间的代码后面前面，观察CpuTimer_Delta的取值，代表经过了多少个 1/200e6 秒。
#if PC_SIMULATION==FALSE
CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
CpuTimer_Delta = (double)CpuTimer_Before - (double)CpuTimer_After;
// EALLOW;
// CpuTimer1.RegsAddr->TCR.bit.TSS = 1; // stop (not needed because of the line TRB=1)
// EDIS;
#endif
*/
