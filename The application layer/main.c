#include <All_Definition.h>
//
// Globals
//
Uint16 LoopCount;
Uint16 ErrorCount;
Uint16 SendChar;
Uint16 ReceivedChar;

void init_experiment_AD_gain_and_offset(){
    /* ADC OFFSET */
    #ifdef _XCUBE1
        G.offsetU=2044;
        G.offsetV=2052;
        G.offsetW=2032; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
        G.offsetUDC=1430;
        // /* 借用Sensor Board的电流传感器 */
        // G.offsetU=2045;
        // G.offsetV=2047;
        // G.offsetW=2030;
    #else
        G.offsetU=2075;
        G.offsetV=2062;
        G.offsetW=2047; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
        G.offsetUDC=0; // 6.03
    #endif

    /* ADC SCALE */
    #ifdef _XCUBE1
        G.AD_scale_U   = 0.0109649;
        G.AD_scale_V   = 0.00988908653610677; // (degaussed CP030A and offset ADC) // 0.01125872551 (YX)
        G.AD_scale_W   = 0.00972075677822004; // (degaussed CP030A and offset ADC) // 0.01134558656 (YX)
        G.AD_scale_VDC = 0.3546099;
            /* 借用Sensor Board的电流传感器 */
            //#define AD_scale_W                       0.0057316444
            //#define AD_scale_V                       0.0056072670
    #else
        G.AD_scale_W   = 0.0049418264616;
        G.AD_scale_V   = 0.0064003544;
        G.AD_scale_U   = 0.005325766233109334;
        G.AD_scale_VDC = 0.09375740207975308; //=180.1 / (182.25 / (0.5*0.1897533207)) // 连ecap系数变！ // =0.5*0.1897533207 / 60 * 80, 袁鑫6月2日改的硬件？ //  (0.5*0.1897533207); // 短上一个470R再补上一个470R，所以乘0.5。
    #endif

    /* eQEP OFFSET */
    CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis = SYSTEM_QEP_CALIBRATED_ANGLE;
    CTRL.enc->theta_d_offset = CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis * CNT_2_ELEC_RAD;
}
void main(void){

    InitSysCtrl();        // 1. Initialize System Control: PLL, WatchDog, enable Peripheral Clocks.
    Gpio_initialize();    // 2. Initialize GPIO and assign GPIO to peripherals.
    DINT;                 // 3.1 Clear all interrupts and initialize PIE vector table.
    InitPieCtrl();        // 3.2 Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;         // 3.3 Disable CPU __interrupts,
    IFR = 0x0000;         // 3.4 and clear all CPU __interrupt flags.
    InitPieVectTable();   // 3.5 Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). At end, ENPIE = 1.

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
        InitSpiaGpio();
        InitSpi();
        InitSciGpio();
        InitSci();
    #elif NUMBER_OF_DSP_CORES == 2
        /* 双核配置*/
        // 初始化SPI，用于与DAC芯片MAX5307通讯。
        EALLOW;
        DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
        DevCfgRegs.CPUSEL5.bit.SCI_C = 1; // assign sci-ca to cpu2
        EDIS;

        InitSpiaGpio();
        //InitSpi(); // this is moved to CPU02
        InitSciGpio();
        //InitSci(); // this is moved to CPU02

        // 在此之前，已经把GPIO和外设的权限转给CPU2了。
        // 这里再把部分贡献内存权限给CPU2，同时告诉CPU2，你可以继续运行代码了。
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

    // 5. Handle Interrupts
    /* Re-map PIE Vector Table to user defined ISR functions. */
        EALLOW; // This is needed to write to EALLOW protected registers
        PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM主中断 10kKHz
        PieVectTable.ECAP1_INT = &ecap1_isr;
        PieVectTable.ECAP2_INT = &ecap2_isr;
        PieVectTable.ECAP3_INT = &ecap3_isr;
        PieVectTable.EQEP1_INT = &EQEP_UTO_INT;      // eQEP
        PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;   // SCI Receive
        PieVectTable.SCIC_TX_INT = &scicTxFifoIsr;   // SCI Transmit
        EDIS; // This is needed to disable write to EALLOW protected registers
    /* PIE Control */
        PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // PWM1 interrupt (Interrupt 3.1)
        #if NUMBER_OF_DSP_CORES == 1
            PieCtrlRegs.PIEIER8.bit.INTx5 = 1;   // PIE Group 8, INT5, SCI-C Rx
            PieCtrlRegs.PIEIER8.bit.INTx6 = 1;   // PIE Group 8, INT6, SCI-C Tx
        #endif
        #if USE_ECAP_CEVT2_INTERRUPT == 1
            PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
            PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // 1 Enable for Interrupt 4.2
            PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // 2 Enable for Interrupt 4.3
        #endif
        #if SYSTEM_PROGRAM_MODE != 223
            PieCtrlRegs.PIEIER5.bit.INTx1 = 1;      // QEP interrupt
        #endif
    /* CPU Interrupt Enable Register (IER) */
        IER |= M_INT3;  // EPWM1_INT
        #if NUMBER_OF_DSP_CORES == 1
            IER |= M_INT8; // SCI-C
        #endif
        #if USE_ECAP_CEVT2_INTERRUPT == 1
            IER |= M_INT4;  // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
        #endif
        IER |= M_INT5;  // EQEP1_INT???
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

    SendChar = 24;
    for(;;)
    {
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

    // 7. Main loop
    while(1){
        //STATE_APP_MachineState();
        //System_Checking();  //状态机第一个状态 ：系统自检
        //System_Protection();//IU/IV/VOLTAGE保护

        //#define Motor_mode_START    GpioDataRegs.GPADAT.bit.GPIO26          //DI Start Button
        if (Motor_mode_START==1){
            G.FLAG_ENABLE_PWM_OUTPUT = 1;
            DSP_START_LED1
            DSP_START_LED2
        }else if (Motor_mode_START==0){
            G.FLAG_ENABLE_PWM_OUTPUT = 0;
            DSP_STOP_LED1
            DSP_STOP_LED2
        }

        #if NUMBER_OF_DSP_CORES == 2
            if(IPCLtoRFlagBusy(IPC_FLAG10) == 1) // if flag
            {
                Write.test += 0.001;
                IPCRtoLFlagAcknowledge (IPC_FLAG10);
            }
        #endif
    }
}




/* Below is moved from CJHMainISR.c */
#if SYSTEM_PROGRAM_MODE==223
void voltage_commands_to_pwm(){
    // ----------------------------SVGEN生成-------------------------------------------------
    CTRL.svgen1.Ualpha= CTRL.O->uab_cmd_to_inverter[0];
    CTRL.svgen1.Ubeta = CTRL.O->uab_cmd_to_inverter[1];
    //    Deadtime_compensation_selection();
    SVGEN_Drive(&CTRL.svgen1);
    //SPWM_Drive(&CTRL.svgen1);

    // ------------------------------------------------------------------------------
    EPwm1Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
    EPwm2Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
    EPwm3Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;
}
void measurement(){

    // 母线电压测量
    G.Voltage_DC_BUS=((REAL)(AdcaResultRegs.ADCRESULT0)-G.offsetUDC)*G.AD_scale_VDC + G.Offset_Udc;//

    // 相电压测量（基于占空比和母线电压）
    CAP.terminal_voltage[0] = (CAP.terminal_DutyOnRatio[0]) * G.Voltage_DC_BUS - G.Voltage_DC_BUS * 0.5; // -0.5 is due to duty ratio calculation; - Voltage_DC_BUS * 0.5 is referring to the center of dc bus capacitor.
    CAP.terminal_voltage[1] = (CAP.terminal_DutyOnRatio[1]) * G.Voltage_DC_BUS - G.Voltage_DC_BUS * 0.5;
    CAP.terminal_voltage[2] = (CAP.terminal_DutyOnRatio[2]) * G.Voltage_DC_BUS - G.Voltage_DC_BUS * 0.5;

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

    // 电流接口
    #ifdef _XCUBE1
        //        G.Current_Not_Used=((AdcaResultRegs.ADCRESULT1)-offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        //        G.Current_V       =((AdcaResultRegs.ADCRESULT3)-offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        //        G.Current_U       =((AdcaResultRegs.ADCRESULT2)-offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        //        if(AD_offset_flag2==TRUE){
        //            G.Current_Not_Used = G.Current_Not_Used - G.Offset_W;
        //            G.Current_V = G.Current_V - G.Offset_V;
        //            G.Current_U = G.Current_U - G.Offset_U;
        //        }
        //        G.Current_W=-(G.Current_V+G.Current_U);

        if(G.flag_overwite_voltage_dc_bus){
            G.Voltage_DC_BUS = G.Overwrite_Voltage_DC_BUS;
        }
        G.Current_W       =((AdcaResultRegs.ADCRESULT1)-G.offsetW)*G.AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        G.Current_V       =((AdcaResultRegs.ADCRESULT3)-G.offsetV)*G.AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        G.Current_Not_Used=((AdcaResultRegs.ADCRESULT2)-G.offsetU)*G.AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        if(G.AD_offset_flag2==TRUE){
            G.Current_W = G.Current_W - G.Offset_W;
            G.Current_V = G.Current_V - G.Offset_V;
            G.Current_Not_Used = G.Current_Not_Used - G.Offset_U;
        }
        G.Current_U=-(G.Current_W+G.Current_V);

    #else
        G.Current_W       =((REAL)(AdcaResultRegs.ADCRESULT1)-G.offsetW)*G.AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        G.Current_Not_Used=((REAL)(AdcaResultRegs.ADCRESULT3)-G.offsetV)*G.AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        G.Current_U       =((REAL)(AdcaResultRegs.ADCRESULT2)-G.offsetU)*G.AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        if(G.AD_offset_flag2==TRUE){
            G.Current_W = G.Current_W - G.Offset_W;
            G.Current_Not_Used = G.Current_Not_Used - G.Offset_V;
            G.Current_U = G.Current_U - G.Offset_U;
        }
        G.Current_V=-(G.Current_W+G.Current_U);
    #endif
    REAL adc_ial = UV2A_AI(G.Current_U, G.Current_V);
    REAL adc_ibe = UV2B_AI(G.Current_U, G.Current_V);

    // 电流测量
    IS_C(0)        = adc_ial;
    IS_C(1)        = adc_ibe;
    CTRL.I->iab[0] = adc_ial;
    CTRL.I->iab[1] = adc_ibe;

    // 转子位置和转速接口 以及 转子位置和转速测量
    {
        Uint32 QPOSCNT   = EQep1Regs.QPOSCNT;
        ENC.rpm          = PostionSpeedMeasurement_MovingAvergage(QPOSCNT);
        ENC.omg_elec     = ENC.rpm * RPM_2_ELEC_RAD_PER_SEC; // 机械转速（单位：RPM）-> 电气角速度（单位：elec.rad/s)
        ENC.theta_d_elec = ENC.theta_d__state;
    }
    CTRL.I->omg_elec = ENC.omg_elec;
    CTRL.I->rpm = CTRL.I->omg_elec * ELEC_RAD_PER_SEC_2_RPM;
    CTRL.I->theta_d_elec = ENC.theta_d_elec;

    //    这样不能形成保护，必须设置故障状态才行。
    //    if(fabs(G.Current_W)>8 || fabs(G.Current_V)>8){
    //        DSP_PWM_DISABLE
    //        DSP_2PWM_DISABLE
    //    }


    // 电流采样ADC温飘校准 // TODO 改成用ADC Raw Results校准。
    if(G.AD_offset_flag2==FALSE)
    {
        G.Offset_Counter += 1;
        G.Offset_W += G.Current_W;
        G.Offset_V += G.Current_V;
        G.Offset_U += G.Current_U;
        if(G.Offset_Counter>5000){
            G.Offset_W = G.Offset_W / 5000;
            G.Offset_V = G.Offset_V / 5000;
            G.Offset_U = G.Offset_U / 5000;
            G.AD_offset_flag2 = TRUE;
        }

        // 来不及完成偏置检测（比如刚上电数字开关就是开的），采用默认值
        /* 427-1401：添加开关信号滤波。今天发现在刚上电的时候，XCUBE-II的前两个中断里，数字开关是打开的，然后才变成关闭。*/
        if(G.FLAG_ENABLE_PWM_OUTPUT && G.Offset_Counter>100){
            G.Offset_W = 0.0;
            G.Offset_V = 0.0;
            G.Offset_U = 0.0;
            G.AD_offset_flag2 = TRUE;
        }

        // 上电的时候，电机可能在转，此时根据电流判断是否还要额外进行偏置补偿。
        if(fabs(G.Current_W)>0.05 || fabs(G.Current_V)>0.05 || fabs(G.Current_U)>0.05){
            G.Offset_W = 0.0;
            G.Offset_V = 0.0;
            G.Offset_U = 0.0;
            G.AD_offset_flag2 = TRUE;
        }
    }
}
// int down_freq_ecap_counter = 1;
void CJHMainISR(void){
    #if NUMBER_OF_DSP_CORES == 2
        write_DAC_buffer();
    #endif

    //if(down_freq_ecap_counter++==2){
    do_enhanced_capture();
    //    down_freq_ecap_counter = 1;
    //}

    // 采样，包括DSP中的ADC采样等
    DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?
    measurement();

    if(!G.FLAG_ENABLE_PWM_OUTPUT){

        DSP_PWM_DISABLE
        DSP_2PWM_DISABLE

        /* Only init once for easy debug */
        if(!G.flag_experimental_initialized){
            G.flag_experimental_initialized = TRUE;

            init_experiment();
            //G.Seletc_exp_operation = 3; // fixed
            init_experiment_overwrite();
        }

        DELAY_US(11);
        GpioDataRegs.GPDCLEAR.bit.GPIO106=1; // TODO: What is this doing?

    }else{
        G.flag_experimental_initialized = FALSE;
        DSP_PWM_ENABLE
        DSP_2PWM_ENABLE

        // DSP中控制器的时间
        CTRL.timebase += CL_TS;

        // 根据指令，产生控制输出（电压）
        #if ENABLE_COMMISSIONING == FALSE
            CTRL.S->Motor_or_Gnerator = sign(CTRL.I->idq_cmd[1]) == sign(ENC.rpm); // sign(CTRL.I->idq_cmd[1]) != sign(CTRL.I->cmd_speed_rpm))
            runtime_command_and_tuning(G.Seletc_exp_operation);
            controller(G.Set_manual_rpm, G.Set_manual_current_iq, G.Set_manual_current_id);
        #else
            commissioning();
        #endif

        if(G.Seletc_exp_operation == XCUBE_TaTbTc_DEBUG_MODE){
            //   CTRL.svgen1.Ta = 0.6; CTRL.svgen1.Tb = 0.4; CTRL.svgen1.Tc = 0.5;
            if(CTRL.svgen1.Ta>0.7) CTRL.svgen1.Ta=0.7;
            if(CTRL.svgen1.Ta<0.3) CTRL.svgen1.Ta=0.3;
            if(CTRL.svgen1.Tb>0.7) CTRL.svgen1.Tb=0.7;
            if(CTRL.svgen1.Tb<0.3) CTRL.svgen1.Tb=0.3;
            if(CTRL.svgen1.Tc>0.7) CTRL.svgen1.Tc=0.7;
            if(CTRL.svgen1.Tc<0.3) CTRL.svgen1.Tc=0.3;
            EPwm1Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
            EPwm2Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
            EPwm3Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;
        }
        else
            voltage_commands_to_pwm();

        #if NUMBER_OF_DSP_CORES == 1
            single_core_dac();
        #endif
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
    CJHMainISR();

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
