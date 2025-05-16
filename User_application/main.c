#include "All_Definition.h"

// 声明全局变量
bool run_enable_from_PC = FALSE;
#if PC_SIMULATION == FALSE
REAL CpuTimer_Delta = 0;
Uint32 CpuTimer_Before = 0;
Uint32 CpuTimer_After = 0;
#endif

void main(void){

    InitSysCtrl();      // 1. Initialize System Control: PLL, WatchDog, enable Peripheral Clocks.
    Gpio_initialize();  // 2. Initialize GPIO and assign GPIO to peripherals.
    DINT;               // 3.1 Clear all interrupts and initialize PIE vector table.
    InitPieCtrl();      // 3.2 Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;       // 3.3 Disable CPU __interrupts,
    IFR = 0x0000;       // 3.4 and clear all CPU __interrupt flags.
    InitPieVectTable(); // 3.5 Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). At end, ENPIE = 1.
    InitCpuTimers();     // for Slessinv TIE.R1 for measuring the execution time
    ConfigCpuTimer(&CpuTimer1, 200, 1000000); // 200MHz, INTERRUPT_period = 1e6 us
    #if NUMBER_OF_DSP_CORES == 2
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);     // 4.1 IPC
    #endif
    #ifdef _STANDALONE
        #ifdef _FLASH
            // use this code when u need to cut off the power and reboot it offline
            //  Send boot command to allow the CPU02 application to begin executioz`n
            IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
        #else
            //  Send boot command to allow the CPU02 application to begin execution
            // seriesly i dont know what does this sentences mean, do no change it might as well
            // i dont know what does sentence do
            // IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
        #endif
    #endif

    // 4.2 Initialize peripherals for CPU01
    ePWM_initialize();
    ADC_initialize();
    eQEP_initialize(0);
    InitECaptureContinuousMode();

    init_experiment_AD_gain_and_offset();

    // Hall sensor
        GPIO_SetupPinMux(68, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(68, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(94, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(17, GPIO_INPUT, GPIO_SYNC);

        GPIO_SetupPinMux(127, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(127, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(116, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(116, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(131, GPIO_INPUT, GPIO_SYNC);

        GPIO_SetupPinMux(115, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(115, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(117, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(117, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(130, GPIO_INPUT, GPIO_SYNC);

        GPIO_SetupPinMux(128, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(128, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(126, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(126, GPIO_INPUT, GPIO_SYNC);
        GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(95, GPIO_INPUT, GPIO_SYNC);


    /* TODO: need to assign I2C to CPU02 */
    /* TODO: need to assign I2C to CPU02 */
    /* TODO: need to assign I2C to CPU02 */
    /*This part is corresponding to the Seeed's Github, of which address is attached below:
     https://github.com/Seeed-Studio/Seeed_LDC1612/blob/master/Seeed_LDC1612.cpp
     This part is corresponding to sensor.single_channel_config from Seeed-LDC1612 */
    // Use GPIO0, GPIO1 and reuse mode is 6 (I2C)
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 6);
     // GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 6);
     // GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 6);
    I2CA_Init();
    Single_channel_config(0); // 0 for CHANNEL_0 



    // 4.3 Assign peripherals to CPU02
    /* SPI and SCI */
    #if NUMBER_OF_DSP_CORES == 1 /* 377单核配置*/
        // 同步 !!!!!!!!
        InitHighSpeedSpiGpio();
        // InitSpiaGpio();
        InitSpi();
        InitSciGpio();
        InitSci();
    #elif NUMBER_OF_DSP_CORES == 2 /* 377双核配置*/
        init_spi_sci_can();
        // 在此之前，已经把GPIO和外设的权限转给CPU2了。
        // 这里再把部分共享内存权限给CPU2，同时告诉CPU2，你可以继续运行代码了。
        while (!(MemCfgRegs.GSxMSEL.bit.MSEL_GS0)){
            EALLOW;
            // Give Memory Access to GS0/ GS14 SARAM to CPU02
            MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
            EDIS;
        }
    #endif

    // 4.4 Initialize algorithms
    user_routine_init_in_main(); // 用户自定义的初始化函数

    // 5. Deal with Interrupts
    handle_interrupts();

    // 6. Pre main loop
    DSP_STOP_LED1
    DSP_STOP_LED2
    DSP_PWM1_DISABLE
    DSP_PWM2_DISABLE
    DSP_PWM3_DISABLE
    DSP_PWM4_DISABLE
    /* Test IPC to CPU02 */
    #if NUMBER_OF_DSP_CORES == 2
        test_ipc_tocpu02();
    #endif

    // 7. CLA
    // cla_test_codes();

    // 8. Main loop
    while (1){
        user_routine_in_main_loop(); // 用户自定义的主循环函数
        #if NUMBER_OF_DSP_CORES == 1
            single_core_dac();
        #endif
    }
}

void main_measurement(){

    if(DSP.if_enable_EQEP){
        PostionSpeedMeasurement_MovingAvergage(EQep1Regs.QPOSCNT, CTRL->enc);
    }

    if(DSP.if_enable_AbsoluteEncoder){
        // read encoder counter data from absolute encoder
        CTRL->enc->encoder_abs_cnt_previous = CTRL->enc->encoder_abs_cnt;
        if (axisCnt == 0) measurement_position_count_axisCnt0();
        if (axisCnt == 1) measurement_position_count_axisCnt1();
        // measure d-axis angle and then use a poor algorithm to calculate speed
        measurement_enc();
    }

    // measure place between machine shaft and Sensor Coil
    if (DSP.if_enable_I2C){
        measurement_sensor_coil();
    }

    // LEM1
    DSP.iuvw[0] = ((REAL)(AdcaResultRegs.ADCRESULT1) - DSP.adc_offset[1]) * DSP.adc_scale[1]; //
    DSP.iuvw[1] = ((REAL)(AdcaResultRegs.ADCRESULT2) - DSP.adc_offset[2]) * DSP.adc_scale[2]; //
    DSP.iuvw[2] = ((REAL)(AdcaResultRegs.ADCRESULT3) - DSP.adc_offset[3]) * DSP.adc_scale[3]; //
    // LEM2
    DSP.iuvw[3] = ((REAL)(AdcbResultRegs.ADCRESULT7) - DSP.adc_offset[4]) * DSP.adc_scale[4]; //
    DSP.iuvw[4] = ((REAL)(AdcbResultRegs.ADCRESULT8) - DSP.adc_offset[5]) * DSP.adc_scale[5]; //
    DSP.iuvw[5] = ((REAL)(AdcbResultRegs.ADCRESULT9) - DSP.adc_offset[6]) * DSP.adc_scale[6]; //
    // LEM3
    DSP.iuvw[6] = 0;
    DSP.iuvw[7] = 0;
    DSP.iuvw[8] = 0;
    // LEM4
    DSP.iuvw[6] = 0;
    DSP.iuvw[7] = 0;
    DSP.iuvw[8] = 0;
    // 电流接口
    if (DSP.USE_3_CURRENT_SENSORS){
        DSP.iabg[0] = UVW2A_AI(DSP.iuvw[0], DSP.iuvw[1], DSP.iuvw[2]);
        DSP.iabg[1] = UVW2B_AI(DSP.iuvw[0], DSP.iuvw[1], DSP.iuvw[2]);
        DSP.iabg[2] = UVW2G_AI(DSP.iuvw[0], DSP.iuvw[1], DSP.iuvw[2]);
        DSP.iabg[3] = UVW2A_AI(DSP.iuvw[3], DSP.iuvw[4], DSP.iuvw[5]);
        DSP.iabg[4] = UVW2B_AI(DSP.iuvw[3], DSP.iuvw[4], DSP.iuvw[5]);
        DSP.iabg[5] = UVW2G_AI(DSP.iuvw[3], DSP.iuvw[4], DSP.iuvw[5]);
        DSP.iabg[6] = UVW2A_AI(DSP.iuvw[6], DSP.iuvw[7], DSP.iuvw[8]);
        DSP.iabg[7] = UVW2B_AI(DSP.iuvw[6], DSP.iuvw[7], DSP.iuvw[8]);
        DSP.iabg[8] = UVW2G_AI(DSP.iuvw[6], DSP.iuvw[7], DSP.iuvw[8]);
        DSP.iabg[9] = UVW2A_AI(DSP.iuvw[9], DSP.iuvw[10], DSP.iuvw[11]);
        DSP.iabg[10] = UVW2B_AI(DSP.iuvw[9], DSP.iuvw[10], DSP.iuvw[11]);
        DSP.iabg[11] = UVW2G_AI(DSP.iuvw[9], DSP.iuvw[10], DSP.iuvw[11]);
    }else{
        // REAL phase_V_current = -DSP.iuvw[0] - DSP.iuvw[2]; // 如果V相没有测量的话就用U和W去算V相电流
        DSP.iabg[0] = UV2A_AI(DSP.iuvw[0], DSP.iuvw[1]);
        DSP.iabg[1] = UV2B_AI(DSP.iuvw[0], DSP.iuvw[1]);;
        DSP.iabg[2] = 0.0;
        DSP.iabg[3] = UV2A_AI(DSP.iuvw[3], DSP.iuvw[4]);
        DSP.iabg[4] = UV2B_AI(DSP.iuvw[3], DSP.iuvw[4]);
        DSP.iabg[5] = 0.0;
        DSP.iabg[6] = UV2A_AI(DSP.iuvw[6], DSP.iuvw[7]);
        DSP.iabg[7] = UV2B_AI(DSP.iuvw[6], DSP.iuvw[7]);
        DSP.iabg[8] = 0.0;
        DSP.iabg[9]  = UV2A_AI(DSP.iuvw[9], DSP.iuvw[10]);
        DSP.iabg[10] = UV2B_AI(DSP.iuvw[9], DSP.iuvw[10]);
        DSP.iabg[11] = 0.0;
    }


    // measure vdc 四套三相逆变器可能有独立的母线电压值
    DSP.vdc[0] = ((REAL)(AdcaResultRegs.ADCRESULT0) - DSP.adc_offset[0]) * DSP.adc_scale[0];
    DSP.vdc[1] = ((REAL)(AdcbResultRegs.ADCRESULT6) - DSP.adc_offset[0]) * DSP.adc_scale[0];
    DSP.vdc[2] = 0.0;
    DSP.vdc[3] = 0.0;
    if (DSP.flag_overwite_vdc) {
        DSP.vdc[0] = DSP.overwrite_vdc;
    }

    // TODO: 这里应该用I2T指标，计算发热用于判断故障，瞬间过流一般没事
    if(fabs(DSP.iuvw[0])>8 
    || fabs(DSP.iuvw[3])>8 
    || fabs(DSP.iuvw[6])>8
    || fabs(DSP.iuvw[9])>8
    ){
        DSP.trip_flag = TRUE;
    }

    // 线电压（占空比）测量（基于占空比和母线电压）
    // voltage_measurement_based_on_eCAP();

    // 电流采样ADC温飘校准 // TODO 改成用ADC Raw Results校准。
    //    if(DSP.AD_offset_flag2==FALSE)
    //    {
    //        DSP.offset_counter += 1;
    //        DSP.iuvw_offset_online[0] += (REAL)(AdcaResultRegs.ADCRESULT1 ) ;
    //        DSP.iuvw_offset_online[1] += (REAL)(AdcaResultRegs.ADCRESULT2 ) ;
    //        DSP.iuvw_offset_online[2] += (REAL)(AdcaResultRegs.ADCRESULT3 ) ;
    //        DSP.iuvw_offset_online[3] += (REAL)(AdcaResultRegs.ADCRESULT11 ) ;
    //        DSP.iuvw_offset_online[4] += (REAL)(AdcaResultRegs.ADCRESULT9 ) ;
    //        DSP.iuvw_offset_online[5] += (REAL)(AdcaResultRegs.ADCRESULT8 ) ;
    //        if(DSP.offset_counter>=5000){
    //            DSP.iuvw_offset_online[0] = DSP.iuvw_offset_online[0] / 5000;
    //            DSP.iuvw_offset_online[1] = DSP.iuvw_offset_online[1] / 5000;
    //            DSP.iuvw_offset_online[2] = DSP.iuvw_offset_online[2] / 5000;
    //            DSP.iuvw_offset_online[3] = DSP.iuvw_offset_online[3] / 5000;
    //            DSP.iuvw_offset_online[4] = DSP.iuvw_offset_online[4] / 5000;
    //            DSP.iuvw_offset_online[5] = DSP.iuvw_offset_online[5] / 5000;
    //            DSP.AD_offset_flag2 = TRUE;
    //            DSP.offset_counter = 0;
    //        }
    //
    //        // 来不及完成偏置检测（比如刚上电数字开关就是开的），采用默认值
    //        /* 427-1401：添加开关信号滤波。今天发现在刚上电的时候，XCUBE-II的前两个中断里，数字开关是打开的，然后才变成关闭。*/
    //        if(DSP.FLAG_ENABLE_PWM_OUTPUT && DSP.offset_counter>100){
    //            DSP.iuvw_offset_online[0] = 0.0;
    //            DSP.iuvw_offset_online[1] = 0.0;
    //            DSP.iuvw_offset_online[2] = 0.0;
    //            DSP.iuvw_offset_online[3] = 0.0;
    //            DSP.iuvw_offset_online[4] = 0.0;
    //            DSP.iuvw_offset_online[5] = 0.0;
    //            DSP.AD_offset_flag2 = TRUE;
    //        }
    //
    //        // 上电的时候，电机可能在转，此时根据电流判断是否还要额外进行偏置补偿。
    //        if( fabs(DSP.iuvw[0])>0.05 || fabs(DSP.iuvw[1])>0.05 || fabs(DSP.iuvw[2])>0.05 || \
    //            fabs(DSP.iuvw[3])>0.05 || fabs(DSP.iuvw[4])>0.05 || fabs(DSP.iuvw[5])>0.05){
    //            DSP.iuvw_offset_online[0] = 0.0;
    //            DSP.iuvw_offset_online[1] = 0.0;
    //            DSP.iuvw_offset_online[2] = 0.0;
    //            DSP.iuvw_offset_online[3] = 0.0;
    //            DSP.iuvw_offset_online[4] = 0.0;
    //            DSP.iuvw_offset_online[5] = 0.0;
    //            DSP.AD_offset_flag2 = TRUE;
    //        }
    //    }
}



void PanGuMainISR(void){
    
    // Measurement
    // 采样，包括DSP中的ADC采样等 // DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?
    main_measurement(); // 电流传感器和编码器测得三相电流iuvw和角度theta，iuvw通过clark变化得到iabg，后面iabg通过park变换得到idq，idq通过dq变换得到abc

    // PWM Register Update
    if (!DSP.FLAG_ENABLE_PWM_OUTPUT){
        DSP_PWM1_DISABLE
        DSP_PWM2_DISABLE
        DSP_PWM3_DISABLE
        DSP_PWM4_DISABLE
        user_routine_disable_pwm_output(); // 用户自定义的关闭PWM输出函数
    }else{
        user_routine_enable_pwm_output(); // 用户自定义的开启PWM输出函数
        int pwm_test_mode = user_routine_main_switch();

        /* PWM signal to Inverter Voltage Output SWPWM */
        if (DSP.epwm_enable[0] == TRUE){
            DSP_PWM1_ENABLE
            EPwm1Regs.CMPA.bit.CMPA = Axes[0].svgen.Ta * SYSTEM_TBPRD; // 0-5000，5000表示0%的占空比
            EPwm2Regs.CMPA.bit.CMPA = Axes[0].svgen.Tb * SYSTEM_TBPRD;
            EPwm3Regs.CMPA.bit.CMPA = Axes[0].svgen.Tc * SYSTEM_TBPRD;
        }else{
            DSP_PWM1_DISABLE
        }
        if (DSP.epwm_enable[1] == TRUE){
            DSP_PWM2_ENABLE
            EPwm4Regs.CMPA.bit.CMPA = Axes[1].svgen.Ta * SYSTEM_TBPRD; // 0-5000，5000表示0%的占空比
            EPwm5Regs.CMPA.bit.CMPA = Axes[1].svgen.Tb * SYSTEM_TBPRD;
            EPwm6Regs.CMPA.bit.CMPA = Axes[1].svgen.Tc * SYSTEM_TBPRD;
        }else{
            DSP_PWM2_DISABLE
        }
        if (DSP.epwm_enable[2] == TRUE){
            DSP_PWM3_ENABLE
            EPwm7Regs.CMPA.bit.CMPA = Axes[2].svgen.Ta * SYSTEM_TBPRD; // 0-5000，5000表示0%的占空比
            EPwm8Regs.CMPA.bit.CMPA = Axes[2].svgen.Tb * SYSTEM_TBPRD;
            EPwm9Regs.CMPA.bit.CMPA = Axes[2].svgen.Tc * SYSTEM_TBPRD;
        }else{
            DSP_PWM3_DISABLE
        }
        if (DSP.epwm_enable[3] == TRUE){
            DSP_PWM4_ENABLE
            EPwm10Regs.CMPA.bit.CMPA = Axes[3].svgen.Ta * SYSTEM_TBPRD; // 0-5000，5000表示0%的占空比
            EPwm11Regs.CMPA.bit.CMPA = Axes[3].svgen.Tb * SYSTEM_TBPRD;
            EPwm12Regs.CMPA.bit.CMPA = Axes[3].svgen.Tc * SYSTEM_TBPRD;
        }else{
            DSP_PWM4_DISABLE
        }
    }
}
Uint64 EPWM1IntCount = 0;
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
        PieCtrlRegs.PIEACK.all = 0xFFFF; // Enable PIE interrupts by writing all 1’s to the PIEACK register
        asm("       NOP");               // Wait at least one cycle
        EINT;                            // Enable global interrupts by clearing INTM
    #endif

    if (DSP.trip_flag == TRUE){
        // TODO：报警，比如点亮LED
    }else{
        if (GpioDataRegs.GPCDAT.bit.GPIO75 == 1) DSP.FLAG_ENABLE_PWM_OUTPUT = 1; else DSP.FLAG_ENABLE_PWM_OUTPUT = 0;
    }

    /* Step 4. [ePWM] Execute EPWM ISR */
    #if ENABLE_ECAP
        do_enhanced_capture();
    #endif

    // 主中断函数
    PanGuMainISR(); // measurement + pwm register update

    #if NUMBER_OF_DSP_CORES == 2
        write_DAC_buffer();
    #endif

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


void init_experiment_AD_gain_and_offset(){ 
    // 母线电压*4
    DSP.adc_vdc_scale[0]  = USER_INVERTER1_SCALE_VDC_BUS_IPM;
    DSP.adc_vdc_scale[1]  = USER_INVERTER2_SCALE_VDC_BUS_IPM;
    DSP.adc_vdc_scale[2]  = USER_INVERTER3_SCALE_VDC_BUS_IPM;
    DSP.adc_vdc_scale[3]  = USER_INVERTER4_SCALE_VDC_BUS_IPM;
    DSP.adc_vdc_offset[0] = USER_INVERTER1_OFFSET_VDC_BUS_IPM;
    DSP.adc_vdc_offset[1] = USER_INVERTER2_OFFSET_VDC_BUS_IPM;
    DSP.adc_vdc_offset[2] = USER_INVERTER3_OFFSET_VDC_BUS_IPM;
    DSP.adc_vdc_offset[3] = USER_INVERTER4_OFFSET_VDC_BUS_IPM;
    // 电流*12
    DSP.adc_iuvw_scale[0] = USER_SCALE_LEM_A1;
    DSP.adc_iuvw_scale[1] = USER_SCALE_LEM_A2;
    DSP.adc_iuvw_scale[2] = USER_SCALE_LEM_A3;
    DSP.adc_iuvw_scale[3] = USER_SCALE_LEM_B7;
    DSP.adc_iuvw_scale[4] = USER_SCALE_LEM_B8;
    DSP.adc_iuvw_scale[5] = USER_SCALE_LEM_B9;
    DSP.adc_iuvw_scale[6] = 0.0;
    DSP.adc_iuvw_scale[7] = 0.0;
    DSP.adc_iuvw_scale[8] = 0.0;
    DSP.adc_iuvw_scale[9] = 0.0;
    DSP.adc_iuvw_scale[10] = 0.0;
    DSP.adc_iuvw_scale[11] = 0.0;
    DSP.adc_iuvw_offset[0] = USER_OFFSET_LEM_A1;
    DSP.adc_iuvw_offset[1] = USER_OFFSET_LEM_A2;
    DSP.adc_iuvw_offset[2] = USER_OFFSET_LEM_A3;
    DSP.adc_iuvw_offset[3] = USER_OFFSET_LEM_B7;
    DSP.adc_iuvw_offset[4] = USER_OFFSET_LEM_B8;
    DSP.adc_iuvw_offset[5] = USER_OFFSET_LEM_B9;
    DSP.adc_iuvw_offset[6] = 0.0;
    DSP.adc_iuvw_offset[7] = 0.0;
    DSP.adc_iuvw_offset[8] = 0.0;
    DSP.adc_iuvw_offset[9] = 0.0;
    DSP.adc_iuvw_offset[10] = 0.0;
    DSP.adc_iuvw_offset[11] = 0.0;
    // 编码器*4
    DSP.OffsetCountBetweenIndexAndUPhaseAxis[0] = USER_MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
    DSP.OffsetCountBetweenIndexAndUPhaseAxis[1] = USER_MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
    DSP.OffsetCountBetweenIndexAndUPhaseAxis[2] = USER_MOTOR3_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
    DSP.OffsetCountBetweenIndexAndUPhaseAxis[3] = USER_MOTOR4_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
}

void init_spi_sci_can() {

    // https://www.ti.com/lit/ug/spruhm8j/spruhm8j.pdf?ts=1710763002632&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMS320F28377D%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Depd-c2x-null-44700045336317350_prodfolderdynamic-cpc-pf-google-wwe_int%2526utm_content%253Dprodfolddynamic%2526ds_k%253DDYNAMIC+SEARCH+ADS%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwzN-vBhAkEiwAYiO7oBh2RIh8aEZiiEHzGLlyxsdf34XwLyRi-Ci53QGwm9calxyVXTdsEBoC7IwQAvD_BwE%2526gclsrc%253Daw.ds
    // Chapter 8.7.1

    // 陈嘉豪：设备管理器（把外设的控制权交给CPU2）
    EALLOW;
        DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
        DevCfgRegs.CPUSEL6.bit.SPI_C = 1; // assign spi-c to cpu2

        DevCfgRegs.CPUSEL5.bit.SCI_A = 1; // assign sci-a to cpu2
        DevCfgRegs.CPUSEL5.bit.SCI_B = 1; // assign sci-b to cpu2
        DevCfgRegs.CPUSEL5.bit.SCI_C = 1;

        DevCfgRegs.CPUSEL8.bit.CAN_A = 1; // assign can-a to cpu2
        DevCfgRegs.CPUSEL8.bit.CAN_B = 1; // assign can-b to cpu2

        // Allows CPU01 bootrom to take control of clock
        // configuration registers
        ClkCfgRegs.CLKSEM.all = 0xA5A50000;
        ClkCfgRegs.LOSPCP.all = 0x0001; // LSPCLK=100MHz table 3-173
    EDIS;

    // 陈嘉豪：初始化通用输入输出引脚
    // 壹、初始化高速SPI，用于与DAC芯片MAX5307通讯。
        InitHighSpeedSpiGpio();

    // 贰、初始化CAN
        //CANA RX TX
        GPIO_SetupPinMux(62, GPIO_MUX_CPU2, 6);
        GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(19, GPIO_MUX_CPU2, 3);
        GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
        //CANB RX TX
        GPIO_SetupPinMux(21, GPIO_MUX_CPU2, 3);
        GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(20, GPIO_MUX_CPU2, 3);
        GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);

    // 叁、初始化SCI-485
        // SCIA RX TX WE (Writing Enable)
        GPIO_SetupPinMux(PIN_SCI_RXDA, GPIO_MUX_CPU2, MUX_SCI_RXDA);
        GPIO_SetupPinOptions(PIN_SCI_RXDA, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(PIN_SCI_TXDA, GPIO_MUX_CPU2, MUX_SCI_TXDA);
        GPIO_SetupPinOptions(PIN_SCI_TXDA, GPIO_OUTPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux    (PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_MUX_CPU2, 0); // GPIO139 - 485-SCIA-WE-(use SCICRX as GPIO, in UART3 pin8)
        GPIO_SetupPinOptions(PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_OUTPUT, GPIO_ASYNC);

        // SCIB RX TX WE (Writing Enable)
        GPIO_SetupPinMux(PIN_SCI_RXDB, GPIO_MUX_CPU2, MUX_SCI_RXDB);
        GPIO_SetupPinOptions(PIN_SCI_RXDB, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(PIN_SCI_TXDB, GPIO_MUX_CPU2, MUX_SCI_TXDB);
        GPIO_SetupPinOptions(PIN_SCI_TXDB, GPIO_OUTPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux    (PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_MUX_CPU2, 0); // GPIO140 - 485-SCIB-WE-(use SCICTX as GPIO, in UART3 pin7)
        GPIO_SetupPinOptions(PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_OUTPUT, GPIO_ASYNC);

        // SCIC RX TX (not used?)
        // GPIO_SetupPinMux(PIN_SCI_RXDC, GPIO_MUX_CPU2, MUX_SCI_RXDC);
        // GPIO_SetupPinOptions(PIN_SCI_RXDC, GPIO_INPUT, GPIO_PUSHPULL);
        // GPIO_SetupPinMux(PIN_SCI_TXDC, GPIO_MUX_CPU2, MUX_SCI_TXDC);
        // GPIO_SetupPinOptions(PIN_SCI_TXDC, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void handle_interrupts() {
        /* Re-map PIE Vector Table to user defined ISR functions. */
        EALLOW;                                   // This is needed to write to EALLOW protected registers
        PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM; //&MainISR;      // PWM主中断 10kKHz
    #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
        PieVectTable.ECAP1_INT = &ecap1_isr;
        PieVectTable.ECAP2_INT = &ecap2_isr;
        PieVectTable.ECAP3_INT = &ecap3_isr;
    #endif
    #if SYSTEM_PROGRAM_MODE != 223
        PieVectTable.EQEP1_INT = &EQEP_UTO_INT; // eQEP
    #endif
    #if NUMBER_OF_DSP_CORES == 1
    // PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;   // SCI Receive (Part 1/3)
    // PieVectTable.SCIC_TX_INT = &scicTxFifoIsr;   // SCI Transmit
    #endif
        EDIS; // This is needed to disable write to EALLOW protected registers
            /* PIE Control */
        /* ePWM */
        PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // PWM1 interrupt (Interrupt 3.1)
    /* SCI */
    #if NUMBER_OF_DSP_CORES == 1
                                        // PieCtrlRegs.PIEIER8.bit.INTx5 = 1;   // PIE Group 8, INT5, SCI-C Rx (Part 2/3)
                                        // PieCtrlRegs.PIEIER8.bit.INTx6 = 1;   // PIE Group 8, INT6, SCI-C Tx
    #endif
    /* eCAP */
    #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
        PieCtrlRegs.PIEIER4.bit.INTx1 = 1; // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
        PieCtrlRegs.PIEIER4.bit.INTx2 = 1; // 1 Enable for Interrupt 4.2
        PieCtrlRegs.PIEIER4.bit.INTx3 = 1; // 2 Enable for Interrupt 4.3
    #endif
    /* eQEP */
    #if SYSTEM_PROGRAM_MODE != 223
        PieCtrlRegs.PIEIER5.bit.INTx1 = 1; // QEP interrupt
    #endif
        /* CPU Interrupt Enable Register (IER) */
        IER |= M_INT3; // EPWM1_INT
    #if NUMBER_OF_DSP_CORES == 1
                    // IER |= M_INT8; // SCI-C (Part 3/3)
    #endif
    #if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
        IER |= M_INT4; // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
    #endif
    #if SYSTEM_PROGRAM_MODE != 223 // used by yuanxin
        IER |= M_INT5;             // EQEP1_INT???
    #endif
        EINT; // Enable Global __interrupt INTM
        ERTM; // Enable Global realtime __interrupt DBGM
}



void test_ipc_tocpu02(){
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
}

void voltage_measurement_based_on_eCAP()
{
    CAP.terminal_voltage[0] = (CAP.terminal_DutyOnRatio[0]) * DSP.vdc - DSP.vdc * 0.5; // -0.5 is due to duty ratio calculation; - vdc * 0.5 is referring to the center of dc bus capacitor.
    CAP.terminal_voltage[1] = (CAP.terminal_DutyOnRatio[1]) * DSP.vdc - DSP.vdc * 0.5;
    CAP.terminal_voltage[2] = (CAP.terminal_DutyOnRatio[2]) * DSP.vdc - DSP.vdc * 0.5;

    CAP.line_to_line_voltage[0] = CAP.terminal_voltage[0] - CAP.terminal_voltage[1];
    CAP.line_to_line_voltage[1] = CAP.terminal_voltage[1] - CAP.terminal_voltage[2];
    CAP.line_to_line_voltage[2] = CAP.terminal_voltage[2] - CAP.terminal_voltage[0];

    if (CAP.flag_bad_U_capture == FALSE && CAP.flag_bad_V_capture == FALSE && CAP.flag_bad_W_capture == FALSE)
    {
        // Use ecap feedback
        CAP.uab0[0] = 0.33333 * (2 * CAP.terminal_voltage[0] - CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[1] = 0.57735 * (CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[2] = 0.33333 * (CAP.terminal_voltage[0] + CAP.terminal_voltage[1] + CAP.terminal_voltage[2]);
        CAP.dq[0] = (*CTRL).s->cosT * CAP.uab0[0] + (*CTRL).s->sinT * CAP.uab0[1];
        CAP.dq[1] = -(*CTRL).s->sinT * CAP.uab0[0] + (*CTRL).s->cosT * CAP.uab0[1];
    }
    else
    {
        // Assume the voltage vector is rtoating at a constant speed when ecap measurement is disturbed.
        CAP.uab0[0] = (*CTRL).s->cosT * CAP.dq[0] - (*CTRL).s->sinT * CAP.dq[1];
        CAP.uab0[1] = (*CTRL).s->sinT * CAP.dq[0] + (*CTRL).s->cosT * CAP.dq[1];
    }

    // 电压测量
    if (G.flag_use_ecap_voltage == 2 || G.flag_use_ecap_voltage == 1)
    {
        /*Use original ecap measured voltage*/
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];
    }
    //    else if(G.flag_use_ecap_voltage==3){
    //        ecap_moving_average();
    //    }
    if (G.flag_use_ecap_voltage == 10)
    {
        /*Use lpf ecap measured voltage*/
        CAP.dq_lpf[0] = _lpf(CAP.dq[0], CAP.dq_lpf[0], 800);
        CAP.dq_lpf[1] = _lpf(CAP.dq[1], CAP.dq_lpf[1], 800);
        CAP.uab0[0] = (*CTRL).s->cosT * CAP.dq_lpf[0] - (*CTRL).s->sinT * CAP.dq_lpf[1];
        CAP.uab0[1] = (*CTRL).s->sinT * CAP.dq_lpf[0] + (*CTRL).s->cosT * CAP.dq_lpf[1];
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];
    }
    else if (G.flag_use_ecap_voltage == 0)
    {
        /*Use command voltage for feedback*/
        US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
        US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
        US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
    }

    // (for watch only) Mismatch between ecap measurement and command to inverter
    (*CTRL).o->cmd_uDQ_to_inverter[0] = (*CTRL).s->cosT * (*CTRL).o->cmd_uAB_to_inverter[0] + (*CTRL).s->sinT * (*CTRL).o->cmd_uAB_to_inverter[1];
    (*CTRL).o->cmd_uDQ_to_inverter[1] = -(*CTRL).s->sinT * (*CTRL).o->cmd_uAB_to_inverter[0] + (*CTRL).s->cosT * (*CTRL).o->cmd_uAB_to_inverter[1];
    CAP.dq_mismatch[0] = (*CTRL).o->cmd_uDQ_to_inverter[0] - CAP.dq[0];
    CAP.dq_mismatch[1] = (*CTRL).o->cmd_uDQ_to_inverter[1] - CAP.dq[1];
}
