#include "All_Definition.h"
st_axis Axis_1, *Axis;
extern bool run_enable_from_PC;

#if NUMBER_OF_AXES == 2 // ====为了同时运行两台电机，增加的另一份控制结构体
    st_axis Axis_2;
    #define get_Axis_CTRL_pointers \
        if (axisCnt == 0)          \
        {                          \
            Axis  = &Axis_1;       \
            CTRL  = &CTRL_1;       \
            debug = &debug_1;      \
        }                          \
        if (axisCnt == 1)          \
        {                          \
            Axis  = &Axis_2;       \
            CTRL  = &CTRL_2;       \
            debug = &debug_2;      \
        }                           
#endif
void main(void){

    InitSysCtrl();      // 1. Initialize System Control: PLL, WatchDog, enable Peripheral Clocks.
    Gpio_initialize();  // 2. Initialize GPIO and assign GPIO to peripherals.
    DINT;               // 3.1 Clear all interrupts and initialize PIE vector table.
    InitPieCtrl();      // 3.2 Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;       // 3.3 Disable CPU __interrupts,
    IFR = 0x0000;       // 3.4 and clear all CPU __interrupt flags.a
    InitPieVectTable(); // 3.5 Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). At end, ENPIE = 1.
    InitCpuTimers();     // for Slessinv TIE.R1 for measuring the execution time
    ConfigCpuTimer(&CpuTimer1, 200, 1000000); // 200MHz, INTERRUPT_period = 1e6 us
    #if NUMBER_OF_DSP_CORES == 2
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);     // 4.1 IPC
    #endif
    #ifdef _STANDALONE
        #ifdef _FLASH
            // use this code when u need to cut off the power and reboot it offline
            //  Send boot command to allow the CPU02 application to begin execution
            IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
        #else
            //  Send boot command to allow the CPU02 application to begin execution
            // seriesly i dont know what does this sentences mean, do no change it might as well
            // i dont know what does sentence
            // IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
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
        // Synchronous !!!!!!!!
        // 
        InitHighSpeedSpiGpio();
        // InitSpiaGpio();
        InitSpi();
        InitSciGpio();
        InitSci();
    #elif NUMBER_OF_DSP_CORES == 2
        /* 双核配置*/
        init_spi();
        EUREKA_GPIO_SETUP();
        //        // =========TEST BOARD PIN============
        //        // =========NOT FOR EUREKA===========

        // 异步！！！！！
        // InitSci(); // this is moved to CPU02

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
    /* All this init operation should be same as emy-c WUBO*/
    init_d_sim();      // do this only once here
    init_debug();      // do this only once here
    init_experiment(); // 控制器结构体初始化（同实验）
    get_bezier_points(); // for testing Cury the leg trajectgory tracking 
    for (axisCnt = 0; axisCnt < NUMBER_OF_AXES; axisCnt++){
        get_Axis_CTRL_pointers
        // (*debug).bool_initilized = FALSE;
        // _user_init();      // debug initilization for user 20240929 is the operation is moved outside of for loop
        axis_basic_setup(axisCnt); // 根据axiscnt对Axis，CTRL的1和2号结构体，进行初始化操作
    }
    axisCnt = 1;

    // 5. Handle Interrupts
    handle_interrupts();

    // 6. Pre main loop
    DSP_STOP_LED1
    DSP_STOP_LED2
    DSP_PWM_DISABLE
    DSP_2PWM_DISABLE
    /* Test IPC to CPU02 */
    #if NUMBER_OF_DSP_CORES == 2
        test_ipc_tocpu02();
    #endif

    // 7. CLA
    // cla_test_codes();

    // 8. Main loop
    main_loop();
}

void main_loop(){
    while (1){
        #if WHO_IS_USER == USER_BEZIER
            if(d_sim.user.bezier_NUMBER_OF_STEPS<8000){
                bezier_controller_run_in_main();
            }
        #endif

        //        mainWhileLoopCounter1++;
        //        mainWhileLoopCounter2=2992;
        //        if (Motor_mode_START==1){
        //            Axis_1.FLAG_ENABLE_PWM_OUTPUT = 1;
        //            DSP_START_LED1
        //            DSP_START_LED2
        //            mainWhileLoopCounter1 = Axis_1.FLAG_ENABLE_PWM_OUTPUT +5;
        //        }else if (Motor_mode_START==0){
        //            Axis_1.FLAG_ENABLE_PWM_OUTPUT = 0;
        //            DSP_STOP_LED1
        //            DSP_STOP_LED2
        //            mainWhileLoopCounter1 = Axis_1.FLAG_ENABLE_PWM_OUTPUT +5;
        //        }
        //        mainWhileLoopCounter1 = mainWhileLoopCounter2;

        //        mainWhileLoopCounter3 += 1;
        //        Axis_1.ID += 1;
        //        mainWhileLoopCounter2 += 1;

        #if NUMBER_OF_DSP_CORES == 1
            single_core_dac();
        #endif
    }
}


/* Below is moved from PanGuMainISR.c */

// extern long long sci_pos;

void main_measurement(){

    #if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
        PostionSpeedMeasurement_MovingAvergage(EQep1Regs.QPOSCNT, CTRL->enc);
    #else
        // read encoder counter data from absolute encoder
        CTRL->enc->encoder_abs_cnt_previous = CTRL->enc->encoder_abs_cnt;
        if (axisCnt == 0) measurement_position_count_axisCnt0();
        if (axisCnt == 1) measurement_position_count_axisCnt1();

        // measure d-axis angle and then use a poor algorithm to calculate speed
        measurement_enc();
    #endif
    CTRL->i->varOmega     = CTRL->enc->varOmega;
    CTRL->i->theta_d_elec = CTRL->enc->theta_d_elec;

    // measure current
    if (axisCnt == 0) measurement_current_axisCnt0();
    if (axisCnt == 1) measurement_current_axisCnt1();

    // 只用第一套三相
    (*CTRL).i->iAB[0] = Axis->iabg[0];
    (*CTRL).i->iAB[1] = Axis->iabg[1];

    // 观测器专用变量（与你无关，别管啦！）
    # if( WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        IS_C(0) = Axis->iabg[0];
        IS_C(1) = Axis->iabg[1];
        US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
    #endif

    // 线电压（占空比）测量（基于占空比和母线电压）
    // voltage_measurement_based_on_eCAP();

    // measure vdc
    Axis->vdc    =((REAL)(AdcaResultRegs.ADCRESULT0 ) - Axis->adc_offset[0]) * Axis->adc_scale[0];
    // Axis->vdc = ((REAL)(AdcbResultRegs.ADCRESULT6) - Axis->adc_offset[0]) * Axis->adc_scale[0];
    if (G.flag_overwite_vdc) Axis->vdc = G.overwrite_vdc;
    {
        // Vdc用于实时更新电流环限幅
        PID_iD->OutLimit = Axis->vdc * 0.5773672 * (*debug).LIMIT_DC_BUS_UTILIZATION;
        PID_iQ->OutLimit = Axis->vdc * 0.5773672 * (*debug).LIMIT_DC_BUS_UTILIZATION;
        PID_Speed->OutLimit = (*debug).LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;


        // 电流环输出限幅2V
        // PID_iD->OutLimit = 2;
        // PID_iQ->OutLimit = 2; 
        // PID_iX->outLimit = Axis->vdc * 0.5773672;
        // PID_iY->outLimit = Axis->vdc * 0.5773672;
    }

    //    这样不能形成保护，必须设置故障状态才行。
    //    if(fabs(G.Current_W)>8 || fabs(G.Current_V)>8){
    //        DSP_PWM_DISABLE
    //        DSP_2PWM_DISABLE
    //    }

    // 电流采样ADC温飘校准 // TODO 改成用ADC Raw Results校准。
    //    if(Axis->AD_offset_flag2==FALSE)
    //    {
    //        Axis->offset_counter += 1;
    //        Axis->iuvw_offset_online[0] += (REAL)(AdcaResultRegs.ADCRESULT1 ) ;
    //        Axis->iuvw_offset_online[1] += (REAL)(AdcaResultRegs.ADCRESULT2 ) ;
    //        Axis->iuvw_offset_online[2] += (REAL)(AdcaResultRegs.ADCRESULT3 ) ;
    //        Axis->iuvw_offset_online[3] += (REAL)(AdcaResultRegs.ADCRESULT11 ) ;
    //        Axis->iuvw_offset_online[4] += (REAL)(AdcaResultRegs.ADCRESULT9 ) ;
    //        Axis->iuvw_offset_online[5] += (REAL)(AdcaResultRegs.ADCRESULT8 ) ;
    //        if(Axis->offset_counter>=5000){
    //            Axis->iuvw_offset_online[0] = Axis->iuvw_offset_online[0] / 5000;
    //            Axis->iuvw_offset_online[1] = Axis->iuvw_offset_online[1] / 5000;
    //            Axis->iuvw_offset_online[2] = Axis->iuvw_offset_online[2] / 5000;
    //            Axis->iuvw_offset_online[3] = Axis->iuvw_offset_online[3] / 5000;
    //            Axis->iuvw_offset_online[4] = Axis->iuvw_offset_online[4] / 5000;
    //            Axis->iuvw_offset_online[5] = Axis->iuvw_offset_online[5] / 5000;
    //            Axis->AD_offset_flag2 = TRUE;
    //            Axis->offset_counter = 0;
    //        }
    //
    //        // 来不及完成偏置检测（比如刚上电数字开关就是开的），采用默认值
    //        /* 427-1401：添加开关信号滤波。今天发现在刚上电的时候，XCUBE-II的前两个中断里，数字开关是打开的，然后才变成关闭。*/
    //        if(Axis->FLAG_ENABLE_PWM_OUTPUT && Axis->offset_counter>100){
    //            Axis->iuvw_offset_online[0] = 0.0;
    //            Axis->iuvw_offset_online[1] = 0.0;
    //            Axis->iuvw_offset_online[2] = 0.0;
    //            Axis->iuvw_offset_online[3] = 0.0;
    //            Axis->iuvw_offset_online[4] = 0.0;
    //            Axis->iuvw_offset_online[5] = 0.0;
    //            Axis->AD_offset_flag2 = TRUE;
    //        }
    //
    //        // 上电的时候，电机可能在转，此时根据电流判断是否还要额外进行偏置补偿。
    //        if( fabs(Axis->iuvw[0])>0.05 || fabs(Axis->iuvw[1])>0.05 || fabs(Axis->iuvw[2])>0.05 || \
    //            fabs(Axis->iuvw[3])>0.05 || fabs(Axis->iuvw[4])>0.05 || fabs(Axis->iuvw[5])>0.05){
    //            Axis->iuvw_offset_online[0] = 0.0;
    //            Axis->iuvw_offset_online[1] = 0.0;
    //            Axis->iuvw_offset_online[2] = 0.0;
    //            Axis->iuvw_offset_online[3] = 0.0;
    //            Axis->iuvw_offset_online[4] = 0.0;
    //            Axis->iuvw_offset_online[5] = 0.0;
    //            Axis->AD_offset_flag2 = TRUE;
    //        }
    //    }
}

// int down_freq_ecap_counter = 1;
// Uint64 timebase_counter = 0;
extern REAL imife_realtime_gain_off;

//REAL wubo_debug_flag_PWM = 0;
//REAL wubo_debug_motor_enc_dirc[2] = {1.0, -1.0};



void DISABLE_PWM_OUTPUT(){
    DSP_PWM_DISABLE
    DSP_2PWM_DISABLE

    /* Only init once for easy debug */
    /* wubo：每次执行ENABLE PWM OUT时候会将下面的flag置为0 从而每次关闭开启开关都会执行下面的操作！上面说的有问题？*/
    if (!G.flag_experimental_initialized){
        G.flag_experimental_initialized = TRUE;

        init_experiment();
        // init_experiment_AD_gain_and_offset();
        // init_experiment_overwrite();

        // TODO: use a function for this purpose!
        // 清空积分缓存
        PID_Speed->OutPrev = 0;
        PID_iD->OutPrev = 0;
        PID_iQ->OutPrev = 0;

        // 清空速度InnerLoop缓存
//        PID_Speed->KFB_Term_Prev = 0;
        #if WHO_IS_USER == USER_WB
            SIL_Controller.KFB_Term = 0;
        #endif
        // PID_iX->OutPrev = 0;
        // PID_iy->OutPrev = 0;

        EPwm1Regs.CMPA.bit.CMPA = 2500;
        EPwm2Regs.CMPA.bit.CMPA = 2500;
        EPwm3Regs.CMPA.bit.CMPA = 2500;
        EPwm4Regs.CMPA.bit.CMPA = 2500;
        EPwm5Regs.CMPA.bit.CMPA = 2500;
        EPwm6Regs.CMPA.bit.CMPA = 2500;

        //            CTRL_2.s->iD  = &_pid_iD_2;
        //            CTRL_2.s->iQ  = &_pid_iQ_2;
        //            CTRL_2.s->spd = &_pid_spd_2;
        //            CTRL_2.s->pos = &_pid_pos_2;

        if ((*CTRL).g->overwrite_vdc < 5)
        {
            (*CTRL).g->overwrite_vdc = 28;
        }
        (*CTRL).g->flag_overwite_vdc = 0;

        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        #if WHO_IS_USER == USER_WB
            (*CTRL).timebase_counter = 0;
            (*CTRL).timebase = 0.0;
            (*debug).CMD_CURRENT_SINE_AMPERE      = d_sim.user.CMD_CURRENT_SINE_AMPERE;
            (*debug).CMD_SPEED_SINE_RPM           = d_sim.user.CMD_SPEED_SINE_RPM;
            (*debug).CMD_SPEED_SINE_HZ            = d_sim.user.CMD_SPEED_SINE_HZ;
            (*debug).CMD_SPEED_SINE_STEP_SIZE     = d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
            (*debug).CMD_SPEED_SINE_LAST_END_TIME = d_sim.user.CMD_SPEED_SINE_LAST_END_TIME;
            (*debug).CMD_SPEED_SINE_END_TIME      = d_sim.user.CMD_SPEED_SINE_END_TIME;
            (*debug).CMD_SPEED_SINE_HZ_CEILING    = d_sim.user.CMD_SPEED_SINE_HZ_CEILING;
        #endif
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
    }

    /* 在不输出PWM波形的时候，也就是“开关”为OFF的时候，更新SpeedInnerLoop的参数*/
    #if WUBO_ONLINE_TUNING
        #if WHO_IS_USER == USER_WB
            if ( (*debug).who_is_user == USER_WB && (*debug).bool_apply_WC_tunner_for_speed_loop == TRUE ){
                _user_wubo_WC_Tuner_Online(); // 和_user_wubo_WC_Tuner函数区别
            }else{
                _user_wubo_TI_Tuner_Online();
            }
        #else
            //TODO: Here needs a online tuning function for public use
        #endif
    #endif

    DELAY_US(5);
    GpioDataRegs.GPDCLEAR.bit.GPIO106 = 1; // TODO: What is this doing?
}

void ENABLE_PWM_OUTPUT(int positionLoopType){
    G.flag_experimental_initialized = FALSE;
    if (use_first_set_three_phase == 1){
        DSP_PWM_ENABLE
        // 第一套逆变器工作的时候，确保第二套逆变器关闭
        EPwm4Regs.CMPA.bit.CMPA = 2500;
        EPwm5Regs.CMPA.bit.CMPA = 2500;
        EPwm6Regs.CMPA.bit.CMPA = 2500;
    } else if (use_first_set_three_phase == 2){
        DSP_2PWM_ENABLE
        // 同上
        EPwm1Regs.CMPA.bit.CMPA = 2500;
        EPwm2Regs.CMPA.bit.CMPA = 2500;
        EPwm3Regs.CMPA.bit.CMPA = 2500;
    }else if (use_first_set_three_phase == -1){
        DSP_PWM_ENABLE
        DSP_2PWM_ENABLE
    }
    #if WHO_IS_USER == USER_YZZ

        if (FE.htz.u_offset[0] > 0.1)
        {
            FE.htz.u_offset[0] = 0;
        }
    #endif
    // DSP中控制器的时间
    (*CTRL).timebase_counter += 1;
    (*CTRL).timebase = CL_TS * (*CTRL).timebase_counter; //(*CTRL).timebase += CL_TS; // 2048 = float/REAL max

    #if WHO_IS_USER != USER_XM
        // 根据指令，产生控制输出（电压）
                                                                //(*CTRL).s->Motor_or_Gnerator = sign((*CTRL).i->cmd_iDQ[1]) == sign(CTRL->enc->rpm); // sign((*CTRL).i->cmd_iDQ[1]) != sign((*CTRL).i->cmd_speed_rpm))
        //        runtime_command_and_tuning(Axis->Select_exp_operation);
        //        // 0x03 is shank
        //        //    position_count_CAN_fromCPU2 = position_count_CAN_ID0x03_fromCPU2;
        //        // 0x01 is hip
        //        // position_count_CAN_fromCPU2 = position_count_CAN_ID0x01_fromCPU2;
        //
        //        if (positionLoopType == 0){
        //            // do nothing
        //        }
        //        else{
        //            // do position loop
        //            Axis->Set_manual_rpm = call_position_loop_controller(positionLoopType);
        //        }
        //
        //        if (flag_RPM_wave == 1)
        //        {
        //            Axis->Set_manual_rpm = (*CTRL).timebase * 20;
        //            if ( (*CTRL).timebase * 20 > 400)
        //            {
        //                Axis->Set_manual_rpm = 400;
        //            }
        //        }

        //        if(IPCRtoLFlagBusy(IPC_FLAG8) == 1){
        //            iq_command_from_PC = Read.current_cmd_from_PC;
        //            IPCRtoLFlagAcknowledge(IPC_FLAG8);
        //        }
        //        if(run_enable_from_PC == false){
        //            iq_command_from_PC = 0.0;
        //        }
    #endif

    /* MAIN SWITCH */
    /* MAIN SWITCH */
    /* MAIN SWITCH */
    Axis->Select_exp_operation = main_switch(debug->mode_select);

    //(*CTRL).o->cmd_uAB_to_inverter[0]

    // operation mode为5的时候，执行下面测试逆变器输出电压的代码
    if (Axis->Select_exp_operation == XCUBE_TaTbTc_DEBUG_MODE){
        if (axisCnt == 0){
            EPwm1Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Ta * 50000000 * CL_TS; // 0-5000，5000表示0%的占空比
            EPwm2Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Tb * 50000000 * CL_TS;
            EPwm3Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Tc * 50000000 * CL_TS;
        }
        if (axisCnt == 1){
            EPwm4Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Ta * 50000000 * CL_TS;
            EPwm5Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Tb * 50000000 * CL_TS;
            EPwm6Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Tc * 50000000 * CL_TS;
        }
    }
    else{ // 否则根据上面的控制率controller()由voltage_commands_to_pwm()计算出的电压，输出到逆变器
        voltage_commands_to_pwm();
    }
}


// 20240720前的有一个bug：我们只通过Axis_1.FLAG_ENABLE_PWM_OUTPUT来决定PWM信号是否输出，但是对应的PWM开通关断函数下，是无脑地对
// 两个逆变器的PWM信号同时进行了开关，这样会导致两个逆变器的PWM信号都输出。
void PanGuMainISR(void){
    // 采样，包括DSP中的ADC采样等
    // DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?
    main_measurement(); // 电流传感器和编码器测得三相电流iuvw和角度theta，iuvw通过clark变化得到iabg，后面iabg通过park变换得到idq，idq通过dq变换得到abc

    if (!Axis_1.FLAG_ENABLE_PWM_OUTPUT){
        DISABLE_PWM_OUTPUT();
        // TODO:需要增加让另外一项axis的Ta Tb Tc在不使用或者
    }else{
        ENABLE_PWM_OUTPUT((*debug).positionLoopType);
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

    if (GpioDataRegs.GPCDAT.bit.GPIO75 == 1)
        Axis_1.FLAG_ENABLE_PWM_OUTPUT = 1;
    else
        Axis_1.FLAG_ENABLE_PWM_OUTPUT = 0;

    /* Step 4. [ePWM] Execute EPWM ISR */
    // 只需要做一次的
    #if NUMBER_OF_DSP_CORES == 2
        write_DAC_buffer();
    #endif
    #if ENABLE_ECAP
        do_enhanced_capture();
    #endif
    // read from CPU02
    read_count_from_cpu02_dsp_cores_2();
    if(IPCRtoLFlagBusy(IPC_FLAG7) == 1){
        run_enable_from_PC = Read.run_enable;
        IPCRtoLFlagAcknowledge(IPC_FLAG7);
    }
    // 对每一个CTRL都需要做一次的代码
    if (use_first_set_three_phase == -1){
        for (axisCnt = 0; axisCnt < NUMBER_OF_AXES; axisCnt++){
            get_Axis_CTRL_pointers //(axisCnt, Axis, CTRL);
            if(axisCnt == 0){
                write_RPM_to_cpu02_dsp_cores_2();
            }
            PanGuMainISR();
        }
        axisCnt = 1; // 这里将axisCnt有什么用啊？因为axisCnt等于2你就会飞了，内存乱写
    }else if (use_first_set_three_phase == 1){
        write_RPM_to_cpu02_dsp_cores_2();
        axisCnt = 0;
        get_Axis_CTRL_pointers //(axisCnt, Axis, CTRL);


        // 这段放需要测时间的代码前面
        #if PC_SIMULATION == FALSE
                EALLOW;
                CpuTimer1.RegsAddr->TCR.bit.TRB = 1;           // reset cpu timer to period value
                CpuTimer1.RegsAddr->TCR.bit.TSS = 0;           // start/restart
                CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
                EDIS;
        #endif

        PanGuMainISR();

        // 这段放需要测时间的代码后面，观察CpuTimer_Delta的取值，代表经过了多少个 1/200e6 秒。
        #if PC_SIMULATION == FALSE
        CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
        CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
        // EALLOW;
        // CpuTimer1.RegsAddr->TCR.bit.TSS = 1; // stop (not needed because of the line TRB=1)
        // EDIS;
        #endif

    }else if (use_first_set_three_phase == 2){
        axisCnt = 1;
        get_Axis_CTRL_pointers //(axisCnt, Axis, CTRL);
        PanGuMainISR();
    }

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

