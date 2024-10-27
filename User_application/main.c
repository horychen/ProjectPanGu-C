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
//             IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
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

    // 4.5 Use GPIO0, GPIO1 and reuse mode is 6 (I2C)
    /*This part is corresponding to the Seeed's Github, of which address is attached below:
     https://github.com/Seeed-Studio/Seeed_LDC1612/blob/master/Seeed_LDC1612.cpp
     This part is corresponding to sensor.single_channel_config from Seeed-LDC1612 */
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 6);
    // GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 6);
    // GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 6);
    I2CA_Init();
    Single_channel_config(0); // 0 for CHANNEL_0


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
            //            if(d_sim.user.bezier_NUMBER_OF_STEPS<8000){
            //                bezier_controller_run_in_main();
            //            }

            if(d_sim.user.bezier_NUMBER_OF_STEPS<8000){
                CTRL = &CTRL_1;
                PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
                PID_Speed->Fbk = (*CTRL).i->varOmega;
                control_output(PID_Speed, &BzController);
                (*debug).set_iq_command = PID_Speed->Out;
                // (*debug).set_iq_command = control_output_adaptVersion(PID_Speed, &BzController_AdaptVersion);
                (*debug).set_id_command = 0;
            }

        #endif

//         #if WHO_IS_USER == USER_QIAN
            // Sensor Coil
            I2CA_ReadData_Channel(0);
            DELAY_US(30);
            I2CA_ReadData_Channel(1);
            DELAY_US(30);
            I2CA_ReadData_Channel(2);
            DELAY_US(300);
            I2CA_ReadData_Channel(3);
            DELAY_US(300);
//         #endif
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

    // measure place between machine shaft and Sensor Coil
    // if (sensor_coil_enable == 1) 
    measurement_sensor_coil();

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
        // init_experiment_PLACE_gain_and_offset();

        // TODO: use a function for this purpose!
        // 清空积分缓存
        PID_Speed->OutPrev = 0;
        PID_Speed->Fbk = 0;
        PID_Speed->Err = 0;
        PID_Speed->ErrPrev = 0;
        PID_Speed->I_Term = 0;

        PID_iD->OutPrev = 0;
        PID_iD->Fbk = 0;
        PID_iD->Err = 0;
        PID_iD->ErrPrev = 0;
        PID_iD->I_Term = 0;
        
        PID_iQ->OutPrev = 0;
        PID_iQ->Fbk = 0;
        PID_iQ->Err = 0;
        PID_iQ->ErrPrev = 0;
        PID_iQ->I_Term = 0;
        // 清空速度InnerLoop缓存
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

    int pwm_test_mode = main_switch(debug->mode_select);

    //(*CTRL).o->cmd_uAB_to_inverter[0]

    // operation mode为5的时候，执行下面测试逆变器输出电压的代码
    if (pwm_test_mode == XCUBE_TaTbTc_DEBUG_MODE){
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





































Uint32 position_count_SCI_shank_fromCPU2;
Uint32 position_count_SCI_hip_fromCPU2;
Uint32 position_count_SCI_fromCPU2;
Uint32 position_count_CAN_ID0x01_fromCPU2;
Uint32 position_count_CAN_ID0x03_fromCPU2;
Uint32 position_count_CAN_fromCPU2;
Uint32 CPU2_commu_error_counter = 0;

int USE_3_CURRENT_SENSORS = TRUE;

REAL legBouncingSpeed = 50;
REAL hipBouncingFreq = 10;
REAL legBouncingIq = 2;
REAL hipBouncingIq = 1.5;
int bool_use_SCI_encoder = TRUE;

REAL target_position_cnt;
REAL target_position_cnt_shank = 58000;
REAL target_position_cnt_hip = 48000;
int bool_TEMP = FALSE;


bool run_enable_from_PC = FALSE;
int counter_missing_position_measurement = 0;
int max_counter_missing_position_measurement = 0;

// 声明全局变量
#if PC_SIMULATION == FALSE
REAL CpuTimer_Delta = 0;
Uint32 CpuTimer_Before = 0;
Uint32 CpuTimer_After = 0;
#endif


void EUREKA_GPIO_SETUP(){
// =========FOR EUREKA===========
    // https://www.ti.com/lit/ug/spruhm8j/spruhm8j.pdf?ts=1710763002632&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMS320F28377D%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Depd-c2x-null-44700045336317350_prodfolderdynamic-cpc-pf-google-wwe_int%2526utm_content%253Dprodfolddynamic%2526ds_k%253DDYNAMIC+SEARCH+ADS%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwzN-vBhAkEiwAYiO7oBh2RIh8aEZiiEHzGLlyxsdf34XwLyRi-Ci53QGwm9calxyVXTdsEBoC7IwQAvD_BwE%2526gclsrc%253Daw.ds
    // Chapter 8.7.1
    /*
    // GPIO62 - CANRXA
    GPIO_SetupPinMux(62, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_ASYNC);
    // GPIO19 - CANTXA
    GPIO_SetupPinMux(19, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO21 - CANRXB
    GPIO_SetupPinMux(21, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_ASYNC);
    // GPIO20 - CANTXB
    GPIO_SetupPinMux(20, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO136 - 485RX-SCIA
    GPIO_SetupPinMux(136, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(136, GPIO_INPUT, GPIO_PUSHPULL);
    // GPIO135 - 485TX-SCIA
    GPIO_SetupPinMux(135, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(135, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO138 - 485RX-SCIB
    GPIO_SetupPinMux(138, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(138, GPIO_INPUT, GPIO_PUSHPULL);
    // GPIO137 - 485TX-SCIB
    GPIO_SetupPinMux(137, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(137, GPIO_OUTPUT, GPIO_PUSHPULL);

    //GPIO39 - SCIRX-C
    GPIO_SetupPinMux(39, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_PUSHPULL);
    //GPIO38 - SCITX-C
    GPIO_SetupPinMux(38, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(38, GPIO_OUTPUT, GPIO_PUSHPULL);
    */

    //CANRXA
    GPIO_SetupPinMux(62, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_ASYNC);
    //CANTXA
    GPIO_SetupPinMux(19, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    //CANRXB
    GPIO_SetupPinMux(21, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_ASYNC);
    //CANTXB
    GPIO_SetupPinMux(20, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);

    //485RX-SCIA
    GPIO_SetupPinMux(PIN_SCI_RXDA, GPIO_MUX_CPU2, MUX_SCI_RXDA);
    GPIO_SetupPinOptions(PIN_SCI_RXDA, GPIO_INPUT, GPIO_PUSHPULL);

    //485TX-SCIA
    GPIO_SetupPinMux(PIN_SCI_TXDA, GPIO_MUX_CPU2, MUX_SCI_TXDA);
    GPIO_SetupPinOptions(PIN_SCI_TXDA, GPIO_OUTPUT, GPIO_PUSHPULL);

    //485RX-SCIB
    GPIO_SetupPinMux(PIN_SCI_RXDB, GPIO_MUX_CPU2, MUX_SCI_RXDB);
    GPIO_SetupPinOptions(PIN_SCI_RXDB, GPIO_INPUT, GPIO_PUSHPULL);
    //485TX-SCIB
    GPIO_SetupPinMux(PIN_SCI_TXDB, GPIO_MUX_CPU2, MUX_SCI_TXDB);
    GPIO_SetupPinOptions(PIN_SCI_TXDB, GPIO_OUTPUT, GPIO_PUSHPULL);

    //SCIRX-C
    GPIO_SetupPinMux(PIN_SCI_RXDC, GPIO_MUX_CPU2, MUX_SCI_RXDC);
    GPIO_SetupPinOptions(PIN_SCI_RXDC, GPIO_INPUT, GPIO_PUSHPULL);
    //SCITX-C
    GPIO_SetupPinMux(PIN_SCI_TXDC, GPIO_MUX_CPU2, MUX_SCI_TXDC);
    GPIO_SetupPinOptions(PIN_SCI_TXDC, GPIO_OUTPUT, GPIO_PUSHPULL);

    // // GPIO31 - 485-SCIB-WE-(use SCICTX as GPIO, in UART3 pin7)
    // GPIO_SetupPinMux(31, GPIO_MUX_CPU2, 0);
    // GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_ASYNC);
    // // GPIO139 - 485-SCIA-WE-(use SCICRX as GPIO, in UART3 pin8)
    // GPIO_SetupPinMux(37, GPIO_MUX_CPU2, 0);
    // GPIO_SetupPinOptions(37, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux    (PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_MUX_CPU2, 0); // GPIO140 - 485-SCIB-WE-(use SCICTX as GPIO, in UART3 pin7)
    GPIO_SetupPinOptions(PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux    (PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_MUX_CPU2, 0); // GPIO139 - 485-SCIA-WE-(use SCICRX as GPIO, in UART3 pin8)
    GPIO_SetupPinOptions(PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_OUTPUT, GPIO_ASYNC);

    // =========FOR EUREKA===========
    //        // =========TEST BOARD PIN============
    //        // =========NOT FOR EUREKA===========
    //        //GPIO30 - CANRXA
    //        GPIO_SetupPinMux(30, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_ASYNC);
    //        //GPIO31 - CANTXA
    //        GPIO_SetupPinMux(31, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    //        //GPIO28 - 485RX-SCIA
    //        GPIO_SetupPinMux(28, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    //        //GPIO29 - 485TX-SCIA
    //        GPIO_SetupPinMux(29, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    //        //GPIO8 - 485WE
    //        GPIO_SetupPinMux(8,GPIO_MUX_CPU2,0);
    //        GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_ASYNC);
    //        InitScicGpio();
}

void axis_basic_setup(int axisCnt){
    Axis->ID = 100 + axisCnt;
    Axis->pCTRL = CTRL;
    Axis->Pdebug = debug;

    // allocate_CTRL(CTRL); // This operation is moved into init_CTRL hence i think this code should not be here 20240929 WB
    init_experiment();
    init_experiment_AD_gain_and_offset();
    init_experiment_PLACE_gain_and_offset();

    // Axis->use_first_set_three_phase = 1; // -1;
    //    Axis->Set_current_loop = FALSE;
    //    Axis->Set_x_suspension_current_loop = FALSE;
    //    Axis->Set_y_suspension_current_loop = FALSE;
    //    Axis->Set_manual_rpm = 50.0;
    //    Axis->Set_manual_current_iq = 0.0;
    //    Axis->Set_manual_current_id = 0.0; // id = -1 A is the magic number to get more torque! cjh 2024-02-29
    //    Axis->Select_exp_operation = 0;    // 200; //202; //200; //101;
    //    // Axis->pFLAG_INVERTER_NONLINEARITY_COMPENSATION = &Axis->pCTRL->g->FLAG_INVERTER_NONLINEARITY_COMPENSATION;
    //   Axis->flag_overwrite_theta_d = FALSE;
    //    Axis->Overwrite_Current_Frequency = 0;
    //    Axis->Overwrite_Suspension_Current_Frequency = 0.5;
    //    Axis->used_theta_d_elec = 0.0;
    //    Axis->angle_shift_for_first_inverter = ANGLE_SHIFT_FOR_FIRST_INVERTER;
    //    Axis->angle_shift_for_second_inverter = ANGLE_SHIFT_FOR_SECOND_INVERTER;
    //    Axis->OverwriteSpeedOutLimitDuringInit = 6; // 10; // A
    //
    //    Axis->FLAG_ENABLE_PWM_OUTPUT = FALSE;

    Axis->channels_preset = 5; // 9; // 101;
    #if WHO_IS_USER == USER_BEZIER
        Axis->channels_preset = 5; // 9; // 101;
    #endif

    Axis->pCTRL->enc->sum_qepPosCnt = 0;
    Axis->pCTRL->enc->cursor = 0;
    Axis->pCTRL->enc->flag_absolute_encoder_powered = FALSE;
}

void init_experiment_AD_gain_and_offset()
{
    /* ADC OFFSET */
    //    // LEM1
    //    Axis->adc_offset[0] = OFFSET_VDC_BUS_IPM1;
    //    Axis->adc_offset[4] = OFFSET_LEM1_L; // b7
    //    Axis->adc_offset[5] = OFFSET_LEM1_R; // b8
    //    Axis->adc_offset[6] = OFFSET_LEM1_M; // b9
    //
    //    // LEM2
    //    Axis->adc_offset[1] = OFFSET_LEM2_L; // a1
    //    Axis->adc_offset[2] = OFFSET_LEM2_R; // a2
    //    Axis->adc_offset[3] = OFFSET_LEM2_M; // a3

    Axis->adc_scale[0] = SCALE_VDC_BUS_IPM1;
    Axis->adc_scale[1] = SCALE_LEM_A1;
    Axis->adc_scale[2] = SCALE_LEM_A2;
    Axis->adc_scale[3] = SCALE_LEM_A3;
    Axis->adc_scale[4] = SCALE_LEM_B7;
    Axis->adc_scale[5] = SCALE_LEM_B8;
    Axis->adc_scale[6] = SCALE_LEM_B9;

    Axis->adc_offset[0] = OFFSET_VDC_BUS_IPM1;
    Axis->adc_offset[1] = OFFSET_LEM_A1;
    Axis->adc_offset[2] = OFFSET_LEM_A2;
    Axis->adc_offset[3] = OFFSET_LEM_A3;
    Axis->adc_offset[4] = OFFSET_LEM_B7;
    Axis->adc_offset[5] = OFFSET_LEM_B8;
    Axis->adc_offset[6] = OFFSET_LEM_B9;

    /* two motor OFFSET */
    #if NUMBER_OF_AXES == 2
        #if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP                      /* eQEP OFFSET */
            if(axisCnt==0){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
            if(axisCnt==0){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK) || (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            if(axisCnt==0){
                Axis_1.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis_2.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #endif
    #else
        #if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP                  /* eQEP OFFSET */
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
            Axis_1.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            // Axis_2.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK)
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #endif
    #endif
}

/* initialize Sensor Coil */
void init_experiment_PLACE_gain_and_offset(){
    Axis->place_offset[0] = OFFSET_PLACE_RIGHT;
    Axis->place_offset[1] = OFFSET_PLACE_DOWN;
    Axis->place_scale[0]  = SCALE_PLACE_X;
    Axis->place_scale[1]  = SCALE_PLACE_Y;
    /* These is prepared for LDC1614 with 4 channels. */
    Axis->place_offset[2] = OFFSET_PLACE_LEFT;
    Axis->place_offset[3] = OFFSET_PLACE_UP;
    Axis->place_scale[2]  = SCALE_PLACE_X;
    Axis->place_scale[3]  = SCALE_PLACE_Y;
}

/* compute CLA task vectors */
void compute_CLA_task_vectors(){
    Cla1Regs.MVECT1 = (Uint16)((Uint32)&Cla1Task1 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT2 = (Uint16)((Uint32)&Cla1Task2 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT3 = (Uint16)((Uint32)&Cla1Task3 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT4 = (Uint16)((Uint32)&Cla1Task4 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT5 = (Uint16)((Uint32)&Cla1Task5 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT6 = (Uint16)((Uint32)&Cla1Task6 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 - (Uint32)&Cla1Prog_Start);
}


void init_spi() {
    // 初始化SPI，用于与DAC芯片MAX5307通讯。
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

    // 同步！！！！！
    InitHighSpeedSpiGpio();
    // InitSpicGpio();
    // InitSpiaGpio();
    // InitSpi(); // this is moved to CPU02
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

#if (FALSE)
    {
        SendChar = 24;
        for (;;)
        {
            // SendChar = 0;
            ScicRegs.SCITXBUF.all = (SendChar);

            //
            // wait for RRDY/RXFFST =1 for 1 data available in FIFO
            //
            while (ScicRegs.SCIFFRX.bit.RXFFST == 0)
            {
            }
            (*CTRL).s->go_sensorless = 100;

            //
            // Check received character
            //
            ReceivedChar = ScicRegs.SCIRXBUF.all;
            if (ReceivedChar != SendChar)
            {
                // error();
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
#endif

REAL wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION = 200;

REAL vvvf_voltage = 4;
REAL vvvf_frequency = 5;
REAL enable_vvvf = FALSE;

REAL wubo_debug_USE_DEATIME_PRECOMP = 0;

/* PWM signal to Inverter Voltage Output SWPWM */
void voltage_commands_to_pwm()
{
    if (axisCnt == 0){
        // SVPWM of the motor 3-phase
        (*CTRL).svgen1.Ualpha = (*CTRL).o->cmd_uAB_to_inverter[0];
        (*CTRL).svgen1.Ubeta  = (*CTRL).o->cmd_uAB_to_inverter[1];

        if (enable_vvvf){
            (*CTRL).svgen1.Ualpha = vvvf_voltage * cos(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
            (*CTRL).svgen1.Ubeta  = vvvf_voltage * sin(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
        }

        SVGEN_Drive(&(*CTRL).svgen1);
        (*CTRL).svgen1.CMPA[0] = (*CTRL).svgen1.Ta * SYSTEM_TBPRD;
        (*CTRL).svgen1.CMPA[1] = (*CTRL).svgen1.Tb * SYSTEM_TBPRD;
        (*CTRL).svgen1.CMPA[2] = (*CTRL).svgen1.Tc * SYSTEM_TBPRD;
        //#if USE_DEATIME_PRECOMP
        if(wubo_debug_USE_DEATIME_PRECOMP){
            // DeadtimeCompensation(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2], (*CTRL).svgen1.CMPA, (*CTRL).svgen1.CMPA_DBC);
            EPwm1Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[0];
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[1];
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[2];
        }
        else{
            EPwm1Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[0];
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[1];
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[2];
        }
    }

    if (axisCnt == 1){
        // SVPWM of the suspension 3-phase
        (*CTRL).svgen2.Ualpha = (*CTRL).o->cmd_uAB_to_inverter[0];
        (*CTRL).svgen2.Ubeta = (*CTRL).o->cmd_uAB_to_inverter[1];

        if (enable_vvvf){
            (*CTRL).svgen2.Ualpha = vvvf_voltage * cos(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
            (*CTRL).svgen2.Ubeta = vvvf_voltage * sin(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
        }
        SVGEN_Drive(&(*CTRL).svgen2); //, -(*CTRL).UNot);
        (*CTRL).svgen2.CMPA[0] = (*CTRL).svgen2.Ta * SYSTEM_TBPRD;
        (*CTRL).svgen2.CMPA[1] = (*CTRL).svgen2.Tb * SYSTEM_TBPRD;
        (*CTRL).svgen2.CMPA[2] = (*CTRL).svgen2.Tc * SYSTEM_TBPRD;
        #if USE_DEATIME_PRECOMP
            DeadtimeCompensation(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5], (*CTRL).svgen2.CMPA, (*CTRL).svgen2.CMPA_DBC);
            EPwm4Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[0];
            EPwm5Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[1];
            EPwm6Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[2];
        #else
            EPwm4Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[0];
            EPwm5Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[1];
            EPwm6Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[2];
        #endif
    }

    //    svgen2.Ualpha = svgen1.Ualpha*0.5        + svgen1.Ubeta*0.8660254; // rotate 60 deg
    //    svgen2.Ubeta  = svgen1.Ualpha*-0.8660254 + svgen1.Ubeta*0.5;
}

#if WHO_IS_USER == USER_YZZ || WHO_IS_USER == USER_CJH
void voltage_measurement_based_on_eCAP()
{
    CAP.terminal_voltage[0] = (CAP.terminal_DutyOnRatio[0]) * Axis->vdc - Axis->vdc * 0.5; // -0.5 is due to duty ratio calculation; - vdc * 0.5 is referring to the center of dc bus capacitor.
    CAP.terminal_voltage[1] = (CAP.terminal_DutyOnRatio[1]) * Axis->vdc - Axis->vdc * 0.5;
    CAP.terminal_voltage[2] = (CAP.terminal_DutyOnRatio[2]) * Axis->vdc - Axis->vdc * 0.5;

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
#endif


void test_ipc_tocpu02()
{
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

void cla_test_codes(){
    /* initialize PI controller */
    pi1.Kp = 5.5f;
    pi1.Ki = 0.015f;
    pi1.i10 = 0.0f;
    pi1.i6 = 1.0f;
    pi1.Umax = 10.2f;
    pi1.Umin = -10.2f;

    /* compute CLA task vectors */
    EALLOW;
    compute_CLA_task_vectors();

    /* CLA task triggers */
    Cla1Regs.MIER.all = 0x00FF;

    /* Switch the CLA program space to the CLA and enable software forcing
     * Also switch over CLA data ram 0,1 and 2
     * CAUTION: The RAMxCPUE bits can only be enabled by writing to the register
     * and not the individual bit field. Furthermore, the status of these bitfields
     * is not reflected in either the watch or register views - they always read as
     * zeros. This is a known bug and the user is advised to test CPU accessibilty
     * first before proceeding
     */
    Cla1Regs.MCTL.bit.IACKE = 1;
    EDIS;
}



extern REAL wubo_debug_motor_enc_dirc[2];

#if ENCODER_TYPE != INCREMENTAL_ENCODER_QEP

void measurement_position_count_axisCnt0(){
    #if (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK)
            position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
    #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
    #endif

    #if NUMBER_OF_AXES == 2
        position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
    #endif
        // 正电流导致编码器读数增大：
        CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[0] * (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
}


void measurement_position_count_axisCnt1(){
    #if NUMBER_OF_AXES == 2
        position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
    #endif
        // 正电流导致编码器读数减小
        CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[1] * ( (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis );
        // dq变化中，d轴理论上指向永磁体的北极，
}

/* 编码器位置信息转换为速度信息 */
void measurement_enc(){
    if (!bool_use_SCI_encoder){
        // 正转电流导致编码器读数减小：
        //CTRL->enc->encoder_abs_cnt = -((int32)cnt_four_bar_map_motor_encoder_angle + CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis);
        // 正转电流导致编码器读数增大：

        CTRL->enc->encoder_abs_cnt = (int32)cnt_four_bar_map_motor_encoder_angle - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
    }

    // 给CTRL->enc->encoder_abs_cnt_previous赋值的操作在measurement()函数中进行

    // ignore this please
    // if (CTRL->enc->flag_absolute_encoder_powered == FALSE)
    // {
    //     CTRL->enc->flag_absolute_encoder_powered = TRUE;
    //     // assume there motor is at still when it is powered on
    //     CTRL->enc->encoder_abs_cnt_previous = CTRL->enc->encoder_abs_cnt;
    // }

    while (CTRL->enc->encoder_abs_cnt > SYSTEM_QEP_QPOSMAX_PLUS_1){ // SYSTEM_QEP_QPOSMAX_PLUS_1为编码器的一圈的脉冲数+1
        CTRL->enc->encoder_abs_cnt -= SYSTEM_QEP_QPOSMAX_PLUS_1;
    }
    while (CTRL->enc->encoder_abs_cnt < 0){
        CTRL->enc->encoder_abs_cnt += SYSTEM_QEP_QPOSMAX_PLUS_1;
    }

    CTRL->enc->theta_d__state = CTRL->enc->encoder_abs_cnt * CNT_2_ELEC_RAD;
    while (CTRL->enc->theta_d__state > M_PI)
        CTRL->enc->theta_d__state -= 2 * M_PI;
    while (CTRL->enc->theta_d__state < -M_PI)
        CTRL->enc->theta_d__state += 2 * M_PI;
    CTRL->enc->theta_d_elec = CTRL->enc->theta_d__state;

    CTRL->enc->encoder_incremental_cnt = CTRL->enc->encoder_abs_cnt - CTRL->enc->encoder_abs_cnt_previous;
    if (CTRL->enc->encoder_incremental_cnt < -0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
        CTRL->enc->encoder_incremental_cnt += (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;
    else if (CTRL->enc->encoder_incremental_cnt > 0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
        CTRL->enc->encoder_incremental_cnt -= (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;

    // CTRL->enc->encoder_incremental_cnt * SYSTEM_QEP_REV_PER_PULSE表示在一次通讯周期内，转子转过的角度，既可以看成是转子的转速（离散系统的pointview2）
    // * 1e4 * 60转换为1分钟内的转速，即将转速单位变为RPM
    CTRL->enc->rpm_raw = CTRL->enc->encoder_incremental_cnt * SYSTEM_QEP_REV_PER_PULSE * 1e4 * 60; // 1e4指的是EPWM1_ISR中断的频率：10kHz


    //                #define ENC(X) (*Axis[X].pCTRL->enc)
    //                ENC(0)
    //                ENC(1)

    // moving average filtering
    CTRL->enc->sum_qepPosCnt -= CTRL->enc->MA_qepPosCnt[CTRL->enc->cursor];
    CTRL->enc->sum_qepPosCnt += CTRL->enc->rpm_raw;                  // CTRL->enc->encoder_incremental_cnt;
    CTRL->enc->MA_qepPosCnt[CTRL->enc->cursor] = CTRL->enc->rpm_raw; // CTRL->enc->encoder_incremental_cnt;

    CTRL->enc->cursor += 1;
    if (CTRL->enc->cursor >= MA_SEQUENCE_LENGTH){
        CTRL->enc->cursor = 0; // Reset CTRL->enc->cursor
    }

    CTRL->enc->rpm = CTRL->enc->sum_qepPosCnt * MA_SEQUENCE_LENGTH_INVERSE; // CL_TS_INVERSE;
    // CTRL->enc->rpm = CTRL->enc->rpm_raw;

    CTRL->enc->varOmega = CTRL->enc->rpm * RPM_2_MECH_RAD_PER_SEC;

    // end of axiscnt

    // 转子位置和转速接口 以及 转子位置和转速测量
    //    int32 QPOSCNT;
    //    if(ENCODER_TYPE == INCREMENTAL_ENCODER_QEP){
    //        QPOSCNT = EQep1Regs.QPOSCNT;
    //    }
    //    // 20240123 腿部电机测试
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_SCI){
    //        QPOSCNT =  - position_count_SCI_fromCPU2; // TODO: 开环电流矢量正转的时候，电机的编码器读数在减小，所以取个负号。
    //        //QPOSCNT = position_count_SCI_fromCPU2;
    //    }
    //    // 使用can_ID0x01编码器
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01){
    //        QPOSCNT = position_count_CAN_ID0x01_fromCPU2;
    //    }
    //    // 使用can_ID0x03编码器
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03){
    //        QPOSCNT = position_count_CAN_ID0x03_fromCPU2;
    //    }
}
#endif

void measurement_current_axisCnt0(){
    // LEM1
    Axis->iuvw[PIN_ADCA_U] = ((REAL)(AdcaResultRegs.ADCRESULT1) - Axis->adc_offset[1]) * Axis->adc_scale[1]; //
    Axis->iuvw[PIN_ADCA_V] = ((REAL)(AdcaResultRegs.ADCRESULT2) - Axis->adc_offset[2]) * Axis->adc_scale[2]; //
    Axis->iuvw[PIN_ADCA_W] = ((REAL)(AdcaResultRegs.ADCRESULT3) - Axis->adc_offset[3]) * Axis->adc_scale[3]; //

    // 电流接口
    if (USE_3_CURRENT_SENSORS)
    {
        Axis->iabg[0] = UVW2A_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
        Axis->iabg[1] = UVW2B_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
        Axis->iabg[2] = UVW2G_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
    }
    else
    {
        REAL phase_V_current = -Axis->iuvw[0] - Axis->iuvw[2];
        Axis->iabg[0] = UV2A_AI(Axis->iuvw[0], phase_V_current);
        Axis->iabg[1] = UV2B_AI(Axis->iuvw[0], phase_V_current);
    }
}

void measurement_current_axisCnt1()
{
    // LEM2
    Axis->iuvw[3] = ((REAL)(AdcbResultRegs.ADCRESULT7) - Axis->adc_offset[4]) * Axis->adc_scale[4]; //
    Axis->iuvw[4] = ((REAL)(AdcbResultRegs.ADCRESULT8) - Axis->adc_offset[5]) * Axis->adc_scale[5]; //
    Axis->iuvw[5] = ((REAL)(AdcbResultRegs.ADCRESULT9) - Axis->adc_offset[6]) * Axis->adc_scale[6]; //

    // 电流接口
    if (USE_3_CURRENT_SENSORS)
    {
        // Axis->iabg[0] = UVW2A_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        // Axis->iabg[1] = UVW2B_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        // Axis->iabg[2] = UVW2G_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        Axis->iabg[0] = UVW2A_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
        Axis->iabg[1] = UVW2B_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
        Axis->iabg[2] = UVW2G_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
    }
    else
    {
        REAL phase_V_current = -Axis->iuvw[3] - Axis->iuvw[5];
        Axis->iabg[0] = UV2A_AI(Axis->iuvw[3], phase_V_current);
        Axis->iabg[1] = UV2B_AI(Axis->iuvw[3], phase_V_current);
    }
}

void measurement_sensor_coil(){
    Axis->place_sensor[0] = (raw_value_rdlu[0] - Axis->place_offset[0])*Axis->place_scale[0];
    Axis->place_sensor[1] = (raw_value_rdlu[1] - Axis->place_offset[1])*Axis->place_scale[1];
    // These is prepared for the LDC1614 with 4 channels.
    Axis->place_sensor[2] = (raw_value_rdlu[2] - Axis->place_offset[2])*Axis->place_scale[2];
    Axis->place_sensor[3] = (raw_value_rdlu[3] - Axis->place_offset[3])*Axis->place_scale[3];
    if (Axis->place_sensor[0] > 0){
            // Axis->xx
    }
}








void read_count_from_cpu02_dsp_cores_2()
{
#if NUMBER_OF_DSP_CORES == 2
    /* CPU02 (Remote) to CPU01 (Local)
     * The register to check is IPCSTS.
     * */
    counter_missing_position_measurement += 1;
    if (IPCRtoLFlagBusy(IPC_FLAG10) == 1) // if flag
    {
        max_counter_missing_position_measurement = counter_missing_position_measurement;
        counter_missing_position_measurement = 0;
        position_count_SCI_shank_fromCPU2 = Read.SCI_shank_position_count;
        position_count_SCI_hip_fromCPU2 = Read.SCI_hip_position_count;
        //            position_count_CAN_ID0x01_fromCPU2 = Read.CAN_position_count_ID0x01;
        //            position_count_CAN_ID0x03_fromCPU2 = Read.CAN_position_count_ID0x03;
        IPCRtoLFlagAcknowledge(IPC_FLAG10);

        // CAN encoder convert to motor built-in encoder
        deg_four_bar_map_motor_encoder_angle = get_motorpos(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125); // 1/131072.0*360.0
        // deg_four_bar_map_motor_encoder_angle = lookup(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125, &ZJL_table);
        rad_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 0.017453292519943295;
        cnt_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 23301.68888888889;




    }
    else
    {
        CPU2_commu_error_counter++;
    }

    if (IPCRtoLFlagBusy(IPC_FLAG11) == 1) // if flag
    {
        max_counter_missing_position_measurement = counter_missing_position_measurement;
        counter_missing_position_measurement = 0;
        position_count_CAN_ID0x01_fromCPU2 = Read.CAN_position_count_ID0x01;
        position_count_CAN_ID0x03_fromCPU2 = Read.CAN_position_count_ID0x03;
        IPCRtoLFlagAcknowledge(IPC_FLAG11);
    }
#endif
}


void write_RPM_to_cpu02_dsp_cores_2(){
    if(run_enable_from_PC == false){
        return;
    }
    if (IPCLtoRFlagBusy(IPC_FLAG9) == 0){
        run_enable_from_PC = false;

//        // 这段放需要测时间的代码后面，观察CpuTimer_Delta的取值，代表经过了多少个 1/200e6 秒。
//        #if PC_SIMULATION == FALSE
//        CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
//        CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
//        // EALLOW;
//        // CpuTimer1.RegsAddr->TCR.bit.TSS = 1; // stop (not needed because of the line TRB=1)
//        // EDIS;
//        #endif

        Write.Read_RPM = (*CTRL).i->varOmega;
        IPCLtoRFlagSet(IPC_FLAG9);

//        // 这段放需要测时间的代码前面
//        #if PC_SIMULATION == FALSE
//                EALLOW;
//                CpuTimer1.RegsAddr->TCR.bit.TRB = 1;           // reset cpu timer to period value
//                CpuTimer1.RegsAddr->TCR.bit.TSS = 0;           // start/restart
//                CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
//                EDIS;
//        #endif
    }
}

