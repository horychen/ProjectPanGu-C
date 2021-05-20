#include <All_Definition.h>
#include "Experiment.h"
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



//extern int CAP.flag_bad_U_capture;
//extern int CAP.flag_bad_V_capture;
//extern int CAP.flag_bad_W_capture;
void measurement(){

    // 母线电压测量
    G.Voltage_DC_BUS=((AdcaResultRegs.ADCRESULT0)-G.offsetUDC)*AD_scale_VDC + G.Offset_Udc;//

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
        CAP.uab0[1] = 0.57735 * (                                     CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[2] = 0.33333 * (  CAP.terminal_voltage[0] + CAP.terminal_voltage[1] + CAP.terminal_voltage[2]);
        CAP.dq[0] =  CTRL.S->cosT*CAP.uab0[0] + CTRL.S->sinT*CAP.uab0[1];
        CAP.dq[1] = -CTRL.S->sinT*CAP.uab0[0] + CTRL.S->cosT*CAP.uab0[1];

    }else{
        // Assume the voltage vector is rtoating at a constant speed when ecap measurement is disturbed.
        CAP.uab0[0] = CTRL.S->cosT*CAP.dq[0] - CTRL.S->sinT*CAP.dq[1];
        CAP.uab0[1] = CTRL.S->sinT*CAP.dq[0] + CTRL.S->cosT*CAP.dq[1];
    }

    // 电压测量
    if(G.flag_use_ecap_voltage==2){
        /*Use original ecap measured voltage*/
        US_P(0) = CAP.uab0[0];
        US_P(1) = CAP.uab0[1];
    }
    //    else if(G.flag_use_ecap_voltage==3){
    //        ecap_moving_average();
    //    }
    if(G.flag_use_ecap_voltage==1){

        /*Use lpf ecap measured voltage*/
        CAP.dq_lpf[0] = _lpf(CAP.dq[0], CAP.dq_lpf[0], 800);
        CAP.dq_lpf[1] = _lpf(CAP.dq[1], CAP.dq_lpf[1], 800);
        CAP.uab0[0] = CTRL.S->cosT*CAP.dq_lpf[0] - CTRL.S->sinT*CAP.dq_lpf[1];
        CAP.uab0[1] = CTRL.S->sinT*CAP.dq_lpf[0] + CTRL.S->cosT*CAP.dq_lpf[1];
        US_P(0) = CAP.uab0[0];
        US_P(1) = CAP.uab0[1];

    }else if(G.flag_use_ecap_voltage==0){
        /*Use command voltage for feedback*/
        US_P(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
        US_P(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current
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

        if(flag_overwite_voltage_dc_bus){
            Voltage_DC_BUS = Overwrite_Voltage_DC_BUS;
        }
        G.Current_W       =((AdcaResultRegs.ADCRESULT1)-offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        G.Current_V       =((AdcaResultRegs.ADCRESULT3)-offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        G.Current_Not_Used=((AdcaResultRegs.ADCRESULT2)-offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        if(AD_offset_flag2==TRUE){
            G.Current_W = G.Current_W - G.Offset_W;
            G.Current_V = G.Current_V - G.Offset_V;
            G.Current_Not_Used = G.Current_Not_Used - G.Offset_U;
        }
        G.Current_U=-(G.Current_W+G.Current_V);

    #else
        G.Current_W       =((AdcaResultRegs.ADCRESULT1)-G.offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        G.Current_Not_Used=((AdcaResultRegs.ADCRESULT3)-G.offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        G.Current_U       =((AdcaResultRegs.ADCRESULT2)-G.offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
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
    //        DSP_EPWM_DISABLE
    //        DSP_2EPWM_DISABLE
    //    }


    // 电流采样ADC温飘校准
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



//#define AS_LOAD_MOTOR
//#define NSOAF_LOW_SPEED_OPERATION
//#define NSOAF_HIGH_SPEED_OPERATION
//#define XCUBE_TaTbTc_DEBUG_MODE

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

        DSP_EPWM_DISABLE
        DSP_2EPWM_DISABLE
        experiment_init();

        /* 750W MOTOR1 (wo/ hall) */
        //CTRL.motor->R = 1.6;
        CTRL.motor->KE = 0.095;
        //pid1_spd.OutLimit = 10;

        pid1_spd.OutLimit = G.OverwriteSpeedOutLimit;

        #ifdef AS_LOAD_MOTOR
            pid1_spd.OutLimit = 0.01;
            G.Set_manual_rpm = -1200;
        #endif
        #ifdef NSOAF_LOW_SPEED_OPERATION
            low_speed_operation_init();
        #endif
        #ifdef NSOAF_HIGH_SPEED_OPERATION
            high_speed_operation_init();
        #endif

        // for debug
        CTRL.S->PSD_Done = FALSE;
        G.bool_comm_status = 0;

        DELAY_US(11);
        GpioDataRegs.GPDCLEAR.bit.GPIO106=1;

    }else{

        DSP_EPWM_ENABLE
        DSP_2EPWM_ENABLE

        // DSP中控制器的时间
        CTRL.timebase += CL_TS;

        // 根据指令，产生控制输出（电压）
        #if ENABLE_COMMISSIONING == FALSE
            #ifdef AS_LOAD_MOTOR
                CTRL.S->go_sensorless = 0;
                if(CTRL.timebase < 0.4){
                    pid1_spd.OutLimit = 4.2;
                }else if(CTRL.timebase < 0.4 + 0.1){
                    pid1_spd.OutLimit -= CL_TS * 4.2 / 0.1;
                }else{
                    pid1_spd.OutLimit = 0.01;
                }
                //        if(CTRL.timebase < 0.4){
                //            pid1_spd.OutLimit += CL_TS * 4.2 / 0.4;
                //        }else if(CTRL.timebase < 1.0){
                //            pid1_spd.OutLimit = 4.2;
                //        }else{
                //            pid1_spd.OutLimit = 0.01;
                //        }
            #endif
            #ifdef NSOAF_LOW_SPEED_OPERATION
                /* Low Speed Operation*/
                if(TRUE){
                    zero_speed_stopping();
                    zero_speed_stopping_tuning();
                    //short_stopping_at_zero_speed();
                }else{
                    slow_speed_reversal();
                    slow_speed_reversal_tuning();
                }
            #endif
            #ifdef NSOAF_HIGH_SPEED_OPERATION
                /* High Speed Operation */
                high_speed_operation();
                high_speed_operation_tuning();
            #endif
            controller(G.Set_manual_rpm, G.Set_manual_current_iq, G.Set_manual_current_id);
        #else
            commissioning();
        #endif

        #ifdef XCUBE_TaTbTc_DEBUG_MODE
    //        CTRL.svgen1.Ta = 0.6; CTRL.svgen1.Tb = 0.4; CTRL.svgen1.Tc = 0.5;
            if(CTRL.svgen1.Ta>0.7) CTRL.svgen1.Ta=0.7;
            if(CTRL.svgen1.Ta<0.3) CTRL.svgen1.Ta=0.3;
            if(CTRL.svgen1.Tb>0.7) CTRL.svgen1.Tb=0.7;
            if(CTRL.svgen1.Tb<0.3) CTRL.svgen1.Tb=0.3;
            if(CTRL.svgen1.Tc>0.7) CTRL.svgen1.Tc=0.7;
            if(CTRL.svgen1.Tc<0.3) CTRL.svgen1.Tc=0.3;
            EPwm1Regs.CMPA.bit.CMPA = CTRL.svgen1.Ta*50000000*CL_TS;
            EPwm2Regs.CMPA.bit.CMPA = CTRL.svgen1.Tb*50000000*CL_TS;
            EPwm3Regs.CMPA.bit.CMPA = CTRL.svgen1.Tc*50000000*CL_TS;
        #else
            voltage_commands_to_pwm();
        #endif

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

