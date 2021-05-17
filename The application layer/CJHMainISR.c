#include <All_Definition.h>
#if SYSTEM_PROGRAM_MODE==223

// #define CURRENT_CLARKE       clarke1
// #define CURRENT_PARK         park1
// #define VOLTAGE_INVERSE_PARK ipark1
// #define VOLTAGE_PARK         park2

void voltage_commands_to_pwm(){
    // ----------------------------SVGEN生成-------------------------------------------------
    svgen1.Ualpha= CTRL.O->uab_cmd_to_inverter[0];
    svgen1.Ubeta = CTRL.O->uab_cmd_to_inverter[1];
    //    Deadtime_compensation_selection();
    SVGEN_Drive(&svgen1);
    //SPWM_Drive(&svgen1);

    // ------------------------------------------------------------------------------
    EPwm1Regs.CMPA.bit.CMPA = svgen1.Ta*50000000*CL_TS;
    EPwm2Regs.CMPA.bit.CMPA = svgen1.Tb*50000000*CL_TS;
    EPwm3Regs.CMPA.bit.CMPA = svgen1.Tc*50000000*CL_TS;
}


/***********************************************
 * Name：Position / Speed Measurement
 * 功能：为控制提供 10 kHz 更新的位置、速度传感器数据。
 ***/
//REAL offset_Udc = 6.0; // 80 V
//REAL offset_Udc = 14.0; // 80 V 408-0800
//REAL offset_Udc = 18.0; // 80 V 409-0900
//REAL offset_Udc = 20.0; // 180 V 409-1300
//REAL offset_Udc = 12.0; // 180 V 413-0844
//REAL offset_Udc = 9.0; // 180 V 414-0809
//REAL offset_Udc = 18.0; // 80 V 416-1600
//REAL offset_Udc = 8.0; // 80 V 419-0948
//REAL offset_Udc = 1.0; // 180 V 419-1121
REAL offset_Udc = 0.0; // 180 V 427-1401 @ XCUBE-II
REAL OverwriteSpeedOutLimit = 2;
REAL Overwrite_Voltage_DC_BUS = 180;
int flag_overwite_voltage_dc_bus = FALSE;
void measurement(){

    // 电压测量
    US_P(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
    US_P(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current

    // 母线电压测量
    Voltage_DC_BUS=((AdcaResultRegs.ADCRESULT0)-offsetUDC)*AD_scale_VDC + offset_Udc;//

    // 相电压测量（基于占空比和母线电压）
    CTRL.I->ecap_terminal_voltage[0] = (CTRL.I->ecap_terminal_DutyOnRatio[0]-0.5) * 2 * Voltage_DC_BUS * 0.5; // -0.5 means referring to the center of dc bus capacitor.
    CTRL.I->ecap_terminal_voltage[1] = (CTRL.I->ecap_terminal_DutyOnRatio[1]-0.5) * 2 * Voltage_DC_BUS * 0.5;
    CTRL.I->ecap_terminal_voltage[2] = (CTRL.I->ecap_terminal_DutyOnRatio[2]-0.5) * 2 * Voltage_DC_BUS * 0.5;

    CTRL.I->ecap_line_to_line_voltage[0] = CTRL.I->ecap_terminal_voltage[0] - CTRL.I->ecap_terminal_voltage[1];
    CTRL.I->ecap_line_to_line_voltage[1] = CTRL.I->ecap_terminal_voltage[1] - CTRL.I->ecap_terminal_voltage[2];
    CTRL.I->ecap_line_to_line_voltage[2] = CTRL.I->ecap_terminal_voltage[2] - CTRL.I->ecap_terminal_voltage[0];

    CTRL.I->ecap_uab0[0] = 0.33333 * (2*CTRL.I->ecap_terminal_voltage[0] - CTRL.I->ecap_terminal_voltage[1] - CTRL.I->ecap_terminal_voltage[2]);
    CTRL.I->ecap_uab0[1] = 0.57735 * (                                     CTRL.I->ecap_terminal_voltage[1] - CTRL.I->ecap_terminal_voltage[2]);
    CTRL.I->ecap_uab0[2] = 0.33333 * (  CTRL.I->ecap_terminal_voltage[0] + CTRL.I->ecap_terminal_voltage[1] + CTRL.I->ecap_terminal_voltage[2]);

    // 电流接口
    #ifdef _XCUBE1
        //        Current_Not_Used=((AdcaResultRegs.ADCRESULT1)-offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        //        Current_V       =((AdcaResultRegs.ADCRESULT3)-offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        //        Current_U       =((AdcaResultRegs.ADCRESULT2)-offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        //        if(AD_offset_flag2==TRUE){
        //            Current_Not_Used = Current_Not_Used - G.Offset_W;
        //            Current_V = Current_V - G.Offset_V;
        //            Current_U = Current_U - G.Offset_U;
        //        }
        //        Current_W=-(Current_V+Current_U);

        if(flag_overwite_voltage_dc_bus){
            Voltage_DC_BUS = Overwrite_Voltage_DC_BUS;
        }
        Current_W       =((AdcaResultRegs.ADCRESULT1)-offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        Current_V       =((AdcaResultRegs.ADCRESULT3)-offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        Current_Not_Used=((AdcaResultRegs.ADCRESULT2)-offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        if(AD_offset_flag2==TRUE){
            Current_W = Current_W - G.Offset_W;
            Current_V = Current_V - G.Offset_V;
            Current_Not_Used = Current_Not_Used - G.Offset_U;
        }
        Current_U=-(Current_W+Current_V);

    #else
        Current_W       =((AdcaResultRegs.ADCRESULT1)-offsetW)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
        Current_V       =((AdcaResultRegs.ADCRESULT3)-offsetV)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
        Current_Not_Used=((AdcaResultRegs.ADCRESULT2)-offsetU)*AD_scale_U;// ADC A1-> Phase U Current  //-11.8-11.8A
        if(AD_offset_flag2==TRUE){
            Current_W = Current_W - G.Offset_W;
            Current_V = Current_V - G.Offset_V;
            Current_Not_Used = Current_Not_Used - G.Offset_U;
        }
        Current_U=-(Current_W+Current_V);
    #endif
    REAL adc_ial = UV2A_AI(Current_U, Current_V);
    REAL adc_ibe = UV2B_AI(Current_U, Current_V);

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
    //    if(fabs(Current_W)>8 || fabs(Current_V)>8){
    //        DSP_EPWM_DISABLE
    //        DSP_2EPWM_DISABLE
    //    }
}


#define SLOW_REVERSAL_RATE 50 // Park.Sul2014

void short_stopping_at_zero_speed(){

    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        Set_maunal_rpm = 0;
    }else if(CTRL.timebase<5){
        Set_maunal_rpm = RPM1;
    }else if(CTRL.timebase<10){
        Set_maunal_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm < 0){
            Set_maunal_rpm = 0.0;
        }
    }else if(CTRL.timebase<15){
        Set_maunal_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm<-RPM1){
            Set_maunal_rpm = -RPM1;
        }
    }else if(CTRL.timebase<20){
        Set_maunal_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm > 0){
            Set_maunal_rpm = 0.0;
        }
    }else if(CTRL.timebase<25){
        Set_maunal_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm>RPM1){
            Set_maunal_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void slow_speed_reversal(){

    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        Set_maunal_rpm = 0;
    }else if(CTRL.timebase<4){
        Set_maunal_rpm = RPM1;
    }else if(CTRL.timebase<10){
        Set_maunal_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm<-RPM1){
            Set_maunal_rpm = -RPM1;
        }
    }else if(CTRL.timebase<16){
        Set_maunal_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(Set_maunal_rpm>RPM1){
            Set_maunal_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void low_speed_operation_init(){
    pid1_spd.OutLimit = 10;

    //    CTRL.motor->KE = 0.104; // ZFY: 0.1A
    //    CTRL.motor->KE = 0.112;  // ZFY: 2.0A
    //    CTRL.motor->KE = 0.15;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=180V (offset_Udc 14V)
    CTRL.motor->KE = 0.10;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=80V (offset_Udc 12V)
}
void slow_speed_reversal_tuning(){

    /* 失去磁场定向后起到锁定转子的作用 */
    // need to also set id cmd according to speed cmd
    if(fabs(Set_maunal_rpm) < 50){ // 5 or 50?
        Set_maunal_current_id = 2;
    }else{
        Set_maunal_current_id = 0;
    }

    /* Steady state error */
    if(Set_maunal_rpm<-10){
        CTRL.motor->KE = 0.085; //2.1A
    }else if (Set_maunal_rpm>10){
        CTRL.motor->KE = 0.11; // 2.1A
    }else{
        CTRL.motor->KE = 0.10;
    }

    /* 防手贱 */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void zero_speed_stopping_tuning(){

    /* 失去磁场定向后起到锁定转子的作用 */
    // need to also set id cmd according to speed cmd
    if(fabs(Set_maunal_rpm) < 50){ // 5 or 50?
        Set_maunal_current_id = 2;
    }else{
        //Set_maunal_current_id = 0;
    }

    /* Steady state error */
    if(Set_maunal_rpm<-5){
        CTRL.motor->KE = 0.082; //2.1A
    }else if (Set_maunal_rpm>5){
        CTRL.motor->KE = 0.118; // 2.1A
    }else{
        CTRL.motor->KE = 0.10;
    }

    /* 防手贱 */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void zero_speed_stopping(){
    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        Set_maunal_rpm = 0;
    }else if(CTRL.timebase<4){
        Set_maunal_rpm = RPM1;
    }else if(CTRL.timebase<6+BIAS){
        Set_maunal_rpm = -RPM1;
    }else if(CTRL.timebase<9+BIAS){
        Set_maunal_rpm = 0;
    }else if(CTRL.timebase<18+BIAS){
        Set_maunal_rpm = RPM1;
    }else if(CTRL.timebase<25+BIAS){
        Set_maunal_rpm = RPM1*sin(2*2*M_PI*CTRL.timebase);
    }

    #undef RPM1
    #undef BIAS
}

void high_speed_operation_init(){
    pid1_spd.OutLimit = 10; // sensorless needs higher bounds (does not work well with nonlinear controller gain)
    CTRL.motor->KE = 0.095;
    nsoaf.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KP取0.5及以上的时候，300rpm会出现active flux马鞍波的现象
    nsoaf.ActiveFlux_KI = 2.0;
}
void high_speed_operation_tuning(){

    /* 动态性能和两个KP的值密切相关 */
        //    nsoaf.KP = fabs(Set_maunal_rpm) / 1500.0 * 4;
        //    if(nsoaf.KP<1){
        //        nsoaf.KP = 1;
        //    }
    if(fabs(Set_maunal_rpm) > 1000){
        nsoaf.KP = 4; //2;
        nsoaf.ActiveFlux_KP = 1;
    }else if(fabs(Set_maunal_rpm) > 500){
        nsoaf.KP = 2; //1.5;
        nsoaf.ActiveFlux_KP = 0.5;
    }else if(fabs(Set_maunal_rpm) > 300){
        nsoaf.KP = 1;
        nsoaf.ActiveFlux_KP = 0.2;
    }else{
        nsoaf.ActiveFlux_KP = 0.1;
    }

    /* Steady state rpm error when ZFY speed command is at 0 rpm (load sudden change when speed command sign changes) */
    //    if(Set_maunal_rpm > 0){
    //        CTRL.motor->KE=0.097;
    //    }else{
    //        CTRL.motor->KE=0.099;
    //    }

    /* Steady state rpm error when ZFY speed command is at 2000 rpm. */
    if(Set_maunal_rpm > 0){
        CTRL.motor->KE = 0.103; // positive spinning with load motor@2000 rpm 2.1A
    }else{
        CTRL.motor->KE=0.087;  // negative positive spinning with load motor@2000 rpm 2.1A
    }

    /* 防止手贱输错了值 */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE = 0.1;
    }
}
REAL some_speed_variable = 0.0;
void high_speed_operation(){

    #define RPM1 1500
    #define BIAS 0
    //    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
    //        Set_maunal_rpm = 100;
    //    }else if(CTRL.timebase<1.5){
    //        Set_maunal_rpm = RPM1;
    //    }else if(CTRL.timebase<2.0+BIAS){
    //        Set_maunal_rpm = -RPM1;
    //    }else if(CTRL.timebase<2.5+BIAS){
    //        Set_maunal_rpm = RPM1;
    //    }else if(CTRL.timebase<3.0+BIAS){
    //        Set_maunal_rpm = -100;
    //    }

    if(CTRL.timebase<10){ // note 1 sec is not enough for stator flux to reach steady state.
        Set_maunal_rpm = 300; // give human more time to setup iq limit and load motor speed
    }else if(CTRL.timebase<10.5){
        Set_maunal_rpm = RPM1;
    }else if(CTRL.timebase<11.0+BIAS){
        Set_maunal_rpm = -RPM1;
    }else if(CTRL.timebase<11.5+BIAS){
        Set_maunal_rpm = RPM1;
    }

    some_speed_variable = Set_maunal_rpm;
    #undef RPM1
    #undef BIAS
}

//#define AS_LOAD_MOTOR
//#define NSOAF_LOW_SPEED_OPERATION
//#define NSOAF_HIGH_SPEED_OPERATION
//#define XCUBE_DEBUG_MODE

struct TestECapture {
    REAL TSt1, TSt2, TSt3, TSt4, Period1, Period2, Period3, DutyOnTime1, DutyOffTime1, DutyOnTime2, DutyOffTime2;
} ecapU, ecapV, ecapW;
void do_enhanced_capture(){    // Section 16.6.4 // The 32 bit counter is stored in ECap1Regs.TSCTR
    /*U*/
    ecapU.DutyOnTime1 = ECap1Regs.CAP2;
    ecapU.DutyOffTime1 = ECap1Regs.CAP3;
    ecapU.DutyOnTime2 = ECap1Regs.CAP4;
    ecapU.DutyOffTime2 = ECap1Regs.CAP1;
    ecapU.Period1 = ecapU.DutyOnTime1 + ecapU.DutyOffTime1;
    ecapU.Period2 = ecapU.DutyOnTime2 + ecapU.DutyOffTime2;

    /*V*/
    ecapV.DutyOnTime1 = ECap2Regs.CAP2;
    ecapV.DutyOffTime1 = ECap2Regs.CAP3;
    ecapV.DutyOnTime2 = ECap2Regs.CAP4;
    ecapV.DutyOffTime2 = ECap2Regs.CAP1;
    ecapV.Period1 = ecapV.DutyOnTime1 + ecapV.DutyOffTime1;
    ecapV.Period2 = ecapV.DutyOnTime2 + ecapV.DutyOffTime2;

    /*W*/
    ecapW.DutyOnTime1 = ECap3Regs.CAP2;
    ecapW.DutyOffTime1 = ECap3Regs.CAP3;
    ecapW.DutyOnTime2 = ECap3Regs.CAP4;
    ecapW.DutyOffTime2 = ECap3Regs.CAP1;
    ecapW.Period1 = ecapW.DutyOnTime1 + ecapW.DutyOffTime1;
    ecapW.Period2 = ecapW.DutyOnTime2 + ecapW.DutyOffTime2;

    /*API to global variable CTRL*/
    CTRL.I->ecap_terminal_DutyOnRatio[0] = 0.5*(ecapU.DutyOnTime1 + ecapU.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
    CTRL.I->ecap_terminal_DutyOnRatio[1] = 0.5*(ecapV.DutyOnTime1 + ecapV.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE;
    CTRL.I->ecap_terminal_DutyOnRatio[2] = 0.5*(ecapW.DutyOnTime1 + ecapW.DutyOnTime2) * SYSTEM_PWM_MAX_COUNT_INVERSE; // / (0.5*(ecapW.Period1+ecapW.Period2))
    CTRL.I->ecap_pwm_time = 0.5*(ecapU.Period1 + ecapU.Period2);
}

interrupt void CJHMainISR(void)
{

#if NUMBER_OF_DSP_CORES == 2
    write_DAC_buffer();
#endif

do_enhanced_capture(); //DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?

// 采样，包括DSP中的ADC采样等
measurement();

// 电流采样ADC温飘校准
if(AD_offset_flag2==FALSE)
{
    G.Offset_Counter += 1;
    G.Offset_W += Current_W;
    G.Offset_V += Current_V;
    G.Offset_U += Current_U;
    if(G.Offset_Counter>5000){
        G.Offset_W = G.Offset_W / 5000;
        G.Offset_V = G.Offset_V / 5000;
        G.Offset_U = G.Offset_U / 5000;
        AD_offset_flag2 = TRUE;
    }

    // 来不及完成偏置检测（比如刚上电数字开关就是开的），采用默认值
    /* 427-1401：添加开关信号滤波。今天发现在刚上电的时候，XCUBE-II的前两个中断里，数字开关是打开的，然后才变成关闭。*/
    if(FLAG_ENABLE_PWM_OUTPUT && G.Offset_Counter>100){
        G.Offset_W = 0.0;
        G.Offset_V = 0.0;
        G.Offset_U = 0.0;
        AD_offset_flag2 = TRUE;
    }

    // 上电的时候，电机可能在转，此时根据电流判断是否还要额外进行偏置补偿。
    if(fabs(Current_W)>0.05 || fabs(Current_V)>0.05 || fabs(Current_U)>0.05){
        G.Offset_W = 0.0;
        G.Offset_V = 0.0;
        G.Offset_U = 0.0;
        AD_offset_flag2 = TRUE;
    }
}


if(!FLAG_ENABLE_PWM_OUTPUT) //&&button_isr==1)
{
    DSP_EPWM_DISABLE
    DSP_2EPWM_DISABLE

    experiment_init();

    /* 750W MOTOR1 (wo/ hall) */
    CTRL.motor->KE = 0.095;
    //pid1_spd.OutLimit = 10;

    pid1_spd.OutLimit = OverwriteSpeedOutLimit;

    #ifdef AS_LOAD_MOTOR
        pid1_spd.OutLimit = 0.01;
        Set_maunal_rpm = -1200;
    #endif
    #ifdef NSOAF_LOW_SPEED_OPERATION
        low_speed_operation_init();
    #endif
    #ifdef NSOAF_HIGH_SPEED_OPERATION
        high_speed_operation_init();
    #endif

    // for debug
    CTRL.S->PSD_Done = FALSE;
    bool_comm_status = 0;

    DELAY_US(11);
    GpioDataRegs.GPDCLEAR.bit.GPIO106=1;
}
else
{
    DSP_EPWM_ENABLE
    DSP_2EPWM_ENABLE

    // DSP中控制器的时间
    CTRL.timebase += CL_TS;

    // 根据指令，产生控制输出（电压）
    if(ENABLE_COMMISSIONING==FALSE){
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
        controller(Set_maunal_rpm, Set_maunal_current_iq, Set_maunal_current_id);
    }
    else{
        commissioning();
    }

    #ifdef XCUBE_DEBUG_MODE
//        svgen1.Ta = 0.6; svgen1.Tb = 0.4; svgen1.Tc = 0.5;
        if(svgen1.Ta>0.7) svgen1.Ta=0.7;
        if(svgen1.Ta<0.3) svgen1.Ta=0.3;
        if(svgen1.Tb>0.7) svgen1.Tb=0.7;
        if(svgen1.Tb<0.3) svgen1.Tb=0.3;
        if(svgen1.Tc>0.7) svgen1.Tc=0.7;
        if(svgen1.Tc<0.3) svgen1.Tc=0.3;
        EPwm1Regs.CMPA.bit.CMPA = svgen1.Ta*50000000*CL_TS;
        EPwm2Regs.CMPA.bit.CMPA = svgen1.Tb*50000000*CL_TS;
        EPwm3Regs.CMPA.bit.CMPA = svgen1.Tc*50000000*CL_TS;
    #else
        voltage_commands_to_pwm();
    #endif

    #if NUMBER_OF_DSP_CORES == 1
        single_core_dac();
    #endif
}

EPwm1Regs.ETCLR.bit.INT = 1;
PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

#endif

    //void SPWM_Drive(SVGENDQ* ptrV){
    //
    //    //    // outputs are inv.ual_comp, inv.u_comp[3]
    //    //    Modified_ParkSul_Compensation(); // inverter nonlinearity compensation
    //    //    // apply conpensation to the comanded voltage signals in al-be frame
    //    //    CTRL.ual = CTRL.ual + inv.ual_comp*0;
    //    //    CTRL.ube = CTRL.ube + inv.ube_comp*0;
    //
    //    /* 调制比 这里以后检查一下，调制比应该用乘以这个根号三分之二吗？显然要的，这里的CTRL.VCurPerUnit == m_a，见徐德鸿《电力电子技术》@p164 */
    //    CTRL.VCurPerUnit = SQRT_2_SLASH_3 * sqrt(CTRL.ual*CTRL.ual+CTRL.ube*CTRL.ube) *2.0 *CTRL.lUdc;
    //    //防止过调制
    //    if(CTRL.VCurPerUnit > 1){
    //        float temp = 1.0/CTRL.VCurPerUnit;
    //        CTRL.pi_iMs.i_state *= temp;
    //        CTRL.pi_iTs.i_state *= temp;
    //        CTRL.uMs_cmd *= temp;
    //        CTRL.uTs_cmd *= temp;
    //        CTRL.ual *= temp;
    //        CTRL.ube *= temp;
    //        /*
    //        \[\begin{array}{l}
    //        {\mathop{\rm suppose}\nolimits} \;ma = \sqrt {\frac{2}{3}} \sqrt {u_{\alpha s}^2 + u_{\beta s}^2}  > 1\\
    //        let\;\sqrt {\frac{2}{3}} \sqrt {{{\left( {c{u_{\alpha s}}} \right)}^2} + {{\left( {c{u_{\beta s}}} \right)}^2}}  = 1\\
    //         \Rightarrow \frac{1}{c} = \sqrt {\frac{2}{3}} \sqrt {{{\left( {{u_{\alpha s}}} \right)}^2} + {{\left( {{u_{\beta s}}} \right)}^2}}  = ma
    //        \end{array}\]
    //         */
    //        CTRL.VCurPerUnit = 1;
    //        CTRL.OverModulation = TRUE;
    //    }
    //    else{
    //        CTRL.OverModulation = FALSE;
    //    }
    //
    //    /*
    //     * 恒幅值变换变成ABC三相时的系数是1，此时变换阵的逆是变换阵的转置乘以3/2。
    //     * 但是我们用的是恒功率变换，变换阵的逆就是变换阵的转置。
    //     * */
    //    // Inverse Clarke Trans. 2to3 into phase axes
    //    CTRL.VaPU = SQRT_2_SLASH_3 * (       CTRL.ual                                   )* 2.0 * CTRL.lUdc;
    //    CTRL.VbPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_DASH_2PI_SLASH_3 * CTRL.ube )* 2.0 * CTRL.lUdc;
    //    CTRL.VcPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_2PI_SLASH_3      * CTRL.ube )* 2.0 * CTRL.lUdc;
    //}

    //    // 考虑母线电压可能的限幅，需要重新计算电压。注意，需要用死区补偿作用之前的Ta，Tb和Tc，因为使用Ta，Tb，Tc计算电压是不考虑死区的，所以也不能考虑死区补偿。
    //    {
    //        volt_calc1.MfuncV1=svgen1.Ta;
    //        volt_calc1.MfuncV2=svgen1.Tb;
    //        volt_calc1.MfuncV3=svgen1.Tc;
    //        volt_calc1.DcBusVolt=Voltage_DC_BUS;
    //        PHASEVOLT_MACRO(volt_calc1);
    //        volt_calc1.Valpha;
    //        volt_calc1.Vbeta;
    //    }

