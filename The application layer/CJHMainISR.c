#include <All_Definition.h>
#if SYSTEM_PROGRAM_MODE==223

// #define CURRENT_CLARKE       clarke1
// #define CURRENT_PARK         park1
// #define VOLTAGE_INVERSE_PARK ipark1
// #define VOLTAGE_PARK         park2

void voltage_commands_to_pwm(){
    // ----------------------------SVGEN����-------------------------------------------------
    svgen1.Ualpha= CTRL.O->uab_cmd_to_inverter[0];
    svgen1.Ubeta = CTRL.O->uab_cmd_to_inverter[1];
    SVGEN_Drive(&svgen1);
    //SPWM_Drive(&svgen1);

    // ------------------------------------------------------------------------------
    EPwm1Regs.CMPA.bit.CMPA = svgen1.Ta*50000000*CL_TS;
    EPwm2Regs.CMPA.bit.CMPA = svgen1.Tb*50000000*CL_TS;
    EPwm3Regs.CMPA.bit.CMPA = svgen1.Tc*50000000*CL_TS;
}


/***********************************************
 * Name��Position / Speed Measurement
 * ���ܣ�Ϊ�����ṩ 10 kHz ���µ�λ�á��ٶȴ��������ݡ�
 ***/
//REAL offset_Udc = 6.0; // 80 V
//REAL offset_Udc = 14.0; // 80 V 408-0800
//REAL offset_Udc = 18.0; // 80 V 409-0900
//REAL offset_Udc = 20.0; // 180 V 409-1300
//REAL offset_Udc = 12.0; // 180 V 413-0844
//REAL offset_Udc = 9.0; // 180 V 414-0809
REAL offset_Udc = 18.0; // 80 V 416-1600
void measurement(){

    // ��ѹ����
    US_P(0) = CTRL.O->uab_cmd[0]; // ��׺_P��ʾ��һ���ĵ�ѹ��P = Previous
    US_P(1) = CTRL.O->uab_cmd[1]; // ��׺_C��ʾ��ǰ���ĵ�ѹ��C = Current

    // ĸ�ߵ�ѹ����
    Voltage_DC_BUS=((AdcaResultRegs.ADCRESULT0)-offsetC)*AD_scale_VDC + offset_Udc;//

    // �����ӿ�
    Current_W=((AdcaResultRegs.ADCRESULT1)-offsetA)*AD_scale_W;// ADC A1-> Phase W Current  //-11.8-11.8A
    Current_V=((AdcaResultRegs.ADCRESULT3)-offsetB)*AD_scale_V;// ADC A1-> Phase V Current  //-11.8-11.8A
    if(AD_offset_flag==TRUE){
        Current_W = Current_W - G.Offset_W;
        Current_V = Current_V - G.Offset_V;
    }
    Current_U=-(Current_W+Current_V);
    REAL adc_ial = UV2A_AI(Current_U, Current_V);
    REAL adc_ibe = UV2B_AI(Current_U, Current_V);

    // ��������
    IS_C(0)        = adc_ial;
    IS_C(1)        = adc_ibe;
    CTRL.I->iab[0] = adc_ial;
    CTRL.I->iab[1] = adc_ibe;

    // ת��λ�ú�ת�ٽӿ� �Լ� ת��λ�ú�ת�ٲ���
    {
        Uint32 QPOSCNT   = EQep1Regs.QPOSCNT;
        ENC.rpm          = PostionSpeedMeasurement_MovingAvergage(QPOSCNT);
        ENC.omg_elec     = ENC.rpm * RPM_2_ELEC_RAD_PER_SEC; // ��еת�٣���λ��RPM��-> �������ٶȣ���λ��elec.rad/s)
        ENC.theta_d_elec = ENC.theta_d__state;
    }
    CTRL.I->omg_elec = ENC.omg_elec;
    CTRL.I->rpm = CTRL.I->omg_elec * ELEC_RAD_PER_SEC_2_RPM;
    CTRL.I->theta_d_elec = ENC.theta_d_elec;

    //    ���������γɱ������������ù���״̬���С�
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
    //    CTRL.motor->KE = 0.104; // ZFY: 0.1A
    //    CTRL.motor->KE = 0.112;  // ZFY: 2.0A
    //    CTRL.motor->KE = 0.15;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=180V (offset_Udc 14V)
    CTRL.motor->KE = 0.10;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=80V (offset_Udc 12V)
}
void low_speed_operation_tuning(){

    /* ʧȥ�ų������������ת�ӵ����� */
    // need to also set id cmd according to speed cmd
    if(fabs(Set_maunal_rpm) < 50){ // 5 or 50?
        Set_maunal_current_id = 2;
    }else{
        Set_maunal_current_id = 0;
    }

    /* Steady state error */
    //    if(Set_maunal_rpm<0){
    //        CTRL.motor->KE = 0.075; // 4.2A
    //    }else{
    //        CTRL.motor->KE = 0.12; // 4.2A
    //    }

    /* Steady state error */
    if(Set_maunal_rpm<-10){
        CTRL.motor->KE = 0.085; //2.1A
    }else if (Set_maunal_rpm>10){
        CTRL.motor->KE = 0.11; // 2.1A
    }else{
        CTRL.motor->KE = 0.10;
    }

    /* ���ּ� */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void low_speed_operation(){
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
    nsoaf.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KPȡ0.5�����ϵ�ʱ��300rpm�����active flux����������
    nsoaf.ActiveFlux_KI = 2.0;
}
void high_speed_operation_tuning(){

    /* ��̬���ܺ�����KP��ֵ������� */
        //    nsoaf.KP = fabs(Set_maunal_rpm) / 1500.0 * 4;
        //    if(nsoaf.KP<1){
        //        nsoaf.KP = 1;
        //    }
    if(fabs(Set_maunal_rpm) > 1000){
        nsoaf.KP = 2;
        nsoaf.ActiveFlux_KP = 1;
    }else if(fabs(Set_maunal_rpm) > 500){
        nsoaf.KP = 1.5;
        nsoaf.ActiveFlux_KP = 0.5;
        //nsoaf.ActiveFlux_KP float   0.100000001 0x0000AA9E@Data
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
        CTRL.motor->KE = 0.092; // positive spinning with load motor@2000 rpm 2.1A
    }else{
        CTRL.motor->KE=0.096;  // negative positive spinning with load motor@2000 rpm 2.1A
    }

    /* ��ֹ�ּ������ֵ */
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

#define NSOAF_LOW_SPEED_OPERATION
//#define NSOAF_HIGW_SPEED_OPERATION

interrupt void CJHMainISR(void)
{
#if NUMBER_OF_DSP_CORES == 2
    write_DAC_buffer();
#endif

DELAY_US(2); // wait for adc conversion TODO: check adc eoc flag?

// ����������DSP�е�ADC������
measurement();

// ��������ADC��ƮУ׼
if(AD_offset_flag==FALSE)
{
    G.Offset_Counter += 1;
    G.Offset_W += Current_W;
    G.Offset_V += Current_V;
    if(G.Offset_Counter>100000){
        G.Offset_W = G.Offset_W / 100000;
        G.Offset_V = G.Offset_V / 100000;
        AD_offset_flag=TRUE;
    }
    //TODO ��Ҫ���ȡƽ��
    //    offsetA=(AdcaResultRegs.ADCRESULT1);
    //    offsetB=(AdcaResultRegs.ADCRESULT3);
    //    offsetC=(AdcaResultRegs.ADCRESULT0)*0;
}


if(Enable_STOP_FLAG) //&&button_isr==1)
{
    DSP_EPWM_DISABLE
    DSP_2EPWM_DISABLE

    experiment_init();

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

    // DSP�п�������ʱ��
    CTRL.timebase += CL_TS;

    // ����ָ����������������ѹ��
    if(ENABLE_COMMISSIONING==FALSE){
        #ifdef NSOAF_LOW_SPEED_OPERATION
            /* Low Speed Operation*/
            low_speed_operation(); // also include zero stopping
            //short_stopping_at_zero_speed();
            //slow_speed_reversal();
            low_speed_operation_tuning();
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

    voltage_commands_to_pwm();

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
    //    /* ���Ʊ� �����Ժ���һ�£����Ʊ�Ӧ���ó��������������֮������ȻҪ�ģ������CTRL.VCurPerUnit == m_a������º衶�������Ӽ�����@p164 */
    //    CTRL.VCurPerUnit = SQRT_2_SLASH_3 * sqrt(CTRL.ual*CTRL.ual+CTRL.ube*CTRL.ube) *2.0 *CTRL.lUdc;
    //    //��ֹ������
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
    //     * ���ֵ�任���ABC����ʱ��ϵ����1����ʱ�任������Ǳ任���ת�ó���3/2��
    //     * ���������õ��Ǻ㹦�ʱ任���任�������Ǳ任���ת�á�
    //     * */
    //    // Inverse Clarke Trans. 2to3 into phase axes
    //    CTRL.VaPU = SQRT_2_SLASH_3 * (       CTRL.ual                                   )* 2.0 * CTRL.lUdc;
    //    CTRL.VbPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_DASH_2PI_SLASH_3 * CTRL.ube )* 2.0 * CTRL.lUdc;
    //    CTRL.VcPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_2PI_SLASH_3      * CTRL.ube )* 2.0 * CTRL.lUdc;
    //}

    //    // ����ĸ�ߵ�ѹ���ܵ��޷�����Ҫ���¼����ѹ��ע�⣬��Ҫ��������������֮ǰ��Ta��Tb��Tc����Ϊʹ��Ta��Tb��Tc�����ѹ�ǲ����������ģ�����Ҳ���ܿ�������������
    //    {
    //        volt_calc1.MfuncV1=svgen1.Ta;
    //        volt_calc1.MfuncV2=svgen1.Tb;
    //        volt_calc1.MfuncV3=svgen1.Tc;
    //        volt_calc1.DcBusVolt=Voltage_DC_BUS;
    //        PHASEVOLT_MACRO(volt_calc1);
    //        volt_calc1.Valpha;
    //        volt_calc1.Vbeta;
    //    }

