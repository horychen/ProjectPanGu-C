#include <All_Definition.h>

void init_experiment_overwrite(){
    /* 750W MOTOR1 (wo/ hall) */
    //CTRL.motor->R = 1.6;
    CTRL.motor->KE = 0.095;
    //pid1_spd.OutLimit = 10;

    #ifdef _XCUBE1
                                 // no DT comp, comp, comp, comp
    CTRL.motor->R = 1.4;      // 1.39377081; 1.57862532; 1.41320038, 1.40607429
    CTRL.motor->KE = 0.10378; // 0.122107916; 0.103980549, 0.103777379, 0.103794798
    CTRL.motor->Js = 0.0007460128; // 0.000751407992; 0.000751283951, 0.000746554753, 0.000740199641
    CTRL.motor->Ld = CTRL.motor->Lq = 0.0108864028;
    #endif

    pid1_spd.OutLimit = G.OverwriteSpeedOutLimit;

    #ifdef AS_LOAD_MOTOR_CONST
        pid1_spd.OutLimit = 2.0;
        G.Set_manual_rpm = -300;
    #endif
    #ifdef AS_LOAD_MOTOR_RAMP
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
}


/* NSOAF */

#define SLOW_REVERSAL_RATE 50 // Park.Sul2014

void short_stopping_at_zero_speed(){

    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        G.Set_manual_rpm = 0;
    }else if(CTRL.timebase<5){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<10){
        G.Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm < 0){
            G.Set_manual_rpm = 0.0;
        }
    }else if(CTRL.timebase<15){
        G.Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm<-RPM1){
            G.Set_manual_rpm = -RPM1;
        }
    }else if(CTRL.timebase<20){
        G.Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm > 0){
            G.Set_manual_rpm = 0.0;
        }
    }else if(CTRL.timebase<25){
        G.Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm>RPM1){
            G.Set_manual_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void slow_speed_reversal(){

    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        G.Set_manual_rpm = 0;
    }else if(CTRL.timebase<4){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<10){
        G.Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm<-RPM1){
            G.Set_manual_rpm = -RPM1;
        }
    }else if(CTRL.timebase<16){
        G.Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm>RPM1){
            G.Set_manual_rpm = RPM1;
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

    /* ʧȥ�ų������������ת�ӵ����� */
    // need to also set id cmd according to speed cmd
    if(fabs(G.Set_manual_rpm) < 50){ // 5 or 50?
        G.Set_manual_current_id = 2;
    }else{
        G.Set_manual_current_id = 0;
    }

    /* Steady state error */
//    if(G.Set_manual_rpm<-10){
//        CTRL.motor->KE = 0.085; //2.1A
//    }else if (G.Set_manual_rpm>10){
//        CTRL.motor->KE = 0.11; // 2.1A
//    }else{
//        CTRL.motor->KE = 0.10;
//    }

    /* ���ּ� */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void zero_speed_stopping_tuning(){

    /* ʧȥ�ų������������ת�ӵ����� */
    // need to also set id cmd according to speed cmd
    if(fabs(G.Set_manual_rpm) < 50){ // 5 or 50?
        G.Set_manual_current_id = 2;
    }else{
        //G.Set_manual_current_id = 0;
    }

    /* Steady state error */
    if(G.Set_manual_rpm<-5){
        CTRL.motor->KE = 0.082; //2.1A
    }else if (G.Set_manual_rpm>5){
        CTRL.motor->KE = 0.118; // 2.1A
    }else{
        CTRL.motor->KE = 0.10;
    }

    /* ���ּ� */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void zero_speed_stopping(){
    #define RPM1 100
    #define BIAS 0
    if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        G.Set_manual_rpm = 0;
    }else if(CTRL.timebase<4){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<6+BIAS){
        G.Set_manual_rpm = -RPM1;
    }else if(CTRL.timebase<9+BIAS){
        G.Set_manual_rpm = 0;
    }else if(CTRL.timebase<18+BIAS){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<25+BIAS){
        G.Set_manual_rpm = RPM1*sin(2*2*M_PI*CTRL.timebase);
    }

    #undef RPM1
    #undef BIAS
}

void high_speed_operation_init(){
    pid1_spd.OutLimit = 10; // sensorless needs higher bounds (does not work well with nonlinear controller gain)
    CTRL.motor->KE = 0.095;
    AFEOE.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KPȡ0.5�����ϵ�ʱ��300rpm�����active flux����������
    AFEOE.ActiveFlux_KI = 2.0;
}
void high_speed_operation_tuning(){

    /* ��̬���ܺ�����KP��ֵ������� */
        //    nsoaf.KP = fabs(G.Set_manual_rpm) / 1500.0 * 4;
        //    if(nsoaf.KP<1){
        //        nsoaf.KP = 1;
        //    }
    if(fabs(G.Set_manual_rpm) > 1000){
        nsoaf.KP = 4; //2;
        AFEOE.ActiveFlux_KP = 1;
    }else if(fabs(G.Set_manual_rpm) > 500){
        nsoaf.KP = 2; //1.5;
        AFEOE.ActiveFlux_KP = 0.5;
    }else if(fabs(G.Set_manual_rpm) > 300){
        nsoaf.KP = 1;
        AFEOE.ActiveFlux_KP = 0.2;
    }else{
        AFEOE.ActiveFlux_KP = 0.1;
    }

    /* Steady state rpm error when ZFY speed command is at 0 rpm (load sudden change when speed command sign changes) */
    //    if(G.Set_manual_rpm > 0){
    //        CTRL.motor->KE=0.097;
    //    }else{
    //        CTRL.motor->KE=0.099;
    //    }

    /* Steady state rpm error when ZFY speed command is at 2000 rpm. */
    if(G.Set_manual_rpm > 0){
        CTRL.motor->KE = 0.103; // positive spinning with load motor@2000 rpm 2.1A
    }else{
        CTRL.motor->KE=0.087;  // negative positive spinning with load motor@2000 rpm 2.1A
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
    //        G.Set_manual_rpm = 100;
    //    }else if(CTRL.timebase<1.5){
    //        G.Set_manual_rpm = RPM1;
    //    }else if(CTRL.timebase<2.0+BIAS){
    //        G.Set_manual_rpm = -RPM1;
    //    }else if(CTRL.timebase<2.5+BIAS){
    //        G.Set_manual_rpm = RPM1;
    //    }else if(CTRL.timebase<3.0+BIAS){
    //        G.Set_manual_rpm = -100;
    //    }

    if(CTRL.timebase<10){ // note 1 sec is not enough for stator flux to reach steady state.
        G.Set_manual_rpm = 300; // give human more time to setup iq limit and load motor speed
    }else if(CTRL.timebase<10.5){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<11.0+BIAS){
        G.Set_manual_rpm = -RPM1;
    }else if(CTRL.timebase<11.5+BIAS){
        G.Set_manual_rpm = RPM1;
    }

    some_speed_variable = G.Set_manual_rpm;
    #undef RPM1
    #undef BIAS
}
