#include <All_Definition.h>


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

    /* 失去磁场定向后起到锁定转子的作用 */
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

    /* 防手贱 */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
}
void zero_speed_stopping_tuning(){

    /* 失去磁场定向后起到锁定转子的作用 */
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

    /* 防手贱 */
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
    AFEOE.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KP取0.5及以上的时候，300rpm会出现active flux马鞍波的现象
    AFEOE.ActiveFlux_KI = 2.0;
}
void high_speed_operation_tuning(){

    /* 动态性能和两个KP的值密切相关 */
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
