#include <All_Definition.h>

void init_experiment_overwrite(){

    init_experiment_AD_gain_and_offset();

    /* Mode Changing During Experiment */
    #ifdef _XCUBE1
        CTRL.g->Seletc_exp_operation = 1; //AS_LOAD_MOTOR_CONST;
        G.dac_watch_stator_resistance = 1.703;
    #else
        // CTRL.g->Seletc_exp_operation = NSOAF_LOW_SPEED_OPERATION;
         CTRL.g->Seletc_exp_operation = NSOAF_HIGH_SPEED_OPERATION;
        // CTRL.g->Seletc_exp_operation = NSOAF_RAMP_SPEED_OPERATION;
        G.dac_watch_stator_resistance = 1.69;
    #endif

    if(G.Seletc_exp_operation == AS_LOAD_MOTOR_CONST){
        pid1_spd.OutLimit = 2.1; //2.0;
        G.Set_manual_rpm = 300;  // motoring + regeneration for low speed test
        G.Set_manual_rpm = 0;    // motoring for high speed test
    }
    else if(G.Seletc_exp_operation == AS_LOAD_MOTOR_RAMP){
        pid1_spd.OutLimit = 0.01;
        G.Set_manual_rpm = -1200;
    }
    else if(G.Seletc_exp_operation == NSOAF_LOW_SPEED_OPERATION){
        low_speed_operation_init();
    }
    else if(G.Seletc_exp_operation == NSOAF_HIGH_SPEED_OPERATION){
        high_speed_operation_init();
    }
    else{
        CTRL.g->Set_manual_rpm = 300;
    }

    #ifdef _XCUBE1
        CTRL.g->OverwriteSpeedOutLimit = 2;

        CTRL.g->Overwrite_Voltage_DC_BUS = 180;
        CTRL.g->flag_overwite_voltage_dc_bus = FALSE;
        CTRL.g->flag_use_ecap_voltage = 0;
    #else
        CTRL.g->OverwriteSpeedOutLimit = 6.3; // 150% rated
    #endif
    pid1_spd.OutLimit = G.OverwriteSpeedOutLimit;

    CTRL.g->DAC_MAX5307_FLAG = FALSE;
    CTRL.g->AD_offset_flag2 = FALSE;
    CTRL.g->FLAG_INVERTER_NONLINEARITY_COMPENSATION = INVERTER_NONLINEARITY_COMPENSATION_INIT;




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


        // tune at 10 rpm sensored
        CTRL.motor->R = 1.7;      // from fitting
        AFEOE.ActiveFlux_KP = 200; // too small, CM correction not effective hence steady state position error; too large, no effect.
        AFEOE.ActiveFlux_KI = 200; // too small, no stable; too large causes steady state position error.
        CTRL.motor->KE = 0.103; // this is effective to tune steady state position error but position speed and negative speed have two different optimal values.
        //CTRL.motor->KE = 0.14; // for negative speed
    #else

        CTRL.motor->R = 1.69;      // from fitting
        CTRL.motor->KE = 0.0968527496;
        CTRL.motor->Js = 0.000767020276;

        //CTRL.motor->R = 2.0;      // from low speed position error correction at 20 rpm for slow reversal. id=2
        //CTRL.motor->R = 2.4;      // from low speed position error correction at 20 rpm for slow reversal. iq=2

        CTRL.motor->R = 1.85; //for negative speed [id?, iq=2 A]
        CTRL.motor->R = 2.20; //for positive speed [id?, iq=2 A]

        CTRL.motor->R = 1.95; //for negative speed [no id, iq=2 A]
        CTRL.motor->R = 2.30; //for positive speed [no id, iq=2 A]

        /* Tuning tips:
         * 1. KP cannot be smaller than 50, for example, KP=40 will make sensorless system unstable at 400 rpm with load.
         * 2. KP can be as large as 150 without any issue at 400 rpm wiht load.
         * 3. KI can be changed in a wide range without any impact on sensorless performance.
         * 4. KP has Speed dependency: lower than 100 rpm, KP can be as low as 20, and the sensorless system is stable with slow reversal.
         * */
        AFEOE.ActiveFlux_KP = 50; // for R=1.7, larger KP causes larger pos error (KP=30 went unstable), but with correct R, large KP can be used.
        AFEOE.ActiveFlux_KI = 25;

        /* ONLy Kp */
        AFEOE.ActiveFlux_KP = 200;
        AFEOE.ActiveFlux_KI = 0.0;

    #endif

    #if TRUE
        MOTOR.KE = 0.095;
        huwu.limiter_KE = 1.0 * MOTOR.KE; // this depends on KE value
    #endif
    AFEOE.limiter_KE = 1.15 * MOTOR.KE; // this depends on KE value

    // for debug
    CTRL.S->PSD_Done = FALSE;
    G.bool_comm_status = 0;
}

void runtime_command_and_tuning(){
    if(G.Seletc_exp_operation == AS_LOAD_MOTOR_CONST){
        CTRL.S->go_sensorless = 0;
        if(CTRL.timebase < 2){
            pid1_spd.OutLimit = 0.1;
        }else{
            pid1_spd.OutLimit = 2.0;
        }
    }
    if(G.Seletc_exp_operation == AS_LOAD_MOTOR_RAMP){
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
    }

    /* Low Speed Operation*/
    if(G.Seletc_exp_operation == NSOAF_LOW_SPEED_OPERATION){
        if(FALSE){
            zero_speed_stopping();
            zero_speed_stopping_tuning();
            //short_stopping_at_zero_speed();
        }else{
            slow_speed_reversal();
            slow_speed_reversal_tuning();
        }
    }

    /* High Speed Reversal Operation */
    if(G.Seletc_exp_operation == NSOAF_HIGH_SPEED_OPERATION){
        high_speed_operation();
        high_speed_operation_tuning();
    }

    /* Ramp High Speed Operation */
    if(G.Seletc_exp_operation == NSOAF_RAMP_SPEED_OPERATION){
        ramp_speed_operation();
        //ramp_speed_operation_tuning();
    }

    /* ���ּ� */
    if(CTRL.motor->KE > 0.2){
        CTRL.motor->KE=0.1;
    }
    if(CTRL.g->Set_manual_rpm>3000){
        CTRL.g->Set_manual_rpm=3000;
    }
    if(CTRL.g->Set_manual_rpm<-3000){
        CTRL.g->Set_manual_rpm=3000;
    }
}


/* NSOAF */

#define SLOW_REVERSAL_RATE 50 // Park.Sul2014 is 90

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
        G.Set_manual_rpm = RPM1; // 0;
    }else if(CTRL.timebase<4-2){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<10-2){
        G.Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm<-RPM1){
            G.Set_manual_rpm = -RPM1;
        }
    }else if(CTRL.timebase<16-2){
        G.Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if(G.Set_manual_rpm>RPM1){
            G.Set_manual_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void low_speed_operation_init(){
    //    pid1_spd.OutLimit = 10;

        //    CTRL.motor->KE = 0.104; // ZFY: 0.1A
        //    CTRL.motor->KE = 0.112;  // ZFY: 2.0A
        //    CTRL.motor->KE = 0.15;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=180V (offset_Udc 14V)
    //    CTRL.motor->KE = 0.10;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=80V (offset_Udc 12V)
}

/*when AS_LOAD_MOTOR_CONST use G.Set_manual_rpm =-300; [rpm], iq is negative*/
//REAL RP =2.15; // 2.15 for cold motor, 2.20 for hot motor
//REAL RN =1.95; // 1.95 for cold motor, 2.00 for hot motor
/*when AS_LOAD_MOTOR_CONST use G.Set_manual_rpm = 300; [rpm], iq is positive*/

/* HUWU--SSR */
REAL RSmall =1.95; // motoring is smaller
REAL RLarge =2.15; // regenerating

/* CM-VM Fusion--SSR */
//REAL RSmall =2.00; // motoring is smaller
//REAL RLarge =2.20; // regenerating

void slow_speed_reversal_tuning(){

    if(CTRL.timebase<30){
        /* ʧȥ�ų������������ת�ӵ����� */
        if(G.flag_auto_id_cmd == TRUE){
            // need to also set id cmd according to speed cmd
            if(fabs(G.Set_manual_rpm) < 50){ // 5 or 50?
                G.Set_manual_current_id = 2;
            }else{
                G.Set_manual_current_id = 0;
            }
        }

        /* Steady state error */
        //        if(CTRL.g->flag_use_ecap_voltage==0){
        //            if(G.Set_manual_rpm>0){
        //                CTRL.motor->R = RP; //for positive speed
        //            }else{
        //                CTRL.motor->R = RN; //for negative speed
        //            }
        //        }else{
        //            RP = 2.20;
        //            RN = 1.80;
        //            if(G.Set_manual_rpm>0){
        //                CTRL.motor->R = RP; //for positive speed
        //            }else{
        //                CTRL.motor->R = RN; //for negative speed
        //            }
        //        }

        if(CTRL.S->Motor_or_Gnerator){
            CTRL.motor->R = RSmall; //for positive speed
        }else{
            CTRL.motor->R = RLarge; //for negative speed
        }

        //    if(G.Set_manual_rpm<-10){
        //
        //        CTRL.motor->KE = 0.14; // for negative speed
        //        //CTRL.motor->KE = 0.085; //2.1A
        //
        //    }else if (G.Set_manual_rpm>10){
        //
        //        CTRL.motor->KE = 0.103; // this is effective to tune steady state position error but position speed and negative speed have two different optimal values.
        //        //CTRL.motor->KE = 0.11; // 2.1A
        //
        //    }else{
        //        CTRL.motor->KE = 0.103;
        //    }
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
    //    pid1_spd.OutLimit = 10; // sensorless needs higher bounds (does not work well with nonlinear controller gain)
    //    CTRL.motor->KE = 0.095;
    //    AFEOE.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KPȡ0.5�����ϵ�ʱ��300rpm�����active flux����������
    //    AFEOE.ActiveFlux_KI = 2.0;
}
int manual_adjust_KE=FALSE;
void high_speed_operation_tuning(){

    #if SELECT_ALGORITHM == ALG_Chi_Xu
    #elif SELECT_ALGORITHM == ALG_NSOAF
        // Original Submission
        /* ��̬���ܺ�����KP��ֵ������� */
            //    nsoaf.KP = fabs(G.Set_manual_rpm) / 1500.0 * 4;
            //    if(nsoaf.KP<1){
            //        nsoaf.KP = 1;
            //    }
        //        if(fabs(G.Set_manual_rpm) > 1000){
        //            nsoaf.KP = 4; //2;
        //            AFEOE.ActiveFlux_KP = 1;
        //        }else if(fabs(G.Set_manual_rpm) > 500){
        //            nsoaf.KP = 2; //1.5;
        //            AFEOE.ActiveFlux_KP = 0.5;
        //        }else if(fabs(G.Set_manual_rpm) > 300){
        //            nsoaf.KP = 1;
        //            AFEOE.ActiveFlux_KP = 0.2;
        //        }else{
        //            AFEOE.ActiveFlux_KP = 0.1;
        //        }

        /* Steady state rpm error when ZFY speed command is at 0 rpm (load sudden change when speed command sign changes) */
        //    if(G.Set_manual_rpm > 0){
        //        CTRL.motor->KE=0.097;
        //    }else{
        //        CTRL.motor->KE=0.099;
        //    }

        /* Steady state rpm error when ZFY speed command is at 2000 rpm. */
    if(manual_adjust_KE==FALSE){
        if(G.Set_manual_rpm > 0){
            //CTRL.motor->KE = 0.0895; // positive spinning with load motor@2000 rpm 2.1A
            CTRL.motor->KE = 0.0895;
        }else{
            //CTRL.motor->KE=0.089;  // negative positive spinning with load motor@2000 rpm 2.1A
            CTRL.motor->KE=  0.0890500024;
        }
    }
    #endif

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

    if(CTRL.timebase<2){ // note 1 sec is not enough for stator flux to reach steady state.
        G.Set_manual_rpm = 500; // give human more time to setup iq limit and load motor speed
    }else if(CTRL.timebase<4.5){
        G.Set_manual_rpm = RPM1;
    }else if(CTRL.timebase<5.0+BIAS){
        G.Set_manual_rpm = -RPM1;
    }else if(CTRL.timebase<5.5+BIAS){
        G.Set_manual_rpm = RPM1;
    }

    // Orignal Submission
    //    if(CTRL.timebase<10){ // note 1 sec is not enough for stator flux to reach steady state.
    //        G.Set_manual_rpm = 300; // give human more time to setup iq limit and load motor speed
    //    }else if(CTRL.timebase<10.5){
    //        G.Set_manual_rpm = RPM1;
    //    }else if(CTRL.timebase<11.0+BIAS){
    //        G.Set_manual_rpm = -RPM1;
    //    }else if(CTRL.timebase<11.5+BIAS){
    //        G.Set_manual_rpm = RPM1;
    //    }
    some_speed_variable = G.Set_manual_rpm;
    #undef RPM1
    #undef BIAS
}


void ramp_speed_operation(){
    if(FALSE){
        #define FAST_REVERSAL_RATE 500
        #define RPM1 1500
        if(CTRL.timebase<1){
            G.Set_manual_rpm = 500;
        }else if(CTRL.timebase<2){
            G.Set_manual_rpm = RPM1;
        }else if(CTRL.timebase<10){
            G.Set_manual_rpm += CL_TS * -FAST_REVERSAL_RATE;
            if(G.Set_manual_rpm<-RPM1){
                G.Set_manual_rpm = -RPM1;
            }
        }else if(CTRL.timebase<18){
            G.Set_manual_rpm += CL_TS * +FAST_REVERSAL_RATE;
            if(G.Set_manual_rpm>RPM1){
                G.Set_manual_rpm = RPM1;
            }
        }
        #undef RPM1
        #undef FAST_REVERSAL_RATE
    }else{
        #define FAST_REVERSAL_RATE 1500
        #define RPM1 1500
        if(CTRL.timebase<1){
            G.Set_manual_rpm = 500;
        }else if(CTRL.timebase<2){
            G.Set_manual_rpm = RPM1;
        }else if(CTRL.timebase<6){
            G.Set_manual_rpm += CL_TS * -FAST_REVERSAL_RATE;
            if(G.Set_manual_rpm<-RPM1){
                G.Set_manual_rpm = -RPM1;
            }
        }else if(CTRL.timebase<10){
            G.Set_manual_rpm += CL_TS * +FAST_REVERSAL_RATE;
            if(G.Set_manual_rpm>RPM1){
                G.Set_manual_rpm = RPM1;
            }
        }
        #undef RPM1
        #undef FAST_REVERSAL_RATE
    }
}
