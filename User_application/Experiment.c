#include <All_Definition.h>

void init_experiment_overwrite(){

    /* Mode Changing During Experiment */
    #ifdef _XCUBE1
        (*Axis).Select_exp_operation = AS_LOAD_MOTOR_CONST;
        G.dac_watch_stator_resistance = 1.703;

        // 2021-07-17
        //(*Axis).Select_exp_operation = NSOAF_LOW_SPEED_OPERATION;

    #else
        // (*Axis).Select_exp_operation = NSOAF_LOW_SPEED_OPERATION;
        // (*Axis).Select_exp_operation = NSOAF_HIGH_SPEED_OPERATION;
        // (*Axis).Select_exp_operation = NSOAF_RAMP_SPEED_OPERATION;
        (*Axis).dac_watch_stator_resistance = 1.69;

        // (*Axis).Select_exp_operation = AS_LOAD_MOTOR_CONST;
        // (*Axis).Select_exp_operation = SLESSINV_CONST_LOAD_PAA;
        // (*Axis).Select_exp_operation = 0;
    #endif

    if((*Axis).Select_exp_operation == AS_LOAD_MOTOR_CONST){
        PID_spd->OutLimit = 2.1; //2.0;
        (*Axis).Set_manual_rpm = -600;  // motoring + regeneration for low speed test
        //(*Axis).Set_manual_rpm = 0;    // motoring for high speed test
    }
    else if((*Axis).Select_exp_operation == AS_LOAD_MOTOR_RAMP){
        PID_spd->OutLimit = 0.01;
        (*Axis).Set_manual_rpm = -1200;
    }
    else if((*Axis).Select_exp_operation == NSOAF_LOW_SPEED_OPERATION){
        low_speed_operation_init();
    }
    else if((*Axis).Select_exp_operation == NSOAF_HIGH_SPEED_OPERATION){
        high_speed_operation_init();
    }
    else{
//        (*Axis).Set_manual_rpm = 0.0; //-200;
    }

    (*Axis).DAC_MAX5307_FLAG = FALSE;
    (*Axis).AD_offset_flag2 = FALSE;
    if((*Axis).Select_exp_operation == AS_LOAD_MOTOR_CONST){
        (*CTRL).g->FLAG_INVERTER_NONLINEARITY_COMPENSATION = 2; // use sigmoid as compensation
    }else{
        (*CTRL).g->FLAG_INVERTER_NONLINEARITY_COMPENSATION = INVERTER_NONLINEARITY_COMPENSATION_INIT;
    }

    /* Overwrite */
    #ifdef _XCUBE1
        (*CTRL).g->Overwrite_Voltage_DC_BUS = 180;
        (*CTRL).g->flag_overwite_voltage_dc_bus = FALSE;
        (*CTRL).g->flag_use_ecap_voltage = 0;

        //(*CTRL).g->OverwriteSpeedOutLimitDuringInit = 6.3;
        PID_spd->OutLimit = (*Axis).OverwriteSpeedOutLimitDuringInit;
    #else
        //(*CTRL).g->OverwriteSpeedOutLimitDuringInit = 2; // 6.3; // 150% rated
        PID_spd->OutLimit = (*Axis).OverwriteSpeedOutLimitDuringInit;
    #endif




    /* 750W MOTOR1 (wo/ hall) */
    //(*CTRL).motor->R = 1.6;
    //(*CTRL).motor->KE = 0.095;
    //PID_spd->OutLimit = 10;

    #ifdef _XCUBE1
                                     // no DT comp, comp, comp, comp
        (*CTRL).motor->R = 1.4;      // 1.39377081; 1.57862532; 1.41320038, 1.40607429
        (*CTRL).motor->KE = 0.10378; // 0.122107916; 0.103980549, 0.103777379, 0.103794798
        (*CTRL).motor->Js = 0.0007460128; // 0.000751407992; 0.000751283951, 0.000746554753, 0.000740199641
        (*CTRL).motor->Ld = (*CTRL).motor->Lq = 0.0108864028;


        // tune at 10 rpm sensored
        (*CTRL).motor->R = 1.7;      // from fitting
        AFEOE.ActiveFlux_KP = 200; // too small, CM correction not effective hence steady state position error; too large, no effect.
        AFEOE.ActiveFlux_KI = 200; // too small, no stable; too large causes steady state position error.
        (*CTRL).motor->KE = 0.103; // this is effective to tune steady state position error but position speed and negative speed have two different optimal values.
        //(*CTRL).motor->KE = 0.14; // for negative speed
    #else

    //        (*CTRL).motor->R = 1.69;      // from fitting
    //        (*CTRL).motor->KE = 0.0968527496;
    //        (*CTRL).motor->Js = 0.000767020276;
    //
    //        //(*CTRL).motor->R = 2.0;      // from low speed position error correction at 20 rpm for slow reversal. id=2
    //        //(*CTRL).motor->R = 2.4;      // from low speed position error correction at 20 rpm for slow reversal. iq=2
    //
    //        (*CTRL).motor->R = 1.85; //for negative speed [id?, iq=2 A]
    //        (*CTRL).motor->R = 2.20; //for positive speed [id?, iq=2 A]
    //
    //        (*CTRL).motor->R = 1.95; //for negative speed [no id, iq=2 A]
    //        (*CTRL).motor->R = 2.30; //for positive speed [no id, iq=2 A]
    //
    //        /* Tuning tips:
    //         * 1. KP cannot be smaller than 50, for example, KP=40 will make sensorless system unstable at 400 rpm with load.
    //         * 2. KP can be as large as 150 without any issue at 400 rpm wiht load.
    //         * 3. KI can be changed in a wide range without any impact on sensorless performance.
    //         * 4. KP has Speed dependency: lower than 100 rpm, KP can be as low as 20, and the sensorless system is stable with slow reversal.
    //         * */
    //        AFEOE.ActiveFlux_KP = 50; // for R=1.7, larger KP causes larger pos error (KP=30 went unstable), but with correct R, large KP can be used.
    //        AFEOE.ActiveFlux_KI = 25;
    //
    //        /* ONLy Kp */
    //        AFEOE.ActiveFlux_KP = 200;
    //        AFEOE.ActiveFlux_KI = 0.0;

    #endif

    //    #if TRUE
    //        MOTOR.KE = 0.095;
    //        MOTOR.KE = 0.098; // 2021-07-28 �綯ģʽ��a2/a3��ʶʱ�����أ�1.5A��ʱhtz�۲������ֵ���εİ����߲��Შ������Ҫ��KE��0.095 Wb���ӵ�0.098 Wb ��0.099 Wb�������������֣���������FFT���ĸ�ֵ�������Ʒ�ֵ��������С����
    ////        MOTOR.KE = 0.101; // 2021-07-28 �綯ģʽ����������KE��0.101��ʱ��htz.u_offset[0]���ٲ����������ڲ�ͬ�ĸ��أ�1.5A��3A���£���Ȼ����һ��ˮƽ�ߣ�
    ////        MOTOR.KE = 0.103; // 2021-07-28 �綯ģʽ�����أ�500rpm��htz.u_offset[0]alpha������KE=0.101����������в��������ӵ�0.102������С�����ӵ�0.103���ˮƽ�ߡ���ע���ʱ��1.5Ts��ʱУ������
    ////        MOTOR.KE = 0.105; // 2021-07-28 �綯ģʽ�����أ�500rpm��htz.u_offset[0]alpha������KE=0.103����������в��������ӵ�0.104������С�����ӵ�0.105���ˮƽ�ߡ���ע���ʱû��1.5Ts��ʱУ������
    ////        MOTOR.KE = 0.108; // 2021-07-29 ���ô�����ֵ���ȥȷ�����ᵼ��htz.u_offset[0]������KE������ֵΪ0.108
    //        huwu.limiter_KE = 1.0 * MOTOR.KE; // this depends on KE value
    //    #endif
    //    AFEOE.limiter_KE = 1.15 * MOTOR.KE; // this depends on KE value

    /* SlessInv */
    FE.htz.psi_aster_max = (*CTRL).motor->KE - 0.003;

    // for debug
    (*CTRL).s->PSD_Done = FALSE;
    #if ENABLE_COMMISSIONING
        COMM.bool_comm_status = 0;
    #endif
}

void runtime_command_and_tuning(){
    if((*Axis).Select_exp_operation == AS_LOAD_MOTOR_CONST){
        (*CTRL).s->go_sensorless = 0;
        G.FLAG_INVERTER_NONLINEARITY_COMPENSATION = 0;
        if((*CTRL).timebase < 2){
            PID_spd->OutLimit = 0.1;
        }else{
            //PID_spd->OutLimit = 4.2;
            PID_spd->OutLimit = 3.0;
            //PID_spd->OutLimit = 1.5;

            if((*CTRL).timebase>60){
                // Slessinv: Variable Load Experiment
                if( ((long int)((*CTRL).timebase*0.125)) % 2 == 0){ // 0.125 means 8 sec period
                    PID_spd->OutLimit = 1.5; // original
                    //PID_spd->OutLimit = 4.2; // compare with Park & Sul
                }else{
                    PID_spd->OutLimit = 3.0;
                }
            }
        }
    }
    if((*Axis).Select_exp_operation == AS_LOAD_MOTOR_RAMP){
        (*CTRL).s->go_sensorless = 0;
        if((*CTRL).timebase < 0.4){
            PID_spd->OutLimit = 4.2;
        }else if((*CTRL).timebase < 0.4 + 0.1){
            PID_spd->OutLimit -= CL_TS * 4.2 / 0.1;
        }else{
            PID_spd->OutLimit = 0.01;
        }
        //        if((*CTRL).timebase < 0.4){
        //            PID_spd->OutLimit += CL_TS * 4.2 / 0.4;
        //        }else if((*CTRL).timebase < 1.0){
        //            PID_spd->OutLimit = 4.2;
        //        }else{
        //            PID_spd->OutLimit = 0.01;
        //        }
    }


    /* Inverter Model Parameter Adaptation */
    if((*Axis).Select_exp_operation == SLESSINV_CONST_LOAD_PAA){
        if((*CTRL).timebase>11){
            ;
        }else if((*CTRL).timebase>10){
            INV.gamma_a2 = 800;
            INV.gamma_a3 = 200;
        }else{
            INV.gamma_a2 = 0; // exp 1: vary Vdc
            INV.gamma_a3 = 0;
            //INV.gamma_a2 = 800; // exp 2: vary load
            //INV.gamma_a3 = 200;
        }
    }

    /* Low Speed Operation*/
    if((*Axis).Select_exp_operation == NSOAF_LOW_SPEED_OPERATION){
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
    if((*Axis).Select_exp_operation == NSOAF_HIGH_SPEED_OPERATION){
        high_speed_operation();
        high_speed_operation_tuning();
    }

    /* Ramp High Speed Operation */
    if((*Axis).Select_exp_operation == NSOAF_RAMP_SPEED_OPERATION){
        ramp_speed_operation();
        //ramp_speed_operation_tuning();
    }

    /* ���ּ� */
    if((*CTRL).motor->KE > 0.2){
        (*CTRL).motor->KE=0.1;
    }
    if((*Axis).Set_manual_rpm>3000){
        (*Axis).Set_manual_rpm=3000;
    }
    if((*Axis).Set_manual_rpm<-3000){
        (*Axis).Set_manual_rpm=3000;
    }
}


/* NSOAF */

#define SLOW_REVERSAL_RATE 50 // Park.Sul2014 is 90

void short_stopping_at_zero_speed(){

    #define RPM1 100
    #define BIAS 0
    if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        (*Axis).Set_manual_rpm = 0;
    }else if((*CTRL).timebase<5){
        (*Axis).Set_manual_rpm = RPM1;
    }else if((*CTRL).timebase<10){
        (*Axis).Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm < 0){
            (*Axis).Set_manual_rpm = 0.0;
        }
    }else if((*CTRL).timebase<15){
        (*Axis).Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm<-RPM1){
            (*Axis).Set_manual_rpm = -RPM1;
        }
    }else if((*CTRL).timebase<20){
        (*Axis).Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm > 0){
            (*Axis).Set_manual_rpm = 0.0;
        }
    }else if((*CTRL).timebase<25){
        (*Axis).Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm>RPM1){
            (*Axis).Set_manual_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void slow_speed_reversal(){

    #define RPM1 100
    #define BIAS 0
    if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        (*Axis).Set_manual_rpm = RPM1; // 0;
    }else if((*CTRL).timebase<4-2){
        (*Axis).Set_manual_rpm = RPM1;
    }else if((*CTRL).timebase<10-2){
        (*Axis).Set_manual_rpm += CL_TS * -SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm<-RPM1){
            (*Axis).Set_manual_rpm = -RPM1;
        }
    }else if((*CTRL).timebase<16-2){
        (*Axis).Set_manual_rpm += CL_TS * +SLOW_REVERSAL_RATE;
        if((*Axis).Set_manual_rpm>RPM1){
            (*Axis).Set_manual_rpm = RPM1;
        }
    }

    #undef RPM1
    #undef BIAS
}

void low_speed_operation_init(){
    //    PID_spd->OutLimit = 10;

        //    (*CTRL).motor->KE = 0.104; // ZFY: 0.1A
        //    (*CTRL).motor->KE = 0.112;  // ZFY: 2.0A
        //    (*CTRL).motor->KE = 0.15;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=180V (offset_Udc 14V)
    //    (*CTRL).motor->KE = 0.10;  // ZFY: 2.0A Slow speed reversal at 100 rpm @ Udc=80V (offset_Udc 12V)
}

/*when AS_LOAD_MOTOR_CONST use (*Axis).Set_manual_rpm =-300; [rpm], iq is negative*/
//REAL RP =2.15; // 2.15 for cold motor, 2.20 for hot motor
//REAL RN =1.95; // 1.95 for cold motor, 2.00 for hot motor
/*when AS_LOAD_MOTOR_CONST use (*Axis).Set_manual_rpm = 300; [rpm], iq is positive*/

/* HUWU--SSR */
REAL RSmall =1.95; // motoring is smaller
REAL RLarge =2.15; // regenerating

/* CM-VM Fusion--SSR */
//REAL RSmall =2.00; // motoring is smaller
//REAL RLarge =2.20; // regenerating

void slow_speed_reversal_tuning(){

    if((*CTRL).timebase<30){
        /* ʧȥ�ų������������ת�ӵ����� */
        if(G.flag_auto_id_cmd == TRUE){
            // need to also set id cmd according to speed cmd
            if(fabs((*Axis).Set_manual_rpm) < 50){ // 5 or 50?
                (*Axis).Set_manual_current_id = 2;
            }else{
                (*Axis).Set_manual_current_id = 0;
            }
        }

        /* Steady state error */
        //        if((*CTRL).g->flag_use_ecap_voltage==0){
        //            if((*Axis).Set_manual_rpm>0){
        //                (*CTRL).motor->R = RP; //for positive speed
        //            }else{
        //                (*CTRL).motor->R = RN; //for negative speed
        //            }
        //        }else{
        //            RP = 2.20;
        //            RN = 1.80;
        //            if((*Axis).Set_manual_rpm>0){
        //                (*CTRL).motor->R = RP; //for positive speed
        //            }else{
        //                (*CTRL).motor->R = RN; //for negative speed
        //            }
        //        }

        if((*CTRL).s->Motor_or_Generator){
            (*CTRL).motor->R = RSmall; //for positive speed
        }else{
            (*CTRL).motor->R = RLarge; //for negative speed
        }

        //    if((*Axis).Set_manual_rpm<-10){
        //
        //        (*CTRL).motor->KE = 0.14; // for negative speed
        //        //(*CTRL).motor->KE = 0.085; //2.1A
        //
        //    }else if ((*Axis).Set_manual_rpm>10){
        //
        //        (*CTRL).motor->KE = 0.103; // this is effective to tune steady state position error but position speed and negative speed have two different optimal values.
        //        //(*CTRL).motor->KE = 0.11; // 2.1A
        //
        //    }else{
        //        (*CTRL).motor->KE = 0.103;
        //    }
    }

}
void zero_speed_stopping_tuning(){

    /* ʧȥ�ų������������ת�ӵ����� */
    // need to also set id cmd according to speed cmd
    if(fabs((*Axis).Set_manual_rpm) < 50){ // 5 or 50?
        (*Axis).Set_manual_current_id = 2;
    }else{
        //(*Axis).Set_manual_current_id = 0;
    }

    /* Steady state error */
    if((*Axis).Set_manual_rpm<-5){
        (*CTRL).motor->KE = 0.082; //2.1A
    }else if ((*Axis).Set_manual_rpm>5){
        (*CTRL).motor->KE = 0.118; // 2.1A
    }else{
        (*CTRL).motor->KE = 0.10;
    }

}
void zero_speed_stopping(){
    #define RPM1 100
    #define BIAS 0
    if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
        (*Axis).Set_manual_rpm = 0;
    }else if((*CTRL).timebase<4){
        (*Axis).Set_manual_rpm = RPM1;
    }else if((*CTRL).timebase<6+BIAS){
        (*Axis).Set_manual_rpm = -RPM1;
    }else if((*CTRL).timebase<9+BIAS){
        (*Axis).Set_manual_rpm = 0;
    }else if((*CTRL).timebase<18+BIAS){
        (*Axis).Set_manual_rpm = RPM1;
    }else if((*CTRL).timebase<25+BIAS){
        (*Axis).Set_manual_rpm = RPM1*sin(2*2*M_PI*(*CTRL).timebase);
    }

    #undef RPM1
    #undef BIAS
}

void high_speed_operation_init(){
    //    PID_spd->OutLimit = 10; // sensorless needs higher bounds (does not work well with nonlinear controller gain)
    //    (*CTRL).motor->KE = 0.095;
    //    AFEOE.ActiveFlux_KP = 0.2; // nsoaf.ActiveFlux_KPȡ0.5�����ϵ�ʱ��300rpm�����active flux������������
    //    AFEOE.ActiveFlux_KI = 2.0;
}
int manual_adjust_KE=FALSE;
void high_speed_operation_tuning(){

    #if SELECT_ALGORITHM == ALG_Chi_Xu
    #elif SELECT_ALGORITHM == ALG_NSOAF
        // Original Submission
        /* ��̬���ܺ�����KP��ֵ������� */
            //    nsoaf.KP = fabs((*Axis).Set_manual_rpm) / 1500.0 * 4;
            //    if(nsoaf.KP<1){
            //        nsoaf.KP = 1;
            //    }
        //        if(fabs((*Axis).Set_manual_rpm) > 1000){
        //            nsoaf.KP = 4; //2;
        //            AFEOE.ActiveFlux_KP = 1;
        //        }else if(fabs((*Axis).Set_manual_rpm) > 500){
        //            nsoaf.KP = 2; //1.5;
        //            AFEOE.ActiveFlux_KP = 0.5;
        //        }else if(fabs((*Axis).Set_manual_rpm) > 300){
        //            nsoaf.KP = 1;
        //            AFEOE.ActiveFlux_KP = 0.2;
        //        }else{
        //            AFEOE.ActiveFlux_KP = 0.1;
        //        }

        /* Steady state rpm error when ZFY speed command is at 0 rpm (load sudden change when speed command sign changes) */
        //    if((*Axis).Set_manual_rpm > 0){
        //        (*CTRL).motor->KE=0.097;
        //    }else{
        //        (*CTRL).motor->KE=0.099;
        //    }

        /* Steady state rpm error when ZFY speed command is at 2000 rpm. */
    if(manual_adjust_KE==FALSE){
        if((*Axis).Set_manual_rpm > 0){
            //(*CTRL).motor->KE = 0.0895; // positive spinning with load motor@2000 rpm 2.1A
            (*CTRL).motor->KE = 0.0895;
        }else{
            //(*CTRL).motor->KE=0.089;  // negative positive spinning with load motor@2000 rpm 2.1A
            (*CTRL).motor->KE=  0.0890500024;
        }
    }
    #endif

    /* ��ֹ�ּ������ֵ */
    if((*CTRL).motor->KE > 0.2){
        (*CTRL).motor->KE = 0.1;
    }
}
REAL some_speed_variable = 0.0;
void high_speed_operation(){

    #define RPM1 1500
    #define BIAS 0
    //    if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
    //        (*Axis).Set_manual_rpm = 100;
    //    }else if((*CTRL).timebase<1.5){
    //        (*Axis).Set_manual_rpm = RPM1;
    //    }else if((*CTRL).timebase<2.0+BIAS){
    //        (*Axis).Set_manual_rpm = -RPM1;
    //    }else if((*CTRL).timebase<2.5+BIAS){
    //        (*Axis).Set_manual_rpm = RPM1;
    //    }else if((*CTRL).timebase<3.0+BIAS){
    //        (*Axis).Set_manual_rpm = -100;
    //    }

    if((*CTRL).timebase<2){ // note 1 sec is not enough for stator flux to reach steady state.
        (*Axis).Set_manual_rpm = 500; // give human more time to setup iq limit and load motor speed
    }else if((*CTRL).timebase<4.5){
        (*Axis).Set_manual_rpm = RPM1;
    }else if((*CTRL).timebase<5.0+BIAS){
        (*Axis).Set_manual_rpm = -RPM1;
    }else if((*CTRL).timebase<5.5+BIAS){
        (*Axis).Set_manual_rpm = RPM1;
    }

    // Orignal Submission
    //    if((*CTRL).timebase<10){ // note 1 sec is not enough for stator flux to reach steady state.
    //        (*Axis).Set_manual_rpm = 300; // give human more time to setup iq limit and load motor speed
    //    }else if((*CTRL).timebase<10.5){
    //        (*Axis).Set_manual_rpm = RPM1;
    //    }else if((*CTRL).timebase<11.0+BIAS){
    //        (*Axis).Set_manual_rpm = -RPM1;
    //    }else if((*CTRL).timebase<11.5+BIAS){
    //        (*Axis).Set_manual_rpm = RPM1;
    //    }
    some_speed_variable = (*Axis).Set_manual_rpm;
    #undef RPM1
    #undef BIAS
}


void ramp_speed_operation(){
    if(FALSE){
        #define FAST_REVERSAL_RATE 500
        #define RPM1 1500
        if((*CTRL).timebase<1){
            (*Axis).Set_manual_rpm = 500;
        }else if((*CTRL).timebase<2){
            (*Axis).Set_manual_rpm = RPM1;
        }else if((*CTRL).timebase<10){
            (*Axis).Set_manual_rpm += CL_TS * -FAST_REVERSAL_RATE;
            if((*Axis).Set_manual_rpm<-RPM1){
                (*Axis).Set_manual_rpm = -RPM1;
            }
        }else if((*CTRL).timebase<18){
            (*Axis).Set_manual_rpm += CL_TS * +FAST_REVERSAL_RATE;
            if((*Axis).Set_manual_rpm>RPM1){
                (*Axis).Set_manual_rpm = RPM1;
            }
        }
        #undef RPM1
        #undef FAST_REVERSAL_RATE
    }else{
        #define FAST_REVERSAL_RATE 1500
        #define RPM1 1500
        if((*CTRL).timebase<1){
            (*Axis).Set_manual_rpm = 500;
        }else if((*CTRL).timebase<2){
            (*Axis).Set_manual_rpm = RPM1;
        }else if((*CTRL).timebase<6){
            (*Axis).Set_manual_rpm += CL_TS * -FAST_REVERSAL_RATE;
            if((*Axis).Set_manual_rpm<-RPM1){
                (*Axis).Set_manual_rpm = -RPM1;
            }
        }else if((*CTRL).timebase<10){
            (*Axis).Set_manual_rpm += CL_TS * +FAST_REVERSAL_RATE;
            if((*Axis).Set_manual_rpm>RPM1){
                (*Axis).Set_manual_rpm = RPM1;
            }
        }
        #undef RPM1
        #undef FAST_REVERSAL_RATE
    }
}
