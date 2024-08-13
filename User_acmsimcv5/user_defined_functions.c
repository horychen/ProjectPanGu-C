// user_defined_functions.c
#include <ACMSim.h>

/* 管仲焘在用的东西 */
    #if WHO_IS_USER==USER_BEZIER
        #include "Bezier.h"
        BezierController BzController;
    #endif

/* 吴波 */
    #if WHO_IS_USER==USER_WUBO
        #define APPLY_WCTUNER TRUE // 是否使用新的调参器
    #endif

void _user_init(){
    // #define MODE_SELECT_PWM_DIRECT 1
    // #define MODE_SELECT_VOLTAGE_OPEN_LOOP 11
    // #define MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE 2
    // #define MODE_SELECT_FOC 3
    // #define MODE_SELECT_FOC_SENSORLESS 31
    // #define MODE_SELECT_VELOCITY_LOOP 4
    // #define MODE_SELECT_VELOCITY_LOOP_SENSORLESS 41
    // #define MODE_SELECT_TESTING_SENSORLESS 42
    // #define MODE_SELECT_POSITION_LOOP 5
    // #define MODE_SELECT_COMMISSIONING 9

    debug.error = 0.0;
    debug.mode_select = MODE_SELECT_VELOCITY_LOOP;
    // debug.mode_select = MODE_SELECT_FOC;
    // debug.mode_select = MODE_SELECT_VOLTAGE_OPEN_LOOP;


    // debug.flag_overwrite_theta_d = 0.0;
    debug.Overwrite_Current_Frequency = 0.0;

    debug.set_deg_position_command = 0.0;
    debug.set_rpm_speed_command = -300;
    debug.set_iq_command =  -1.0;
    debug.set_id_command = 1*-1.0;


    #if WHO_IS_USER == USER_BEZIER
        set_points(&BzController);
    #endif
}

void _user_commands(){

    #if WHO_IS_USER == USER_BEZIER
        if ((*CTRL).timebase > CL_TS){
            (*CTRL).i->cmd_varOmega = d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_step_command){
            (*CTRL).i->cmd_varOmega = -d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance){
            #if PC_SIMULATION
                ACM.TLoad = 2;
            #else
                //CTRL_2.i->cmd_iDQ[1] = 0.3;
            #endif
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance+0.1){
            // break;
        }
    #elif
        (*CTRL).i->cmd_varOmega = debug.set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
    #endif
}

void _user_controller(){

    /* 更新依赖于dq轴电流的物理量 */
    (*CTRL).i->Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0]) * (*CTRL).i->iDQ[1];     // 转矩 For luenberger position observer for HFSI
    (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0]) * (*CTRL).i->cmd_iDQ[1];
    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];

    // 用户指令
    _user_commands();

    /// 5. 转速环（使用反馈转速）
    if ((*CTRL).s->the_vc_count++ >= SPEED_LOOP_CEILING){
        (*CTRL).s->the_vc_count = 1;
        PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
        PID_Speed->Fbk = (*CTRL).i->varOmega;
        #if WHO_IS_USER == USER_BEZIER
            control_output(PID_Speed, &BzController);
        #elif WHO_IS_USER == USE_WUBVO
            PID_Speed->calc(&PID_Speed);
        #else
            PID_Speed->calc(&PID_Speed);
        #endif
        (*CTRL).i->cmd_iDQ[1] = PID_Speed->Out;
    }

    /// 6. 电流环
    // d-axis
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    PID_iD->calc(&PID_iD);
    // q-axis
    PID_iQ->Fbk = (*CTRL).i->iDQ[1];
    PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
    PID_iQ->calc(&PID_iQ);

    // 电流环前馈DQ轴解耦
    #if VOLTAGE_CURRENT_DECOUPLING_CIRCUIT == TRUE
        REAL decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->omg_elec;
        REAL decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KE + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->omg_elec;
    #else
        REAL decoupled_d_axis_voltage = PID_iD->Out;
        REAL decoupled_q_axis_voltage = PID_iQ->Out;
    #endif

    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    /// 7. 反帕克变换
    // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
    // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
    // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5*(*CTRL).i->omg_elec*CL_TS);
    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

    // (*CTRL).o->cmd_uAB[0+2] = MT2A((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);
    // (*CTRL).o->cmd_uAB[1+2] = MT2B((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);


    /// 8. 补偿逆变器非线性
    main_inverter_voltage_command(TRUE);
    (*CTRL).o->cmd_uAB_to_inverter[0];
    (*CTRL).o->cmd_uAB_to_inverter[1];
}

void _user_observer(REAL iAB[2]){

    /// 3. 调用观测器：估计的电气转子位置和电气转子转速反馈
    // observers();
    if (debug.SENSORLESS_CONTROL == TRUE)
    {
        // （无感）
        (*CTRL).i->varOmega = ELECTRICAL_SPEED_FEEDBACK;        // harnefors.omg_elec;
        (*CTRL).i->theta_d_elec = ELECTRICAL_POSITION_FEEDBACK; // harnefors.theta_d;
    }
}


void _user_time_varying_parameters(){
    // ACM;
    /// 0. 参数时变
    // if (fabs((*CTRL).timebase-2.0)<CL_TS){
    //     printf("[Runtime] Rotor resistance of the simulated IM has changed!\n");
    //     ACM.alpha = 0.5*IM_ROTOR_RESISTANCE / IM_MAGNETIZING_INDUCTANCE;
    //     ACM.rreq = ACM.alpha*ACM.Lmu;
    //     ACM.rr   = ACM.alpha*(ACM.Lm+ACM.Llr);
    // }
}


// 定义特定的测试指令，如快速反转等
#if 0 // defined in Experiment.c???
    void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd){
        if(timebase > instant+2*interval){
            debug.set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else if(timebase > instant+interval){
            debug.set_rpm_speed_command = 1*1500 + -rpm_cmd;
        }else if(timebase > instant){
            debug.set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else{
            debug.set_rpm_speed_command = 20; // default initial command
        }
    }

    REAL short_stopping_at_zero_speed(){
        static REAL set_rpm_speed_command=0.0;

        #define RPM1 100
        #define BIAS 0
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<5){
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<10){
            set_rpm_speed_command += CL_TS * -50;
            if(set_rpm_speed_command < 0){
                set_rpm_speed_command = 0.0;
            }
        }else if((*CTRL).timebase<15){
            set_rpm_speed_command += CL_TS * -50;
            if(set_rpm_speed_command<-RPM1){
                set_rpm_speed_command = -RPM1;
            }
        }else if((*CTRL).timebase<20){
            set_rpm_speed_command += CL_TS * +50;
            if(set_rpm_speed_command > 0){
                set_rpm_speed_command = 0.0;
            }
        }else if((*CTRL).timebase<25){
            set_rpm_speed_command += CL_TS * +50;
            if(set_rpm_speed_command>RPM1){
                set_rpm_speed_command = RPM1;
            }
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL slow_speed_reversal(REAL slope){ // slope = 20 rpm/s, 50 rpm/s
        static REAL set_rpm_speed_command=0.0;

        #define RPM1 100
        #define BIAS 0
        if((*CTRL).timebase<0.5){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<1){
            set_rpm_speed_command = -50;
        }else if((*CTRL).timebase<4){
            set_rpm_speed_command = RPM1+50;
        }else if((*CTRL).timebase<15){
            set_rpm_speed_command += CL_TS * -slope;
            if(set_rpm_speed_command<-RPM1){
                set_rpm_speed_command = -RPM1;
            }
        }else if((*CTRL).timebase<25){
            set_rpm_speed_command += CL_TS * +slope;
            if(set_rpm_speed_command>RPM1){
                set_rpm_speed_command = RPM1;
            }
        }

        if((*CTRL).timebase>25 && (*CTRL).timebase<35){
            set_rpm_speed_command = RPM1*2;
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL low_speed_operation(){
        REAL set_rpm_speed_command;

        #define RPM1 200
        #define BIAS 50
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<3){
            set_rpm_speed_command = RPM1 + BIAS;
        }else if((*CTRL).timebase<6+BIAS){
            set_rpm_speed_command = -RPM1;
        }else if((*CTRL).timebase<9+BIAS){
            set_rpm_speed_command = 10;
        }else if((*CTRL).timebase<12+BIAS){
            set_rpm_speed_command = RPM1*sin(20*2*M_PI*(*CTRL).timebase);
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL high_speed_operation(){
        REAL set_rpm_speed_command;

        #define RPM1 1500
        #define BIAS 0
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<4){
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<6+BIAS){
            set_rpm_speed_command = -RPM1;
        }else if((*CTRL).timebase<9+BIAS){
            set_rpm_speed_command = 10;
        }else if((*CTRL).timebase<12+BIAS){
            set_rpm_speed_command = RPM1*sin(2*2*M_PI*(*CTRL).timebase);
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }
#endif


struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};


REAL _user_load_model(){
    static double Tload = 0.0;
    // static int load_state = 0;
    // static double dc_part = LOAD_TORQUE;
    // static double viscous_part = 0.0; // 这个变量去到Config文件里面了
    // viscous_part = VISCOUS_COEFF*ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
    // Tload = dc_part + viscous_part;
    if(CTRL_1.timebase > 0.5){
        Tload = 0.5;
    }
    return Tload;
}


// /* --------------------------下面的是从 pmsm_controller.c 挪过来的公用逆变器死区电压辨识&补偿代码 */
//     #define LUT_N_LC  70
//     #define LUT_N_HC  29
//     REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
//     REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
//     #define LUT_STEPSIZE_BIG 0.11641244037931034
//     #define LUT_STEPSIZE_SMALL 0.01237159786376811
//     #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
//     #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
//     #define LUT_I_TURNING_LC 0.8660118504637677
//     #define LUT_I_TURNING_HC 4.241972621463768
//     #define V_PLATEAU 7.43925517763064
// REAL lookup_compensation_voltage_indexed(REAL current_value){
//     REAL abs_current_value = fabs(current_value);

//     if(abs_current_value < LUT_I_TURNING_LC){
//         REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
//         int index = (int)float_index;
//         REAL slope;
//         if(index+1 >= LUT_N_LC)
//             slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         else
//             slope = (lut_lc_voltage[index+1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL));
//     }else{
//         REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
//         int index = (int)float_index; // THIS IS A RELATIVE INDEX!
//         REAL slope;
//         if(index+1 >= LUT_N_HC)
//             return V_PLATEAU;
//         else
//             slope = (lut_hc_voltage[index+1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
//         return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index*LUT_STEPSIZE_BIG));
//     }
// }
// int test_lookup_compensation_voltage_indexed(){
//     int i=0;
//     while(TRUE){
//         i+=1;
//         printf("%g, %g\n", lookup_compensation_voltage_indexed(0.02*i), 0.02*i);
//         if(i>100)
//             break;
//     }
//     return 0;

// }
