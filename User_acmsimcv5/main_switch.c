// user_defined_functions.c
#include <ACMSim.h>

REAL global_id_ampl = 1.5;
REAL global_id_freq = 1.0;

REAL global_id_freq_ampl = 1;
REAL global_id_freq_freq_ampl = 1;

REAL global_id_freq_freq = 0.1;
REAL global_id_freq_freq_freq = 4.99999987e-06;

REAL local_id_commmand = 0.0;

REAL tmp = 0.0;
REAL tmp_id_freq = 0.0;
REAL tmp_id_freq_freq = 0.0;

REAL global_uD_ampl = 0.0;
REAL global_uD_square_time = 1; // second
REAL gloval_uD_square_time_sum = 0.0; //
REAL omega_2_current_gain = -0.06;
int index_1 = 0;
REAL count_1 = 0;
REAL find_max_speed[100] = {0.0};
REAL find_max_parameter[100] = {0.0};
REAL sum_omega = 0;
REAL ave_omega = 0;
REAL sum_cnt = 0;
void overwrite_d_sim(){
    // for now 20240902 do nothing
}

void _user_init(){

    init_d_sim();   // initilizating d_sim is removed into main.c to run only once
    overwrite_d_sim(); // overwrite d_sim with user's algorithm
    init_CTRL_Part1();

    if ((*debug).bool_initilized == FALSE){

        // (*debug).use_first_set_three_phase = 1;
        (*debug).bool_initilized = TRUE;
        (*debug).error = 0;
        (*debug).who_is_user = d_sim.user.who_is_user;
        if(CTRL->motor->Rreq>0){
            (*debug).mode_select = d_sim.user.mode_select_induction_motor;
        }else{
            (*debug).mode_select = d_sim.user.mode_select_synchronous_motor;
        }
            /* Open Loop  */
            // (*debug).mode_select = MODE_SELECT_PWM_DIRECT;                            //  1
            // (*debug).mode_select = MODE_SELECT_VOLTAGE_OPEN_LOOP;                     // 11
            /*  Without the Encoder */
            // (*debug).mode_select = MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE; //  2
            /* FOC  */
            // (*debug).mode_select = MODE_SELECT_FOC;                                   //  3
            // (*debug).mode_select = MODE_SELECT_FOC_SENSORLESS;                        // 31
            // (*debug).mode_select = MODE_SELECT_INDIRECT_FOC;                          // 32
            /* Speed Loop  */
            // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP;                         //  4
            // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP_SENSORLESS;              // 41
            // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP_WC_TUNER;                // 43
            /* Position Loop  */
            // (*debug).mode_select = MODE_SELECT_POSITION_LOOP;                         //  5
            /* Commission  */
            // (*debug).mode_select = MODE_SELECT_COMMISSIONING;                         //  9

        (*debug).Overwrite_Current_Frequency = 0;
        (*debug).Overwrite_theta_d           = 0.0;

        (*debug).set_id_command              = 0;
        (*debug).set_iq_command              = 0;
        (*debug).set_rpm_speed_command       = 100;
        (*debug).set_deg_position_command    = 0.0;
        (*debug).vvvf_voltage = 3.0;
        (*debug).vvvf_frequency = 5.0;

        (*debug).delta                                                = d_sim.FOC.delta;
        (*debug).CLBW_HZ                                              = d_sim.FOC.CLBW_HZ;
        (*debug).VL_EXE_PER_CL_EXE                                    = d_sim.FOC.VL_EXE_PER_CL_EXE;
        (*debug).LIMIT_DC_BUS_UTILIZATION                             = d_sim.CL.LIMIT_DC_BUS_UTILIZATION;
        (*debug).LIMIT_OVERLOAD_FACTOR                                = d_sim.VL.LIMIT_OVERLOAD_FACTOR;
        (*debug).Select_exp_operation                                 = d_sim.user.Select_exp_operation;
        (*debug).bool_apply_decoupling_voltages_to_current_regulation = d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation;
        (*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT = d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD;
    }

    // (*debug).INVERTER_NONLINEARITY = 0;
    #if WHO_IS_USER == USER_YZZ
        (*debug).SENSORLESS_CONTROL      = 0;
        (*debug).SENSORLESS_CONTROL_HFSI = 0;
        (*debug).BOOL_FOC_SENSORLESS_MODE_OPEN = 0;
    #endif

    #if WHO_IS_USER == USER_BEZIER
        set_points(&BzController);
    #endif

    #if WHO_IS_USER == 2023231051
        //For WuBo
        (*debug).zeta                                                 = d_sim.user.zeta;
        (*debug).omega_n                                              = d_sim.user.omega_n;
        (*debug).max_CLBW_PER_min_CLBW                                = d_sim.user.max_CLBW_PER_min_CLBW;
        (*debug).bool_apply_WC_tunner_for_speed_loop                  = d_sim.user.bool_apply_WC_tunner_for_speed_loop;
        (*debug).bool_sweeping_frequency_for_speed_loop               = d_sim.user.bool_sweeping_frequency_for_speed_loop;
        (*debug).bool_Null_D_Control                                  = d_sim.user.bool_Null_D_Control;
        (*debug).bool_apply_sweeping_frequency_excitation             = d_sim.user.bool_apply_sweeping_frequency_excitation;
        //For Sweeping
        (*debug).CMD_CURRENT_SINE_AMPERE                              = d_sim.user.CMD_CURRENT_SINE_AMPERE;
        (*debug).CMD_SPEED_SINE_RPM                                   = d_sim.user.CMD_SPEED_SINE_RPM;
        (*debug).CMD_SPEED_SINE_HZ                                    = d_sim.user.CMD_SPEED_SINE_HZ;
        (*debug).CMD_SPEED_SINE_STEP_SIZE                             = d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
        (*debug).CMD_SPEED_SINE_LAST_END_TIME                         = d_sim.user.CMD_SPEED_SINE_LAST_END_TIME;
        (*debug).CMD_SPEED_SINE_END_TIME                              = d_sim.user.CMD_SPEED_SINE_END_TIME;
        (*debug).CMD_SPEED_SINE_HZ_CEILING                            = d_sim.user.CMD_SPEED_SINE_HZ_CEILING;
    #endif
}

void _user_commands(){
    /* RPM GIVEN */
    (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;

    if (CTRL->motor->Rreq > 0){
        // 感应电机需要励磁
        (*CTRL).i->cmd_iDQ[0] = 2.0;

    }else{
        // 表贴永磁采用 iD=0 控制
        (*CTRL).i->cmd_iDQ[0] = 0.0;
        // (*CTRL).i->cmd_iDQ[0] = -20.0;

        // 凸极永磁采用 iD<0 获得更大的 有功磁链（aka 转矩系数）
        // (*CTRL).i->cmd_iDQ[0] = -1.0;
    }

    #if PC_SIMULATION == TRUE
        #if WHO_IS_USER == USER_BEZIER
            if ((*CTRL).timebase > CL_TS){
                (*CTRL).i->cmd_varOmega = d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > d_sim.user.bezier_seconds_step_command){
                (*CTRL).i->cmd_varOmega = -d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance){
                #if PC_SIMULATION
                    ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                #else
                    //CTRL_2.i->cmd_iDQ[1] = 0.3;
                #endif
            }
            if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance+0.1){
                // break;
            }
        #elif WHO_IS_USER == USER_WB
            // /* 加入恒负载 */
            // ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN * d_sim.VL.LIMIT_OVERLOAD_FACTOR * 0);
            // if ((*CTRL).timebase > CL_TS){
            //     (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 0.08){
            //     (*CTRL).i->cmd_varOmega = - (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 0.4){
            //     #if PC_SIMULATION
            //         ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN * d_sim.VL.LIMIT_OVERLOAD_FACTOR * 0);
            //     #endif
            // }
            (*CTRL).i->cmd_varOmega = 0.0;
            if ((*CTRL).timebase > CL_TS){
                // (*debug).set_iq_command = 0.07;
                (*CTRL).i->cmd_varOmega = - (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 1){
                // (*debug).set_iq_command = 0.02;
                (*CTRL).i->cmd_varOmega = + (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 2){
                // (*debug).set_iq_command = 0.07;
                // #if PC_SIMULATION
                //     ACM.TLoad = (0.4 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                // #endif
                (*CTRL).i->cmd_varOmega = 0.5 * (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 3){
                // #if PC_SIMULATION
                //     ACM.TLoad = (0.3 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                // #endif
                // (*debug).set_iq_command = 0;
            }
        #elif WHO_IS_USER == USER_CJH
            (*CTRL).i->cmd_varOmega = 0.0;
            if ((*CTRL).timebase > 1.0){
                (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 2){
                (*CTRL).i->cmd_varOmega = - (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 3.0){
                #if PC_SIMULATION
                    ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                #endif
            }
        #elif WHO_IS_USER == USER_YZZ
            (*CTRL).i->cmd_varOmega = 0.0;
            if ((*CTRL).timebase > CL_TS){
                (*CTRL).i->cmd_varOmega = - (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 1){
                (*CTRL).i->cmd_varOmega = + (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 2){
                #if PC_SIMULATION
                    ACM.TLoad = (0.4 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                #endif
            }
            if ((*CTRL).timebase > 3){
                #if PC_SIMULATION
                    ACM.TLoad = (0.3 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                #endif
            }
            // (*CTRL).i->cmd_varOmega = 0.0;
            // if ((*CTRL).timebase > CL_TS){
            //     // (*debug).set_iq_command = 0.07;
            //     (*CTRL).i->cmd_varOmega = - (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 1){
            //     // (*debug).set_iq_command = 0.02;
            //     (*CTRL).i->cmd_varOmega = + (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 2){
            //     // (*debug).set_iq_command = 0.07;
            //     #if PC_SIMULATION
            //         ACM.TLoad = (0.4 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
            //     #endif
            //     (*CTRL).i->cmd_varOmega = 0.5 * (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 3){
            //     // #if PC_SIMULATION
            //         ACM.TLoad = (0.3 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
            //     // #endif
            //     // (*debug).set_iq_command = 0;
            // }
        #endif
   #endif
}

int main_switch(long mode_select){
    static long mode_select_last = 0;
    static int mode_initialized = FALSE;
    #if PC_SIMULATION == FALSE
        if (axisCnt == 0) {
            CTRL = &CTRL_1;  // 根据axisCnt选择CTRL_1
        } else if (axisCnt == 1) {
            CTRL = &CTRL_2;  // 根据axisCnt选择CTRL_2
        }
    #endif

    if(mode_select != mode_select_last){
        mode_initialized = TRUE;
    }
    switch (mode_select){
    case MODE_SELECT_PWM_DIRECT: // 1
        if(mode_initialized == FALSE){
            mode_initialized = TRUE;
            (*CTRL).svgen1.Ta = 0.5;
            (*CTRL).svgen1.Tb = 0.5;
            (*CTRL).svgen1.Tc = 0.5;
        }
        return 5; // set Axis->Select_exp_operation
        break;
    case MODE_SELECT_VOLTAGE_OPEN_LOOP: // 11
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*debug).vvvf_voltage * cos((*debug).vvvf_frequency*2*M_PI* CTRL->timebase);
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*debug).vvvf_voltage * sin((*debug).vvvf_frequency*2*M_PI* CTRL->timebase);
        break;
    case MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE: // 2
        if(mode_initialized == FALSE){
            mode_initialized = TRUE;
            // TODO: add your default setup
        }
        _user_virtual_ENC();
        break;
    case MODE_SELECT_FOC: // 3
        #if WHO_IS_USER == USER_WB
            // (*debug).set_iq_command = 0;
            // (*debug).set_id_command = global_id_ampl * sin(2 * M_PI * global_id_freq * (*CTRL).timebase);
            // (*debug).set_iq_command = 0.0;
            // tmp = global_id_freq * (`*CTRL).timebase;
            // tmp -= (long)tmp;
            // // (*debug).set_id_command = global_id_ampl * sinf(2.0 * M_PI * global_id_freq * (*CTRL).timebase);
            // // local_id_commmand = global_id_ampl * sinf(2.0 * M_PI * global_id_freq * (*CTRL).timebase);
            // (*debug).set_id_command = global_id_ampl * sinf(2.0 * M_PI * tmp);
            
            //* The current command from Lao Yan's Exp
        #endif
            _user_commands();
            // (*debug).set_iq_command = (*CTRL).i->varOmega * omega_2_current_gain;
            // #if PC_SIMULATION
            //     (*debug).set_iq_command = (*CTRL).i->cmd_varOmega * omega_2_current_gain;
            // #endif
        tmp = global_id_freq * (*CTRL).timebase;
        tmp -= (long)tmp;
        (*debug).set_id_command = global_id_ampl * sinf(2.0 * M_PI * tmp);
        _user_onlyFOC();
        break;
    case MODE_SELECT_FOC_SENSORLESS : //31
        #if WHO_IS_USER == USER_YZZ
        if ((*CTRL).motor->Lq < 0.0062){
            if(count_1 >50000){
                find_max_parameter[index_1] = (*CTRL).motor->Lq;
                (*CTRL).motor->Lq  += 0.0001;
                ave_omega = sum_omega * 1e-4;
                find_max_speed[index_1]= ave_omega;
                index_1 +=1;
                count_1 = 0;
                sum_omega = 0;
                sum_cnt=0;
            }
            else if ((count_1 > 40000)&&(count_1 < 50000)){
                sum_omega += CTRL->enc->rpm;
                sum_cnt +=1;    
            }
            count_1 +=1;
        }
        // if ((*CTRL).motor->KE < 0.4){
        //     if(count_1 >100000){
        //         (*CTRL).motor->KE  += 0.01;
        //         ave_omega = sum_omega * 1e-4;
        //         find_max_speed[index_1]= ave_omega;
        //         index_1 +=1;
        //         count_1 = 0;
        //         sum_omega = 0;
        //         sum_cnt=0;
        //     }
        //     else if ((count_1 > 90000)&&(count_1 < 100000)){
        //         sum_omega += CTRL->enc->rpm;
        //         sum_cnt +=1;    
        //     }
        //     count_1 +=1;
        // }
        _user_commands(); 
        pmsm_observers();
        if ((*debug).BOOL_FOC_SENSORLESS_MODE_OPEN == TRUE){
            (*CTRL).i->theta_d_elec = FE.clfe4PMSM.theta_d;
        }
        _user_onlyFOC();
        #endif
        //TODO:
        break;
    case MODE_SELECT_INDIRECT_FOC:   // 32
        _user_commands();         // 用户指令
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        controller_IFOC();
        #endif
        break;
    case MODE_SELECT_FOC_AS_DC_GENERATOR : //33
        _user_commands(); 
            (*debug).set_iq_command = (*CTRL).i->varOmega * omega_2_current_gain;
        #if PC_SIMULATION
            (*debug).set_iq_command = (*CTRL).i->cmd_varOmega * omega_2_current_gain;
        #endif
        _user_onlyFOC();
        //TODO:
        break;
    case MODE_SELECT_VELOCITY_LOOP: // 4
        _user_commands();         // 用户指令
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
            (*CTRL).i->varOmega,
            (*CTRL).i->cmd_varOmega, 
            (*CTRL).i->cmd_iDQ, 
            (*CTRL).i->iAB);
        break;
    case MODE_SELECT_VELOCITY_LOOP_SENSORLESS : //41
        #if WHO_IS_USER == USER_YZZ
        _user_commands();
        pmsm_observers();
        // observer_PMSMife();
        // controller_PMSMife_with_commands();
        #endif
        
        // observer();
        // FOC_with_vecocity_control(FE.AFEOE.theta_d,
        //     OBSV.nsoaf.xOmg * MOTOR.npp_inv,
        //     (*CTRL).i->cmd_varOmega,
        //     (*CTRL).i->cmd_iDQ,
        //     (*CTRL).i->iAB);
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
            (*CTRL).i->varOmega, 
            (*CTRL).i->cmd_varOmega, 
            (*CTRL).i->cmd_iDQ, 
            (*CTRL).i->iAB);
        break;
    case MODE_SELECT_TESTING_SENSORLESS : //42
        break;
    case MODE_SELECT_VELOCITY_LOOP_WC_TUNER: // 43
        break;
    case MODE_SELECT_Marino2005: //44
    #if (WHO_IS_USER == USER_CJH)
        controller_marino2005_with_commands();
    #endif
        break;
    case MODE_SELECT_POSITION_LOOP: // 5
        break;
    case MODE_SELECT_COMMISSIONING: // 9
        #if ENABLE_COMMISSIONING == TRUE
            commissioning();
        #endif
        break;
    case MODE_SELECT_GENERATOR://8
        #if PC_SIMULATION == TRUE
            Generator();
        #endif
        // ACM.R = 0.4; 
        // ACM.Ld = 0.017;
        // ACM.Lq = 0.015;
        break;
        case MODE_SWEEPING_FREQUENCY: // 20
        #if WHO_IS_USER == USER_WB
            _user_wubo_Sweeping_Command();
            if ( (*debug).bool_sweeping_frequency_for_speed_loop == TRUE ){
                FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
                            (*CTRL).i->varOmega,
                            (*CTRL).i->cmd_varOmega, 
                            (*CTRL).i->cmd_iDQ, 
                            (*CTRL).i->iAB);
            }else if ( (*debug).bool_sweeping_frequency_for_speed_loop == FALSE ){
                _user_onlyFOC();
            }
            // _user_wubo_controller();
            #endif
            break;
    case MODE_SELECT_NB_MODE: // 99
        /* You can make sound */
        tmp_id_freq_freq = global_id_freq_freq_freq * (*CTRL).timebase;
        tmp_id_freq_freq -= (long)tmp_id_freq_freq;
        global_id_freq_freq = global_id_freq_freq_ampl * sinf (2.0 * M_PI * tmp_id_freq_freq);


        tmp_id_freq = global_id_freq_freq * (*CTRL).timebase;
        tmp_id_freq -= (long)tmp_id_freq;
        global_id_freq = global_id_freq_ampl * sinf (2.0 * M_PI * tmp_id_freq);

        tmp = global_id_freq * (*CTRL).timebase;
        tmp -= (long)tmp;
        // (*debug).set_id_command = global_id_ampl * sinf(2.0 * M_PI * global_id_freq * (*CTRL).timebase);
        // local_id_commmand = global_id_ampl * sinf(2.0 * M_PI * global_id_freq * (*CTRL).timebase);
        (*debug).set_id_command = global_id_ampl * sinf(2.0 * M_PI * tmp);
        local_id_commmand = global_id_ampl * sinf(2.0 * M_PI * tmp);
        _user_onlyFOC();
    default:
        // 电压指令(*CTRL).o->cmd_uAB[0/1]通过逆变器，产生实际电压ACM.ual, ACM.ube（变换到dq系下得到ACM.ud，ACM.uq）
        // voltage_commands_to_pwm(); // this function only exists in DSP codes
        // inverter_model(); // in Simulation
        (*debug).error = 999;
        break;
    }
    return 0;
}

REAL Veclocity_Controller(REAL cmd_varOmega, REAL varOmega){

    if ((*CTRL).s->the_vc_count++ >= SPEED_LOOP_CEILING){
        (*CTRL).s->the_vc_count = 1;

        PID_Speed->Ref = cmd_varOmega;
        PID_Speed->Fbk = varOmega;
        
        /* Here is the algorithem*/
        #if WHO_IS_USER == USER_BEZIER
            control_output(PID_Speed, &BzController);
        #elif WHO_IS_USER == USER_WB
            _user_wubo_SpeedInnerLoop_controller(PID_Speed);
        #else
            PID_Speed->calc(PID_Speed);
        #endif
    }
    return PID_Speed->Out;
}

void FOC_with_vecocity_control(REAL theta_d_elec,
        REAL varOmega,
        REAL cmd_varOmega,
        REAL cmd_iDQ[2],
        REAL iAB[2]){

    // if((*debug).bool_Null_D_Control == TRUE) cmd_iDQ[0] = 0;
    /* Default is the Null D control */
    cmd_iDQ[0] = 0;
    cmd_iDQ[1] = Veclocity_Controller(cmd_varOmega, varOmega);
    // cmd_iDQ[1] = 0.0;     // iq=0 control
    (*CTRL).s->omega_syn = varOmega * (*CTRL).motor->npp;
    (*CTRL).s->xRho += CL_TS * (*CTRL).s->omega_syn;
    if((*CTRL).s->xRho > M_PI){
        (*CTRL).s->xRho -= 2*M_PI;
    }else if((*CTRL).s->xRho < -M_PI){
        (*CTRL).s->xRho += 2*M_PI; // 反转！
    }
    /// 5.Sweep 扫频将覆盖上面产生的励磁、转矩电流指令
    #if EXCITATION_TYPE == EXCITATION_SWEEP_FREQUENCY
    #if SWEEP_FREQ_C2V == TRUE
        cmd_iDQ[1] = (*debug).set_iq_command;
    #endif
    #if SWEEP_FREQ_C2C == TRUE
        cmd_iDQ[1] = 0.0;
        cmd_iDQ[0] = (*debug).set_iq_command; // 故意反的
    #endif
    #endif

    // 帕克变换
    REAL iDQ[2] = {0.0, 0.0};
    (*CTRL).s->cosT = cos(theta_d_elec);
    (*CTRL).s->sinT = sin(theta_d_elec);
    iDQ[0] = AB2M(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    iDQ[1] = AB2T(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[0] = iDQ[0];
    (*CTRL).i->iDQ[1] = iDQ[1];

    /* 速度双环的电流环 */
    // d-axis
    PID_iD->Fbk = iDQ[0];
    PID_iD->Ref = cmd_iDQ[0];
    PID_iD->calc(PID_iD);
    // q-axis
    PID_iQ->Fbk = iDQ[1];
    PID_iQ->Ref = cmd_iDQ[1];
    PID_iQ->calc(PID_iQ);

    /* 更新依赖于dq轴电流的物理量 */
    REAL Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ[0]) * iDQ[1];     // 转矩 For luenberger position observer for HFSI
    REAL cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * cmd_iDQ[0]) * cmd_iDQ[1];
    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ[0];

    /* 电流环前馈DQ轴解耦 */
    REAL decoupled_d_axis_voltage;
    REAL decoupled_q_axis_voltage;
    if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
        decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * varOmega * MOTOR.npp;
        decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * varOmega * MOTOR.npp;
    }else{
        decoupled_d_axis_voltage = PID_iD->Out;
        decoupled_q_axis_voltage = PID_iQ->Out;
    }

    /* 对补偿后的dq轴电压进行限幅度 */
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;

    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    /// 7. 反帕克变换
    // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
    // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[0] = MT2A(cmd_iDQ[0], cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[1] = MT2B(cmd_iDQ[0], cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

    // (*CTRL).o->cmd_uAB[0+2] = MT2A((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);
    // (*CTRL).o->cmd_uAB[1+2] = MT2B((*CTRL).o->cmd_uDQ[0+2], (*CTRL).o->cmd_uDQ[1+2], (*CTRL).s->cosT2, (*CTRL).s->sinT2);

    
    /// 8. 补偿逆变器非线性
    #if WHO_IS_USER == USER_WB
        /* wubo:补偿逆变器非线性 */
        wubo_inverter_Compensation( iAB );
    #else 
        main_inverter_voltage_command(TRUE);
    #endif
}
#if PC_SIMULATION == TRUE
void Generator(){
    REAL speed_cmd = 12000 * RPM_2_MECH_RAD_PER_SEC;
    ACM.TLoad = - 1.5 * (speed_cmd - ACM.varOmega);

    ACM.uDQ[0] =  150 * cos( 0.2*M_PI*0.5);
    ACM.uDQ[1] =  150 * sin( 0.2*M_PI*0.5);
    printf("ACM.omega_syn * ACM.KA * 1.732: %g\n", ACM.omega_syn * ACM.KA * 1.732);
    printf("P: %g\n", ACM.Tem * ACM.varOmega * MECH_RAD_PER_SEC_2_RPM);
    ACM.uAB[0] = MT2A(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);
    ACM.uAB[1] = MT2B(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);
    ACM.current_theta = atan2(ACM.iAB[1], ACM.iAB[0]) - M_PI;
    ACM.voltage_theta = atan2(ACM.uAB[1], ACM.uAB[0]);
    ACM.powerfactor = (angle_diff(ACM.voltage_theta, ACM.current_theta) ) * ONE_OVER_2PI * 360;
    CTRL->s->cosT = cos(angle_diff(ACM.voltage_theta, ACM.current_theta) );
    // if(sqrtf(ACM.iAB[1] * ACM.uAB[1] + ACM.iAB[0] * ACM.uAB[0])>0)
        // CTRL->s->cosT = ACM.TLoad * ACM.varOmega / sqrtf(ACM.iAB[1] * ACM.uAB[1] + ACM.iAB[0] * ACM.uAB[0]);
        //CTRL->s->cosT = sqrtf(ACM.iAB[1] * ACM.iAB[1] + ACM.iAB[0] * ACM.iAB[0]) * ACM.KA * ACM.omega_syn
        // CTRL->s->cosT = ACM.Tem * ACM.varOmega / (
        //         sqrtf((ACM.iAB[1] * ACM.iAB[1]) +  (ACM.iAB[0] *ACM.iAB[0])) * sqrtf((ACM.uAB[1] * ACM.uAB[1]) +  (ACM.uAB[0] *ACM.uAB[0]))
        //     );
    printf("power factor: %g\n", CTRL->s->cosT);
}
#endif
#include <math.h>
#include <stdio.h>

double angle_diff(double a, double b) {
    // a 和 b 必须在 [0, 2 * M_PI] 范围内
    a = fmod(a, 2 * M_PI);
    b = fmod(b, 2 * M_PI);
    double d1 = a - b;
    double d2;
    if (d1 > 0) {
        d2 = a - (b + 2 * M_PI); // d2 是负的
    } else {
        d2 = (2 * M_PI + a) - b; // d2 是正的
    }
    if (fabs(d1) < fabs(d2)) {
        return d1;
    } else {
        return d2;
    }
}

void _user_virtual_ENC(){
    if (fabs((*debug).Overwrite_Current_Frequency) > 0)
        {
            (*debug).Overwrite_theta_d += CL_TS * (*debug).Overwrite_Current_Frequency * 2 * M_PI;
            if ((*debug).Overwrite_theta_d > M_PI)  (*debug).Overwrite_theta_d -= 2 * M_PI;
            if ((*debug).Overwrite_theta_d < -M_PI) (*debug).Overwrite_theta_d += 2 * M_PI;
        }
        else
        {
            (*debug).Overwrite_theta_d = 0.0;
        }
    (*CTRL).s->cosT = cos((*debug).Overwrite_theta_d);
    (*CTRL).s->sinT = sin((*debug).Overwrite_theta_d);
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    /* 直接给定电流环command，速度环的command由程序给出 */
    (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command; 
    (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    PID_iD->calc(PID_iD);

    PID_iQ->Fbk = (*CTRL).i->iDQ[1];
    PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
    PID_iQ->calc(PID_iQ);

    /* 电流环前馈DQ轴解耦 */ 
    REAL decoupled_d_axis_voltage;
    REAL decoupled_q_axis_voltage;
    if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
        decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
        decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
    }else{
        decoupled_d_axis_voltage = PID_iD->Out;
        decoupled_q_axis_voltage = PID_iQ->Out;
    }
    
    /* 对补偿后的dq轴电压进行限幅度 */
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;

    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

    /* 更新依赖于dq轴电流的物理量 */
    (*CTRL).i->Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0]) * (*CTRL).i->iDQ[1];     // 转矩 For luenberger position observer for HFSI
    (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0]) * (*CTRL).i->cmd_iDQ[1];
    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
}

void _user_onlyFOC(){
    // 帕克变换
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    /// 5.Sweep 扫频将覆盖上面产生的励磁、转矩电流指令
    #if EXCITATION_TYPE == EXCITATION_SWEEP_FREQUENCY
    #if SWEEP_FREQ_C2V == TRUE
        PID_iQ->Ref = set_iq_cmd;
    #endif
    #if SWEEP_FREQ_C2C == TRUE
        PID_iQ->Ref = 0.0;
        PID_iD->Ref = set_iq_cmd; // 故意反的
    #endif
    #endif
    (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command;
    (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    PID_iD->calc(PID_iD);

    PID_iQ->Fbk = (*CTRL).i->iDQ[1];
    PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
    PID_iQ->calc(PID_iQ);

    // 电流环前馈DQ轴解耦
    REAL decoupled_d_axis_voltage;
    REAL decoupled_q_axis_voltage;
    if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
        decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
        decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
    }else{
        decoupled_d_axis_voltage = PID_iD->Out;
        decoupled_q_axis_voltage = PID_iQ->Out;
    }

    /* 对补偿后的dq轴电压进行限幅度 */
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;

    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

    /// 8. 补偿逆变器非线性
    #if WHO_IS_USER == USER_WB
        /* wubo:补偿逆变器非线性 */
        wubo_inverter_Compensation( (*CTRL).i->iAB );
    #elif WHO_IS_USER != USER_WB
        main_inverter_voltage_command(TRUE);
    #endif
}
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
void _user_pmsm_observer(void){

    /// 3. 调用观测器：估计的电气转子位置和电气转子转速反馈
    // observer_marino2005();
    if ((*debug).SENSORLESS_CONTROL == TRUE){ // （无感）
        (*CTRL).i->varOmega     = CTRL->motor->npp_inv * PMSM_ELECTRICAL_SPEED_FEEDBACK;    // OBSV.harnefors.omg_elec;
        (*CTRL).i->theta_d_elec = CTRL->motor->npp_inv * PMSM_ELECTRICAL_POSITION_FEEDBACK; // OBSV.harnefors.theta_d;
    }
}
#endif

#if PC_SIMULATION
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
#endif

// 定义特定的测试指令，如快速反转等
#if 0 // defined in Experiment.c???
    struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};

    void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd){
        if(timebase > instant+2*interval){
            (*debug).set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else if(timebase > instant+interval){
            (*debug).set_rpm_speed_command = 1*1500 + -rpm_cmd;
        }else if(timebase > instant){
            (*debug).set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else{
            (*debug).set_rpm_speed_command = 20; // default initial command
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

REAL _user_load_model(){
    static REAL Tload = 0.0;
    // static int load_state = 0;
    // static REAL dc_part = LOAD_TORQUE;
    // static REAL viscous_part = 0.0; // 这个变量去到Config文件里面了
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


/* MAIN */
void main_inverter_voltage_command(int bool_use_cmd_iAB){
    REAL Ia, Ib;

    /* We use cmd_iAB instead of iAB to look-up */
    if (bool_use_cmd_iAB)
    {
        Ia = (*CTRL).o->cmd_iAB[0];
        Ib = (*CTRL).o->cmd_iAB[1];
    }
    else
    {
        Ia = (*CTRL).i->iAB[0];
        Ib = (*CTRL).i->iAB[1];
    }

    G.FLAG_INVERTER_NONLINEARITY_COMPENSATION = 0;
    if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 0){
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

        /* For scope only */
        #if PC_SIMULATION
            REAL ualbe_dist[2];
            get_distorted_voltage_via_CurveFitting((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist);
            INV.ual_comp = ualbe_dist[0];
            INV.ube_comp = ualbe_dist[1];
        #endif
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 4)
    {
        REAL ualbe_dist[2];
        get_distorted_voltage_via_LUT_indexed(Ia, Ib, ualbe_dist);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 3)
    {
    /* 查表法-补偿 */
    // // Measured in Simulation when the inverter is modelled according to the experimental measurements
    // #define LENGTH_OF_LUT  19
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-3.78, -3.36, -2.94, -2.52, -2.1, -1.68, -1.26, -0.839998, -0.419998, -0, 0.420002, 0.840002, 1.26, 1.68, 2.1, 2.52, 2.94, 3.36, 3.78};
    // // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-6.41808, -6.41942, -6.39433, -6.36032, -6.25784, -6.12639, -5.79563, -5.35301, -3.61951, 0, 3.5038, 5.24969, 5.73176, 6.11153, 6.24738, 6.35941, 6.43225, 6.39274, 6.39482};
    // #define MANUAL_CORRECTION 1.1 // [V]
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {
    //     -6.41808+MANUAL_CORRECTION,
    //     -6.41942+MANUAL_CORRECTION,
    //     -6.39433+MANUAL_CORRECTION,
    //     -6.36032+MANUAL_CORRECTION,
    //     -6.25784+MANUAL_CORRECTION,
    //     -6.12639+MANUAL_CORRECTION,
    //     -5.79563+MANUAL_CORRECTION,
    //     -5.35301+MANUAL_CORRECTION,
    //     -3.61951, 0, 3.5038,
    //      5.24969-MANUAL_CORRECTION,
    //      5.73176-MANUAL_CORRECTION,
    //      6.11153-MANUAL_CORRECTION,
    //      6.24738-MANUAL_CORRECTION,
    //      6.35941-MANUAL_CORRECTION,
    //      6.43225-MANUAL_CORRECTION,
    //      6.39274-MANUAL_CORRECTION,
    //      6.39482-MANUAL_CORRECTION};

    // #define LENGTH_OF_LUT  100
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.116, -4.032, -3.948, -3.864, -3.78, -3.696, -3.612, -3.528, -3.444, -3.36, -3.276, -3.192, -3.108, -3.024, -2.94, -2.856, -2.772, -2.688, -2.604, -2.52, -2.436, -2.352, -2.268, -2.184, -2.1, -2.016, -1.932, -1.848, -1.764, -1.68, -1.596, -1.512, -1.428, -1.344, -1.26, -1.176, -1.092, -1.008, -0.923998, -0.839998, -0.755998, -0.671998, -0.587998, -0.503998, -0.419998, -0.335999, -0.251999, -0.168, -0.084, -0, 0.084, 0.168, 0.252001, 0.336001, 0.420002, 0.504002, 0.588002, 0.672002, 0.756002, 0.840002, 0.924002, 1.008, 1.092, 1.176, 1.26, 1.344, 1.428, 1.512, 1.596, 1.68, 1.764, 1.848, 1.932, 2.016, 2.1, 2.184, 2.268, 2.352, 2.436, 2.52, 2.604, 2.688, 2.772, 2.856, 2.94, 3.024, 3.108, 3.192, 3.276, 3.36, 3.444, 3.528, 3.612, 3.696, 3.78, 3.864, 3.948, 4.032, 4.116, 4.2};
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-6.48905, -6.49021, -6.49137, -6.49253, -6.49368, -6.49227, -6.49086, -6.48945, -6.48803, -6.48662, -6.47993, -6.47323, -6.46653, -6.45983, -6.45313, -6.44465, -6.43617, -6.42769, -6.41921, -6.41072, -6.38854, -6.36637, -6.34419, -6.32202, -6.29984, -6.27187, -6.2439, -6.21593, -6.18796, -6.15999, -6.09216, -6.02433, -5.9565, -5.88867, -5.82083, -5.73067, -5.6405, -5.55033, -5.46016, -5.36981, -5.02143, -4.67305, -4.32467, -3.97629, -3.62791, -2.90251, -2.17689, -1.45126, -0.725632, -1e-06, 0.702441, 1.40488, 2.10732, 2.80976, 3.5122, 3.86321, 4.21409, 4.56497, 4.91585, 5.26649, 5.36459, 5.46268, 5.56078, 5.65887, 5.75696, 5.8346, 5.91224, 5.98987, 6.0675, 6.14513, 6.17402, 6.20286, 6.2317, 6.26054, 6.28938, 6.31347, 6.33755, 6.36164, 6.38572, 6.40981, 6.4303, 6.4508, 6.47129, 6.49178, 6.49105, 6.48483, 6.4786, 6.47238, 6.46616, 6.45994, 6.46204, 6.46413, 6.46623, 6.46832, 6.47042, 6.47202, 6.47363, 6.47524, 6.47684, 6.47843};

    // Experimental measurements
    // #define LENGTH_OF_LUT 21
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.19999, -3.77999, -3.36001, -2.94002, -2.51999, -2.10004, -1.68004, -1.26002, -0.840052, -0.419948, 5.88754e-06, 0.420032, 0.839998, 1.26003, 1.67998, 2.10009, 2.51996, 2.87326, 3.36001, 3.78002, 4.2};
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-5.20719, -5.2079, -5.18934, -5.15954, -5.11637, -5.04723, -4.93463, -4.76367, -4.42522, -3.46825, 0.317444, 3.75588, 4.55737, 4.87773, 5.04459, 5.15468, 5.22904, 5.33942, 5.25929, 5.28171, 5.30045};

    // Experimental measurements 03-18-2021
    #define LENGTH_OF_LUT 41
        REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.20001, -3.98998, -3.78002, -3.57, -3.36002, -3.14999, -2.93996, -2.72993, -2.51998, -2.31003, -2.1, -1.88996, -1.67999, -1.46998, -1.25999, -1.05001, -0.839962, -0.62995, -0.420046, -0.210048, 1.39409e-05, 0.209888, 0.420001, 0.629998, 0.840008, 1.05002, 1.25999, 1.47002, 1.68001, 1.89001, 2.10002, 2.31, 2.51999, 2.73004, 2.94, 3.14996, 3.35995, 3.57001, 3.77999, 3.98998, 4.2};
        REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-5.75434, -5.74721, -5.72803, -5.70736, -5.68605, -5.66224, -5.63274, -5.59982, -5.56391, -5.52287, -5.47247, -5.40911, -5.33464, -5.25019, -5.14551, -5.00196, -4.80021, -4.48369, -3.90965, -2.47845, -0.382101, 2.02274, 3.7011, 4.35633, 4.71427, 4.94376, 5.10356, 5.22256, 5.31722, 5.39868, 5.46753, 5.5286, 5.57507, 5.62385, 5.66235, 5.70198, 5.73617, 5.76636, 5.79075, 5.81737, 5.83632};

        REAL ualbe_dist[2];
        get_distorted_voltage_via_LUT((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist, lut_voltage_ctrl, lut_current_ctrl, LENGTH_OF_LUT);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 2)
    {
        /* 拟合法-补偿 */
        REAL ualbe_dist[2] = {0.0, 0.0};
        get_distorted_voltage_via_CurveFitting((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 1)
    {   
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        /* 梯形波自适应 */
        Modified_ParkSul_Compensation();
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + INV.ual_comp;
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + INV.ube_comp;

        #endif
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 5)
    {   
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        /* Sigmoid自适应 */
        Online_PAA_Based_Compensation();
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + INV.ual_comp;
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + INV.ube_comp;

        #endif
    }
}


