#include "All_Definition.h"
#include "new_user.h"

st_axis Axes[4];

//* 暂时的，为了程序不报错，之后Axis会被全部去掉！
st_axis Axis_1 = {0};
st_axis Axis_2 = {0};
st_axis *Axis = {0};

extern BOOL run_enable_from_PC;
extern int axisCnt;
extern struct ControllerForExperiment CTRL_1;
extern struct ControllerForExperiment CTRL_2;
extern struct ControllerForExperiment CTRL_3;
extern struct ControllerForExperiment CTRL_4;


//TODO ：下面变量需要移动到合适的地方去！
int counter_missing_position_measurement = 0;
int max_counter_missing_position_measurement = 0;
Uint32 CPU2_commu_error_counter = 0;
int bool_use_SCI_encoder = TRUE;
Uint32 position_count_SCI_shank_fromCPU2;
Uint32 position_count_SCI_hip_fromCPU2;
Uint32 position_count_SCI_fromCPU2;
Uint32 position_count_CAN_ID0x01_fromCPU2;
Uint32 position_count_CAN_ID0x03_fromCPU2;
Uint32 position_count_CAN_fromCPU2;

int USE_3_CURRENT_SENSORS = TRUE;
int bool_TEMP = FALSE;
REAL legBouncingSpeed = 50;
REAL hipBouncingFreq = 10;
REAL legBouncingIq = 2;
REAL hipBouncingIq = 1.5;
REAL target_position_cnt;
REAL target_position_cnt_shank = 58000;
REAL target_position_cnt_hip = 48000;



/* Functional Function */


/*
 * Function: user_init_axis
 * Purpose: init Axes[4] and assign CTRL to pCTRL of Axes[4]
*/
void user_init_axis(int axisCnt){
    if (axisCnt == 0) CTRL = &CTRL_1;
    if (axisCnt == 1) CTRL = &CTRL_2;
    if (axisCnt == 2) CTRL = &CTRL_3;
    if (axisCnt == 3) CTRL = &CTRL_4;
    allocate_CTRL(CTRL); // allocate memory for CTRL

    Axes[axisCnt].ID     = 100 + axisCnt;
    Axes[axisCnt].pCTRL  = CTRL;
    Axes[axisCnt].Pdebug = debug;

    Axes[axisCnt].channels_preset = 12;  // 9; // 101;
                                         // 9  /* With SPEED ESO */
                                         // 10 /* WCtuner Debug */
                                         // 11 /* 20240414 Debugging */
                                         // 12 /* 20240418 Pos Loop */

    #if  WHO_IS_USER              == USER_BEZIER
        Axes[axisCnt].channels_preset  = 9;           // 8; // 101;
    #endif
    #if BOOL_LOAD_SWEEPING_ON  //6作为Load Sweeping使用的channel preset
        Axes[axisCnt].channels_preset = 6;
    #endif

    Axes[axisCnt].pCTRL->enc->sum_qepPosCnt                 = 0;
    Axes[axisCnt].pCTRL->enc->cursor                        = 0;
    Axes[axisCnt].pCTRL->enc->flag_absolute_encoder_powered = FALSE;
}

/*
 * Function: _user_ResetController, _Reset_CTRL_except_Controller_Param
 * Purpose: used at 'user_routine_disable_pwm_output' to reset CTRL
*/
void _Reset_CTRL_except_Controller_Param(){
    PID_Speed->Ref = 0;
    PID_Speed->Out = 0;
    PID_Speed->OutPrev = 0;
    PID_Speed->Fbk = 0;
    PID_Speed->Err = 0;
    PID_Speed->ErrPrev = 0;
    PID_Speed->I_Term = 0;

    PID_iD->Ref = 0;
    PID_iD->Out = 0;
    PID_iD->OutPrev = 0;
    PID_iD->Fbk = 0;
    PID_iD->Err = 0;
    PID_iD->ErrPrev = 0;
    PID_iD->I_Term = 0;
    
    PID_iQ->Ref = 0;
    PID_iQ->Out = 0;
    PID_iQ->OutPrev = 0;
    PID_iQ->Fbk = 0;
    PID_iQ->Err = 0;
    PID_iQ->ErrPrev = 0;
    PID_iQ->I_Term = 0;
    // 清空速度InnerLoop缓存 & Harnefors Var
    #if WHO_IS_USER == USER_WB
        SIL_Controller.KFB_Term = 0;
        Harnefors_1998_BackCals_Variable.I_Term_prev = 0.0;
        Harnefors_1998_BackCals_Variable.I_Term_prev_iD = 0.0;
        Harnefors_1998_BackCals_Variable.I_Term_prev_iQ = 0.0;
    #endif

    // Sweeping 
    d_sim.user.timebase_for_Sweeping        = 0.0;
    d_sim.user.CMD_SPEED_SINE_HZ            = 0.0;
    d_sim.user.CMD_SPEED_SINE_END_TIME      = CL_TS;
    d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = 0.0;
    d_sim.user.flag_clear_timebase_once     = FALSE;
    // d_sim.user.Mark_Counter                 = 0.0; // clear the MARK !!!!! clear ti manually !!
}
void _Reset_CTRL_ALL(){
}
void _Reset_Controller_Param(){
}

void _user_ResetController(st_reset_mode mode) {
    switch(mode) {
        case RESET_MODE_SOFT:  _Reset_CTRL_except_Controller_Param(); break;
        case RESET_MODE_HARD:  _Reset_CTRL_ALL(); break;
        case RESET_MODE_SAFE:  _Reset_Controller_Param();
    }
}




/* Routine Function */

// Initialization
void user_routine_init_in_main(){
    init_d_sim();      // do this only once here
    init_debug();      // do this only once here
    init_experiment(); // 控制器结构体初始化（同实验）

    // get_bezier_points(); // for testing Cury the leg trajectgory tracking
    
    /*  Cyclic Initialization For Axes and CTRL */
    for (axisCnt = 0; axisCnt < NUMBER_OF_AXES; axisCnt++){
        // (*debug).bool_initilized = FALSE;
        // _user_init();      // debug initilization for user 20240929 is the operation is moved outside of for loop
        // get_Axis_CTRL_pointers
        user_init_axis(axisCnt); // 根据axiscnt对Axis，CTRL的1和2号结构体，进行初始化操作
    }
    axisCnt = 1;
}

// The Func run in mainLoop (not in )
void user_routine_in_main_loop(){
    // for moment it's null
}




#if ENCODER_TYPE != INCREMENTAL_ENCODER_QEP

    void measurement_position_count_axisCnt0(){
        #if (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK)
                position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
                position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
        #endif
        #if NUMBER_OF_AXES == 2
            // position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
            position_count_SCI_fromCPU2 = Axis->SCI_Position_Count_fromCPU2;
        #endif
        #if NUMBER_OF_AXES == 4

        #endif
            // 正电流导致编码器读数增大：
            // CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[0] * (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
            // CTRL_1.enc->encoder_abs_cnt = (int32)position_count_SCI_fromCPU2 - 
            CTRL->enc->encoder_abs_cnt = (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
    }

    void measurement_position_count_axisCnt1(){
        #if NUMBER_OF_AXES == 2
            // position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
            position_count_SCI_fromCPU2 = Axis->SCI_Position_Count_fromCPU2;
        #endif
            // 正电流导致编码器读数减小
            // CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[1] * ( (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis );
            CTRL->enc->encoder_abs_cnt =  (-1) * ( (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis );
            // dq变化中，d轴理论上指向永磁体的北极，
    }

    void measurement_position_count_axisCnt2(){

    }
    void measurement_position_count_axisCnt3(){
    
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



void user_routine_read_from_cpu02(){
#if NUMBER_OF_DSP_CORES == 2
    /* CPU02 (Remote) to CPU01 (Local)
     * The register to check is IPCSTS.
     * */
    counter_missing_position_measurement += 1;
    if (IPCRtoLFlagBusy(IPC_FLAG10) == 1){ // if flag
        max_counter_missing_position_measurement = counter_missing_position_measurement;
        counter_missing_position_measurement = 0;
        Axes[0].SCI_Position_Count_fromCPU2 = Read.SCI_A_position_count;
        Axes[1].SCI_Position_Count_fromCPU2 = Read.SCI_B_position_count;
        
        // Axis_1.SCI_Position_Count_fromCPU2 = Read.SCI_A_position_count;
        // Axis_2.SCI_Position_Count_fromCPU2 = Read.SCI_B_position_count;
        // Axis_3.SCI_Position_Count_fromCPU2 = Read.SCI_C_position_count;
        // Axis_4.SCI_Position_Count_fromCPU2 = Read.SCI_D_position_count;

        // position_count_SCI_shank_fromCPU2 = Read.SCI_shank_position_count;
        // position_count_SCI_hip_fromCPU2   = Read.SCI_hip_position_count;
        //            position_count_CAN_ID0x01_fromCPU2 = Read.CAN_position_count_ID0x01;
        //            position_count_CAN_ID0x03_fromCPU2 = Read.CAN_position_count_ID0x03;
        IPCRtoLFlagAcknowledge(IPC_FLAG10);

        // CAN encoder convert to motor built-in encoder
        deg_four_bar_map_motor_encoder_angle = get_motorpos(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125); // 1/131072.0*360.0
        // deg_four_bar_map_motor_encoder_angle = lookup(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125, &ZJL_table);
        rad_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 0.017453292519943295;
        cnt_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 23301.68888888889;
    }
    else{
        CPU2_commu_error_counter++;
    }

    if (IPCRtoLFlagBusy(IPC_FLAG11) == 1){ // if flag
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

void user_routine_disable_pwm_output(){

    /* Only init once for easy debug */
    /* wubo：每次执行ENABLE PWM OUT时候会将下面的flag置为0 从而每次关闭开启开关都会执行下面的操作！上面说的有问题？*/
    if (!G.flag_experimental_initialized){
        G.flag_experimental_initialized = TRUE;

        /* wubo: init_CTRL will clear timebase but when the timecounter is running all the time hence it will give value to timebase whenever DSP is on or off, finally timebase seems like not been cleared*/
        init_experiment();

        // wubo first try enum structure
        st_reset_mode resetMode;
        _user_ResetController(resetMode); // 修改resetMode来修改重置操作




        // 按理说，new_user中不允许出现有关DSP的操作
        // 我想通过Axes建立CTRL和DSP变量之间的关系，DSP结构体不允许被直接操作

        // 这句话存疑
        // for (int i = 0; i < 4; i++) {
//             memset(Axes[i].Ta, 0, sizeof(Axes[i].Ta)); // 将 Ta[0~3] 全部置为 0
        // }
        
        // EPwm1Regs.CMPA.bit.CMPA = 2500;
        // EPwm2Regs.CMPA.bit.CMPA = 2500;
        // EPwm3Regs.CMPA.bit.CMPA = 2500;
        // EPwm4Regs.CMPA.bit.CMPA = 2500;
        // EPwm5Regs.CMPA.bit.CMPA = 2500;
        // EPwm6Regs.CMPA.bit.CMPA = 2500;

        if ((*CTRL).g->overwrite_vdc < 5){
            (*CTRL).g->overwrite_vdc = 28;
        }(*CTRL).g->flag_overwite_vdc = 0;

        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        #if WHO_IS_USER == USER_WB
            (*debug).CMD_CURRENT_SINE_AMPERE      = d_sim.user.CMD_CURRENT_SINE_AMPERE;
            (*debug).CMD_SPEED_SINE_RPM           = d_sim.user.CMD_SPEED_SINE_RPM;
            (*debug).CMD_SPEED_SINE_HZ            = d_sim.user.CMD_SPEED_SINE_HZ;
            (*debug).CMD_SPEED_SINE_STEP_SIZE     = d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
            (*debug).CMD_SPEED_SINE_LAST_END_TIME = d_sim.user.CMD_SPEED_SINE_LAST_END_TIME;
            (*debug).CMD_SPEED_SINE_END_TIME      = d_sim.user.CMD_SPEED_SINE_END_TIME;
            (*debug).CMD_SPEED_SINE_HZ_CEILING    = d_sim.user.CMD_SPEED_SINE_HZ_CEILING;
        #endif
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
    }

    /* 在不输出PWM波形的时候，也就是“开关”为OFF的时候，更新SpeedInnerLoop的参数*/
    #if WUBO_ONLINE_TUNING
        #if WHO_IS_USER == USER_WB
        wubo_debug_tools[9] = 11;
            if ( (*debug).who_is_user == USER_WB && (*debug).bool_apply_WC_tunner_for_speed_loop == TRUE ){
                _user_wubo_WC_Tuner_Online(); // 和_user_wubo_WC_Tuner函数区别
                wubo_debug_tools[9] = 12;
            }else{
                _user_wubo_TI_Tuner_Online();
                wubo_debug_tools[9] = 13;
            }
        #else
            //TODO: Here needs a online tuning function for public use
        #endif
    #endif

    DELAY_US(5);
    GpioDataRegs.GPDCLEAR.bit.GPIO106 = 1; // TODO: What is this doing?
    
}





/* 初始化4套EPWM的占空比 */
void user_routine_init_pwm_output(){

    if (use_first_set_three_phase == 1){
        DSP_PWM1_ENABLE
        // 第一套逆变器工作的时候，确保第二套逆变器关闭
        EPwm4Regs.CMPA.bit.CMPA = 2500;
        EPwm5Regs.CMPA.bit.CMPA = 2500;
        EPwm6Regs.CMPA.bit.CMPA = 2500;
    } else if (use_first_set_three_phase == 2){
        DSP_PWM2_ENABLE
        // 同上
        EPwm1Regs.CMPA.bit.CMPA = 2500;
        EPwm2Regs.CMPA.bit.CMPA = 2500;
        EPwm3Regs.CMPA.bit.CMPA = 2500;
    }else if (use_first_set_three_phase == -1){
        DSP_PWM1_ENABLE
        DSP_PWM2_ENABLE
    }
}




REAL user_routine_enable_pwm_output(){

    (*CTRL).timebase_counter += 1;
    (*CTRL).timebase = CL_TS * (*CTRL).timebase_counter; //(*CTRL).timebase += CL_TS; // 2048 = float/REAL max

    REAL vvvf_voltage = 4;
    REAL vvvf_frequency = 5;
    REAL enable_vvvf = FALSE;

    /* PWM signal to Inverter Voltage Output SWPWM */
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

    return 0;
}

// 多功能debug的时候写这里
void user_routine_debug_switch(){
    G.flag_experimental_initialized = TRUE; // Set this flag to TRUE to run the init in DISABLE_PWM_ENABLE once again
    
    // WE SHOULD USE STRCTURE ENUM TO MAKE DEBUG CLEAR !
    int pwm_test_mode = main_switch(debug->mode_select); 



    // Waintg for Your Da Zuo

    
}

// 机器人直接部署的时候就写这里，因为机器人底层的控制模式可以不用修改了！
void user_routine_main_switch(){

}
