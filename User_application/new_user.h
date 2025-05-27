#ifndef NEW_USER_H  // 如果没有定义 NEW_USER_H
#define NEW_USER_H  // 定义 NEW_USER_H



/* When PWM is DISABLE*/
typedef enum {
    RESET_MODE_SOFT,  // 保持配置参数 _Reset_CTRL_except_Controller_Param();
    RESET_MODE_HARD,  // 恢复出厂设置 _Reset_CTRL_ALL();
    RESET_MODE_SAFE   // 进入安全状态后重置 _Reset_Controller_Param();
} st_reset_mode;

void _user_ResetController(st_reset_mode mode);
void _Reset_CTRL_except_Controller_Param();
void _Reset_CTRL_ALL();
void _Reset_Controller_Param();


/* When PWM is ENABLE */
// typedef enum { 
//     MODE_SELECT_PWM_DIRECT         
//     MODE_SELECT_VOLTAGE_OPEN_LOOP  
//     MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE
//     MODE_SELECT_FOC                      
//     MODE_SELECT_FOC_SENSORLESS           
//     MODE_SELECT_INDIRECT_FOC             
//     MODE_SELECT_ID_SWEEPING_FREQ         
//     MODE_SELECT_IQ_SWEEPING_FREQ         
//     MODE_SELECT_FOC_HARNEFORS_1998       
//     MODE_SELECT_VELOCITY_LOOP            
//     MODE_SELECT_VELOCITY_LOOP_SENSORLESS 
//     MODE_SELECT_TESTING_SENSORLESS       
//     MODE_SELECT_V_LOOP_WC_TUNER     // 这个模式被弃用了，现在于USER_WB中实现WC Tuner
//     MODE_SELECT_Marino2005               
//     MODE_SELECT_SWEEPING_FREQ   
//     MODE_SELECT_V_LOOP_ESO_SPEED_REF 
//     MODE_SELECT_POSITION_LOOP            
//     MODE_SELECT_CURY_POSITION_LOOP       
//     MODE_SELECT_POSITION_IMPEDANCE_CONTRssOL 
//     MODE_SELECT_COMMISSIONING            
// } st_mode_select;
// st_mode_select modeSelect;

typedef struct{
    int ID;
    struct ControllerForExperiment *pCTRL;
    struct DebugExperiment *Pdebug;

    // Motor Operation Mode
    // mode_select


    // Commonly used for prototype motor testing
       //int use_first_set_three_phase;
        //int Set_current_loop;
        //int Set_x_suspension_current_loop;
        //int Set_y_suspension_current_loop;
        //REAL Set_manual_rpm;
        //REAL Prev_manual_rpm;
        //REAL rampRate_rpm;
        //REAL Set_manual_current_id;
        //REAL Set_manual_current_iq;
        //REAL Set_manual_current_ix;
        //REAL Set_manual_current_iy;
        //int Select_exp_operation;
        //int *pFLAG_INVERTER_NONLINEARITY_COMPENSATION;
        //int flag_overwrite_theta_d;
        //REAL Overwrite_Current_Frequency;
        //REAL Overwrite_Suspension_Current_Frequency;
        //REAL used_theta_d_elec;
        //REAL angle_shift_for_first_inverter;
        //REAL angle_shift_for_second_inverter;
        //REAL OverwriteSpeedOutLimitDuringInit;

        // ADC Offset
        // Automatic Offset Removing

        //        volatile struct ADC_RESULT_REGS *pAdcaResultRegs;
        //        volatile struct ADC_RESULT_REGS *pAdcbResultRegs;
        //        volatile struct ADC_RESULT_REGS *pAdccResultRegs;
    
    
    // int FLAG_ENABLE_PWM_OUTPUT; //  电机模式标志位
    int AD_offset_flag2;
    REAL offset_counter;
    REAL offset_online[6];
    // Raw
        REAL adc_offset[12]; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
        REAL adc_scale[12];
    // Sensor - Raw measurement
        REAL vdc;
        REAL iabg[6];
        REAL iuvw[6];
        REAL iuvw_offset_online[6];
        Uint32 SCI_Position_Count_fromCPU2;
    // DAC
        int DAC_MAX5307_FLAG; // for single core case
        REAL dac_offset[NO_OF_DAC_CHANNELS];
        REAL dac_time;
        REAL dac_watch[80];
        REAL dac_watch_stator_resistance;
        int channels[NO_OF_DAC_CHANNELS];
        int channels_preset;
    // Sensor Coil
        REAL place_sensor[8];
        REAL place_offset[8];
        REAL place_scale[8];
} st_axis;

extern st_axis Axis_1;
extern st_axis Axis_2;
extern st_axis *Axis;


void user_init_axis(int axisCnt);
void measurement_position_count_axisCnt0(); // 对四个电机采取不同的axisCnt函数，而不是循环——方便处理当驱动对象为4台不同电机时的情况
void measurement_position_count_axisCnt1();
void measurement_position_count_axisCnt2();
void measurement_position_count_axisCnt3();
void measurement_enc();
void write_RPM_to_cpu02_dsp_cores_2();

void user_routine_init_in_main();
void user_routine_in_main_loop();
void user_routine_init_pwm_output();
void user_routine_read_from_cpu02();
void user_routine_debug_switch();
void user_routine_disable_pwm_output();
REAL user_routine_enable_pwm_output(); // 返回四台逆变器的三相占空比

#endif // NEW_USER_H

