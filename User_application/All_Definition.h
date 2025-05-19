#ifndef ALL_DEFINITATION_H
#define ALL_DEFINITATION_H
//#include <stdio.h>
//#include <stdlib.h>

/* User Application: Motor Drives */
    #include "ACMSim.h"
    #include "ACMConfig.h"
    #include "Experiment.h"
    #include "Device_define.h" // 和具体硬件（腿组，电机组，感应电机组）相关的配置参数在这
    //#include "math.h"
//    #include "IQmathLib.h"

/* User Application: Cury the leg */
    #include "AppCury.h" 

/* CLA header files */
    #include "DCLCLA.h"
    #include "CLA_shared.h"
    #include "F2837xD_Cla.h"           // Control Law Accelerator Registers
    #include "F2837xD_Cla_defines.h"   // Macros used for CLA examples.
    #include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions

/* DSP system Configuration------------------------------------------------------------------*/
    #include "F2837xD_device.h"        // F2837xD Headerfile Include File
    #include "F2837xD_Examples.h"      // F2837xD Examples Include File

/* Motor Library file------------------------------------------------------------------------*/
    #define USE_DEATIME_PRECOMP FALSE
    __interrupt void EPWM1ISR(void);
    #define ENABLE_ECAP 0
    #define USE_ECAP_CEVT2_INTERRUPT 0 // measure terminal voltage duty using e-capture
    __interrupt void ecap1_isr(void);
    __interrupt void ecap2_isr(void);
    __interrupt void ecap3_isr(void);
    __interrupt void EQEP_UTO_INT(void);
    __interrupt void scicRxFifoIsr(void);
    __interrupt void scicTxFifoIsr(void);
    void SVGEN_Drive(SVGENDQ* ptrV);
/* Driver -----------------------------------------------------------------------------------*/
    #include "CONSOLE.h"
    #include "DAC_MAX5307.h"
    #include "ECaptureVoltage.h"
    #include "F2837xD_Ipc_drivers.h" // ˫��ͨѶ
    #include "F2837xD_struct.h"
    #include "F2837xD_sdfm_drivers.h"
    #include "ShareMemory.h"
    #include "i2cTalkToLDC1612.h"
    void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);//flash
/* Hardware Peripherals Configuration -------------------------------------------------------*/
    void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
    void ePWM_initialize(void);
    void eQEP_initialize(int m);
    void Gpio_initialize(void);
    void ADC_initialize(void);
    //PWM CONFIGURATION // see ACMSim.h
    //QEP CONFIGURATION
    // #define SYSTEM_QEP_LINE                  2500     //encoder line
    // #define SYSTEM_QEP_POLE_PAIRS               4     //pairs
    #define SYSTEM_QEP_UNITTIME_ISR          0.001    //1K ,1ms     time_out timer   isr_time
    #define SYSTEM_QEP_CAP_X128           0.00000064  //cap timer
    #define SYSTEM_QEP_CUTOFF_FILTER           5      // CUTOFF FREQUENCY 10HZ
    #define SYSTEM_QEP_SWAP_ENABLE              1     //���������?
    #define SYSTEM_QEP_SWAP_DISABLE             0     //���������?
    //ADC CONFIGURATION is moved to main.c
    //DAC Configuration
    #define NO_OF_DAC_CHANNELS 8
    //TRIP CONFIGURATION
    //#define MAX_CURRENT_N                      -11.8      //-12A  // Set your negative current trip threshold here in [0, 4095]
    //#define MAX_CURRENT_P                       11.8       //12A   // Set your positive current trip threshold here in [0, 4095]
    //#define MAX_OVERVOLTAGE                    400      //400V  // Set your unipolar trip Over-voltage threshold in [0, 4095]
    //GPIO
    //    #define DSP_PWM_DISABLE        GpioDataRegs.GPDSET.bit.GPIO105=1;    // ����Ч����λ�Ⲩ
    //    #define DSP_PWM_ENABLE         GpioDataRegs.GPDCLEAR.bit.GPIO105=1;  // ����Ч��������Ч
    //    #define DSP_2PWM_DISABLE       GpioDataRegs.GPASET.bit.GPIO27=1;
    //    #define DSP_2PWM_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    // #define DSP_PWM_DISABLE        GpioDataRegs.GPDSET.bit.GPIO108=1;    // ����Ч����λ�Ⲩ
    // #define DSP_PWM_ENABLE         GpioDataRegs.GPDCLEAR.bit.GPIO108=1;  // ����Ч��������Ч
    // #define DSP_2PWM_DISABLE       GpioDataRegs.GPCSET.bit.GPIO93=1;
    // #define DSP_2PWM_ENABLE        GpioDataRegs.GPCCLEAR.bit.GPIO93=1;
    
    // [WuBo]:这里的GPDCLEAR GPCCLEAR GPACLEAR GPASET 不一样原因是？
    #define DSP_PWM1_ENABLE        GpioDataRegs.GPDCLEAR.bit.GPIO108=1;
    #define DSP_PWM2_ENABLE        GpioDataRegs.GPCCLEAR.bit.GPIO93=1;
    #define DSP_PWM3_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO28=1;
    #define DSP_PWM4_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO29=1;

    #define DSP_PWM1_DISABLE       GpioDataRegs.GPDSET.bit.GPIO108=1;
    #define DSP_PWM2_DISABLE       GpioDataRegs.GPCSET.bit.GPIO93=1;
    #define DSP_PWM3_DISABLE       GpioDataRegs.GPASET.bit.GPIO28=1;
    #define DSP_PWM4_DISABLE       GpioDataRegs.GPASET.bit.GPIO29=1;

    #define INVERTER_FLT_FAULT     GpioDataRegs.GPDDAT.bit.GPIO104  //Inverter_error signal

    #define DSP_STOP_LED1  GpioDataRegs.GPDCLEAR.bit.GPIO124=1;
    #define DSP_STOP_LED2  GpioDataRegs.GPBCLEAR.bit.GPIO33=1;
    #define DSP_START_LED1 GpioDataRegs.GPDSET.bit.GPIO124=1;
    #define DSP_START_LED2 GpioDataRegs.GPBSET.bit.GPIO33=1;

/* Logic  -----------------------------------------------------------------------------------*/
    //#include "Logic.h"                //�߼��� �������ϴ��룬DI,״̬��
    extern int VoltageOVER_FLAG;
    struct Trip_Variables
    {
        BOOL VoltageOVER_FLAG;
        BOOL CurrentOVER_FLAG;
    };
    extern struct Trip_Variables trip;//trip flag
    //#define Motor_mode_START    GpioDataRegs.GPCDAT.bit.GPIO75    //DI Start Button
    int Motor_MODE_START(void);
    //#define Motor_mode_START    digital_virtual_button          //virtual DI Start Button
    int Motor_MODE_STOP(void);
    int Motor_MODE_REVERSE(void);
    void System_Checking(void);

/* Global Variable */
typedef struct{
    int ID;

    /* Position Sensor
    * eQEP for Incremental Encoder, eQEP:增强型正交编码器接口
    * Abs Encoder Your know it,
    * I2C for coil sensor for Artificial Heart
    */
        int if_enable_EQEP;
        int if_enable_AbsoluteEncoder;
        int if_enable_I2C;

    /* PWM Operation Status
    * FLAG_ENABLE_PWM_OUTPUT enable/disable ALL PWM channel; controlled by GpioDataRegs.GPCDAT.bit.GPIO75, a elec switch
    * epwm_enable[4] enable/disable each PWM channel
    */
    int FLAG_ENABLE_PWM_OUTPUT;
    int epwm_enable[4];
    int flag_init_DSP_svgen;
    REAL  svgen_Ta[4];     // Output: reference phase-a switching function
    REAL  svgen_Tb[4];     // Output: reference phase-b switching function
    REAL  svgen_Tc[4];     // Output: reference phase-c switching function

    /* Misc
    */
    int USE_3_CURRENT_SENSORS;
    int Trip_Flag;
    int flag_overwite_vdc;
    REAL overwrite_vdc;

    /* Scale and Offset
    * Linear Approximation is used for ADC results, like real_value = scale * digital_value + offset
        * ADC offset and scale can be changed online
        * Go to Device_define.h to change the scale and offset to save them
    * OffsetCountBetweenIndexAndUPhaseAxis is the offset between encoder and U phase of motor
        * AKA 编码器零位
        * For FOC control, we need to align them to have the rotor position
        * For more explaination, please refer to website:
        * https://www.bilibili.com/video/BV17m411278e/?spm_id_from=333.1387.homepage.video_card.click&vd_source=b02e7899298289ea702c9877667b9ebf
    * All these values should be determined again before using a new device
    */
    int AD_offset_flag2;
    REAL offset_counter;
    REAL offset_online[6];
    // Sensor - Raw measurement
        REAL vdc[4];
        REAL iabg[12];
        REAL iuvw[12];
        REAL iuvw_offset_online[6];
        Uint32 SCI_Position_Count_fromCPU2[4];
    // ADC offset and scale
        REAL adc_vdc_scale[4];
        REAL adc_vdc_offset[4];
        REAL adc_iuvw_scale[12];
        REAL adc_iuvw_offset[12]; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1. <- WHAT???
    // Encoder offset
        REAL OffsetCountBetweenIndexAndUPhaseAxis[4];
    // Sensor Coil
        REAL place_sensor[8];
        REAL place_offset[8];
        REAL place_scale[8];
} st_dsp;
extern st_dsp DSP;


typedef struct{
    int ID;
    struct ControllerForExperiment *pCTRL;
    struct DebugExperiment *Pdebug;
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

/* Tools that main use */
    #include "tools.h"

#endif
