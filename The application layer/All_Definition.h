/*
 *  Created on: 2021��1��15��
 *      Author: yuanxin and jiahao
 *       */
#ifndef ALL_DEFINITATION_H
#define ALL_DEFINITATION_H
#include "math.h"
//#include "IQmathLib.h"            //Ӳ���� StepAngle
/* DSP system Configuration-----------------------------------------------------------------------------------*/
    #include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
    #include "F2837xD_device.h"        // F2837xD Headerfile Include File
    #include "F2837xD_Examples.h"      // F2837xD Examples Include File
/* �궨�岼������ */
    #define BOOL int
    #define TRUE 1
    #define FALSE 0
/* ������Ҫ�޸ĵĺ궨�������ļ� */
    #include <All_User_Config.h>
    #include "ACMConfig.h"
    #include "ACMSim.h"
/* Logic  -----------------------------------------------------------------------------------*/
    //#include "Logic.h"                //�߼��� �������ϴ��룬DI,״̬��
    //extern int _gExpState;
    extern int Enable_START_FLAG;
    extern int Enable_STOP_FLAG;
    extern int VoltageOVER_FLAG;
    struct Trip_Variables
    {
        BOOL VoltageOVER_FLAG;
        BOOL CurrentOVER_FLAG;
    };
    extern struct Trip_Variables trip;//trip flag
    #define Motor_mode_START    GpioDataRegs.GPADAT.bit.GPIO26          //DI Start Button
    int Motor_MODE_START(void);
    int Motor_MODE_STOP(void);
    int Motor_MODE_REVERSE(void);
    void System_Checking(void);
/* Driver -----------------------------------------------------------------------------------*/
    #include "DAC_MAX5307.h"
    #include "F2837xD_Ipc_drivers.h" // ˫��ͨѶ
    #include "F2837xD_struct.h"
    #include "F2837xD_sdfm_drivers.h"
    #include "ShareMemory.h"
    void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);//flash
/*  Motor Library file-----------------------------------------------------------------------------------*/
    //δ����Ҫ�޸ĳɽṹ��
    interrupt void CJHMainISR(void);
    interrupt void EQEP_UTO_INT(void);
    extern float Current_W,Current_V,Current_U,Voltage_DC_BUS,Controller_sample_Ts;
    extern float Set_maunal_current_iq,Set_maunal_current_id,Set_maunal_rpm;
    typedef struct     { float  Ualpha;            // Input: reference alpha-axis phase voltage
                          float  Ubeta;         // Input: reference beta-axis phase voltage
                          float  Ta;                // Output: reference phase-a switching function
                          float  Tb;                // Output: reference phase-b switching function
                          float  Tc;                // Output: reference phase-c switching function
                    } SVGENDQ;
    extern SVGENDQ svgen1;
/* Hardware Peripherals Configuration -----------------------------------------------------------------------------------*/
    //extern QEP qep1;
    extern BOOL AD_offset_flag;
    extern float offsetA,offsetB,offsetC;//ADC offset
    //extern float Controller_sample_Ts,Rated_elec_frequency,Rated_Angular_elec_speed;
    void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
    void ePWM_initialize(void);
    void Gpio_initialize(void);
    void ADC_initialize(void);
    #define DSP_ENPWM  GpioDataRegs.GPDSET.bit.GPIO105=1;     //SD�øߣ��Ⲩ
    #define STOP_LED1  GpioDataRegs.GPDCLEAR.bit.GPIO124=1;
    #define STOP_LED2  GpioDataRegs.GPBCLEAR.bit.GPIO33=1;
    #define START_LED1 GpioDataRegs.GPDSET.bit.GPIO124=1;
    #define START_LED2 GpioDataRegs.GPBSET.bit.GPIO33=1;
    //PWM CONFIGURATION
    #define SYSTEM_PWM_DEADTIME               5     // us   PWM deadtime
    #define SYSTEM_PWM_DB_ACTV                0x1     //DB_ACTV_LOC  0x1   DB_ACTV_HIC  0x2 // LOW EFFECT /HIGH EFFECT
    #define SYSTEM_PWM_AQ_SET                 0x2     //AQ_SET;    OTHERWISE, REVERSE!
    #define SYSTEM_PWM_AQ_CLEAR               0x1     //AQ_CLEAR;
    //QEP CONFIGURATION
    #define SYSTEM_QEP_CALIBRATED_ANGLE     -2668     // CALIBRATED_ANGLE
    #define SYSTEM_QEP_LINE                  2500     //encoder line
    #define SYSTEM_QEP_POLE_PAIRS               4     //pairs
    #define SYSTEM_QEP_UNITTIME_ISR          0.001    //1K ,1ms     time_out timer   isr_time
    #define SYSTEM_QEP_CAP_X128           0.00000064  //cap timer
    #define SYSTEM_QEP_CUTOFF_FILTER           5      // CUTOFF FREQUENCY 10HZ
    #define SYSTEM_QEP_SWAP_ENABLE              1     //���������
    #define SYSTEM_QEP_SWAP_DISABLE             0     //���������
    //ADC CONFIGURATION
    #define AD_scale_W                       0.005491759 //0.00558484 //0.0057316444 // -11.8��11.8 A // 0.0062421972 //-12-12A
    #define AD_scale_V                       0.005491759 //0.00558484 //0.0056072670 // -11.8��11.8 A // 0.0061728395 //-12-12A
    #define AD_scale_VDC                     0.1897533207 // 0��800 V
    //TRIP CONFIGURATION
    #define MAX_CURRENT_N                      -11.8      //-12A  // Set your negative current trip threshold here in [0, 4095]
    #define MAX_CURRENT_P                       11.8       //12A   // Set your positive current trip threshold here in [0, 4095]
    #define MAX_OVERVOLTAGE                    400      //400V  // Set your unipolar trip Over-voltage threshold in [0, 4095]
    //SVPWM CONFIGURATION
    #define MAX_PWM_LIMATATION                    0.96
    #define MIN_PWM_LIMATATION                    0.04
    //GPIO
    #define DSP_ENPWM               GpioDataRegs.GPDSET.bit.GPIO105=1;    //SD�øߣ��Ⲩ
    #define DSP_ENPWM_LOW           GpioDataRegs.GPDCLEAR.bit.GPIO105=1;
    #define DSP_2ENPWM              GpioDataRegs.GPASET.bit.GPIO27=1;     //SD�øߣ��Ⲩ
    #define DSP_2ENPWM_LOW          GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    #define DSP_EPWM_DISABLE        GpioDataRegs.GPDSET.bit.GPIO105=1;    // ����Ч����λ�Ⲩ
    #define DSP_EPWM_ENABLE         GpioDataRegs.GPDCLEAR.bit.GPIO105=1;  // ����Ч��������Ч
    #define DSP_2EPWM_DISABLE       GpioDataRegs.GPASET.bit.GPIO27=1;
    #define DSP_2EPWM_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    #define INVERTER_FLT_FAULT      GpioDataRegs.GPDDAT.bit.GPIO104  //Inverter_error signal
#endif