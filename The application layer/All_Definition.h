/*
 *  Created on: 2021年1月15日
 *      Author: yuanxin and jiahao
 *       */
#ifndef ALL_DEFINITATION_H
#define ALL_DEFINITATION_H
#include "math.h"
//#include "IQmathLib.h"            //硬件库 StepAngle
/* DSP system Configuration------------------------------------------------------------------*/
    #include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
    #include "F2837xD_device.h"        // F2837xD Headerfile Include File
    #include "F2837xD_Examples.h"      // F2837xD Examples Include File
/* 经常需要修改的宏定义声明文件 */
    #include "ACMConfig.h"
    #include "ACMSim.h"
/* Logic  -----------------------------------------------------------------------------------*/
    //#include "Logic.h"                //逻辑库 包括故障代码，DI,状态机
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
    #include "ECaptureVoltage.h"
    #include "F2837xD_Ipc_drivers.h" // 双核通讯
    #include "F2837xD_struct.h"
    #include "F2837xD_sdfm_drivers.h"
    #include "ShareMemory.h"
    void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);//flash
/* Motor Library file------------------------------------------------------------------------*/
    //未来需要修改成结构体
    __interrupt void EPWM1ISR(void);
    __interrupt void ecap1_isr(void);
    __interrupt void ecap2_isr(void);
    __interrupt void ecap3_isr(void);
    __interrupt void EQEP_UTO_INT(void);
    void SVGEN_Drive(SVGENDQ* ptrV);
/* Hardware Peripherals Configuration -------------------------------------------------------*/
    void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
    void ePWM_initialize(void);
    void eQEP_initialize(int m);
    void Gpio_initialize(void);
    void ADC_initialize(void);
    #define DSP_ENPWM  GpioDataRegs.GPDSET.bit.GPIO105=1;     //SD置高，封波
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
    //#define SYSTEM_QEP_CALIBRATED_ANGLE     -2668     // CALIBRATED_ANGLE
    #define SYSTEM_QEP_LINE                  2500     //encoder line
    #define SYSTEM_QEP_POLE_PAIRS               4     //pairs
    #define SYSTEM_QEP_UNITTIME_ISR          0.001    //1K ,1ms     time_out timer   isr_time
    #define SYSTEM_QEP_CAP_X128           0.00000064  //cap timer
    #define SYSTEM_QEP_CUTOFF_FILTER           5      // CUTOFF FREQUENCY 10HZ
    #define SYSTEM_QEP_SWAP_ENABLE              1     //正方向计数
    #define SYSTEM_QEP_SWAP_DISABLE             0     //反方向计数
    //ADC CONFIGURATION
    //    #define AD_scale_W                       0.005491759 //0.00558484     //0.0057316444 // -11.8—11.8 A // 0.0062421972 //-12-12A
    //    #define AD_scale_V                       0.005491759 //0.00558484     //0.0056072670 // -11.8—11.8 A // 0.0061728395 //-12-12A
    //    #define AD_scale_U                       0.005491759
    #ifdef _XCUBE1
        #define AD_scale_U                       0.0109649
        #define AD_scale_V                       0.01125872551
        #define AD_scale_W                       0.01134558656
        #define AD_scale_VDC                     0.3546099

/* 借用Sensor Board的电流传感器 */
//#define AD_scale_W                       0.0057316444
//#define AD_scale_V                       0.0056072670

    #else
        // test condition, Ta=0.5, Tb=0.4, Tc=0.6, W=1.708, V=-1.693.
        //        #define AD_scale_W                       0.001002814052101 // 0.001050801473 / (1.708/1.63)   // 120 Ohm: 0.0057316444 / (120/22)
        //        #define AD_scale_V                       0.000989745002067 // 0.001027998950 / (1.693 / 1.63) //  120 Ohm: 0.0056072670 / (120/22)
        //        #define AD_scale_U                       0.0057316444
        //        #define AD_scale_VDC                     0.3546099

        #define AD_scale_W                       0.0058002658 // 0.00867354 // 0.0096392 // 0.0057316444
        #define AD_scale_V                       0.0064003544 // 0.01027935 //0.0094433 // 0.0056072670
        #define AD_scale_U                       0.006300196648
        #define AD_scale_VDC                     (0.5*0.1897533207) // 短上一个470R再补上一个470R
        //#define AD_scale_VDC                     0.1897533207
    #endif

    //TRIP CONFIGURATION
    #define MAX_CURRENT_N                      -11.8      //-12A  // Set your negative current trip threshold here in [0, 4095]
    #define MAX_CURRENT_P                       11.8       //12A   // Set your positive current trip threshold here in [0, 4095]
    #define MAX_OVERVOLTAGE                    400      //400V  // Set your unipolar trip Over-voltage threshold in [0, 4095]
    //SVPWM CONFIGURATION
    #define MAX_PWM_LIMATATION                    0.96
    #define MIN_PWM_LIMATATION                    0.04
    //GPIO
    #define DSP_ENPWM               GpioDataRegs.GPDSET.bit.GPIO105=1;    //SD置高，封波
    #define DSP_ENPWM_LOW           GpioDataRegs.GPDCLEAR.bit.GPIO105=1;
    #define DSP_2ENPWM              GpioDataRegs.GPASET.bit.GPIO27=1;     //SD置高，封波
    #define DSP_2ENPWM_LOW          GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    #define DSP_EPWM_DISABLE        GpioDataRegs.GPDSET.bit.GPIO105=1;    // 低有效，置位封波
    #define DSP_EPWM_ENABLE         GpioDataRegs.GPDCLEAR.bit.GPIO105=1;  // 低有效，清零有效
    #define DSP_2EPWM_DISABLE       GpioDataRegs.GPASET.bit.GPIO27=1;
    #define DSP_2EPWM_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    #define INVERTER_FLT_FAULT      GpioDataRegs.GPDDAT.bit.GPIO104  //Inverter_error signal
#endif
