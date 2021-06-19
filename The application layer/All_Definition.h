/*  All experimental definitions Header File
 *  Created on: 2021年1月15日
 *      Author: yuanxin and jiahao
 *       */
#ifndef ALL_DEFINITATION_H
#define ALL_DEFINITATION_H
#include "math.h"
//#include "IQmathLib.h"
/* DSP system Configuration------------------------------------------------------------------*/
    #include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
    #include "F2837xD_device.h"        // F2837xD Headerfile Include File
    #include "F2837xD_Examples.h"      // F2837xD Examples Include File
/* 经常需要修改的宏定义声明文件 */
    #include "ACMConfig.h"
    #include "ACMSim.h"
    #include "Experiment.h"
/* Motor Library file------------------------------------------------------------------------*/
    __interrupt void EPWM1ISR(void);
    #define ENABLE_ECAP 0
    #define USE_ECAP_CEVT2_INTERRUPT 1
    __interrupt void ecap1_isr(void);
    __interrupt void ecap2_isr(void);
    __interrupt void ecap3_isr(void);
    __interrupt void EQEP_UTO_INT(void);
    __interrupt void scicRxFifoIsr(void);
    __interrupt void scicTxFifoIsr(void);
    void SVGEN_Drive(SVGENDQ* ptrV);
/* Driver -----------------------------------------------------------------------------------*/
    #include "DAC_MAX5307.h"
    #include "ECaptureVoltage.h"
    #include "F2837xD_Ipc_drivers.h" // 双核通讯
    #include "F2837xD_struct.h"
    #include "F2837xD_sdfm_drivers.h"
    #include "ShareMemory.h"
    void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);//flash
/* Hardware Peripherals Configuration -------------------------------------------------------*/
    void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
    void ePWM_initialize(void);
    void eQEP_initialize(int m);
    void Gpio_initialize(void);
    void ADC_initialize(void);
    //PWM CONFIGURATION
    #define SYSTEM_PWM_DEADTIME               5     // us   PWM deadtime
    #define SYSTEM_PWM_DB_ACTV                0x1     //DB_ACTV_LOC  0x1   DB_ACTV_HIC  0x2 // LOW EFFECT /HIGH EFFECT
    #define SYSTEM_PWM_AQ_SET                 0x2     //AQ_SET;    OTHERWISE, REVERSE!
    #define SYSTEM_PWM_AQ_CLEAR               0x1     //AQ_CLEAR;
    //QEP CONFIGURATION
    #define USE_ORIGINAL_INVERTER_MOTOR_PAIR
    #ifdef USE_ORIGINAL_INVERTER_MOTOR_PAIR
        #ifdef _XCUBE1
            #define SYSTEM_QEP_CALIBRATED_ANGLE -2668 // for MOTOR2
        #else
            #define SYSTEM_QEP_CALIBRATED_ANGLE -976 // for MOTOR1 (w/ hall sensor) // -968 for MOTOR1
        #endif
    #else
        #ifdef _XCUBE1
            #define SYSTEM_QEP_CALIBRATED_ANGLE -976 // for MOTOR1 (w/ hall sensor) // -968 for MOTOR1
        #else
            #define SYSTEM_QEP_CALIBRATED_ANGLE -2668 // for MOTOR2
        #endif
    #endif
    #define SYSTEM_QEP_LINE                  2500     //encoder line
    #define SYSTEM_QEP_POLE_PAIRS               4     //pairs
    #define SYSTEM_QEP_UNITTIME_ISR          0.001    //1K ,1ms     time_out timer   isr_time
    #define SYSTEM_QEP_CAP_X128           0.00000064  //cap timer
    #define SYSTEM_QEP_CUTOFF_FILTER           5      // CUTOFF FREQUENCY 10HZ
    #define SYSTEM_QEP_SWAP_ENABLE              1     //正方向计数
    #define SYSTEM_QEP_SWAP_DISABLE             0     //反方向计数
    //ADC CONFIGURATION is moved to main.c
    //TRIP CONFIGURATION
    #define MAX_CURRENT_N                      -11.8      //-12A  // Set your negative current trip threshold here in [0, 4095]
    #define MAX_CURRENT_P                       11.8       //12A   // Set your positive current trip threshold here in [0, 4095]
    #define MAX_OVERVOLTAGE                    400      //400V  // Set your unipolar trip Over-voltage threshold in [0, 4095]
    //SVPWM CONFIGURATION
    #define MAX_PWM_LIMATATION                    0.96
    #define MIN_PWM_LIMATATION                    0.04
    //GPIO
    #define DSP_PWM_DISABLE        GpioDataRegs.GPDSET.bit.GPIO105=1;    // 低有效，置位封波
    #define DSP_PWM_ENABLE         GpioDataRegs.GPDCLEAR.bit.GPIO105=1;  // 低有效，清零有效
    #define DSP_2PWM_DISABLE       GpioDataRegs.GPASET.bit.GPIO27=1;
    #define DSP_2PWM_ENABLE        GpioDataRegs.GPACLEAR.bit.GPIO27=1;

    #define INVERTER_FLT_FAULT     GpioDataRegs.GPDDAT.bit.GPIO104  //Inverter_error signal

    #define DSP_STOP_LED1  GpioDataRegs.GPDCLEAR.bit.GPIO124=1;
    #define DSP_STOP_LED2  GpioDataRegs.GPBCLEAR.bit.GPIO33=1;
    #define DSP_START_LED1 GpioDataRegs.GPDSET.bit.GPIO124=1;
    #define DSP_START_LED2 GpioDataRegs.GPBSET.bit.GPIO33=1;

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
#endif
