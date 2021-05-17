/*
 * main.c
 * Author: yuanxin and jiahao
 * 2021.01.11
 * @NTU-IQMATH
 */

#include <All_Definition.h>

SVGENDQ svgen1;
float Current_U=0.0, Current_V=0.0, Current_W=0.0, Current_Not_Used=0.0, Voltage_DC_BUS=0.0;
int FLAG_ENABLE_PWM_OUTPUT; // 电机模式标志位
//int Enable_STOP_FLAG;  // 电机模式标志位
Uint16 Rotor_angle_selection=SYSTEM_QEP_ROTOR_ANGLE;
float Set_maunal_current_iq=0,Set_maunal_current_id=0,Set_maunal_rpm=300;
#ifdef _XCUBE1
    float offsetU=2044,offsetV=2047,offsetW=2032; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
    float offsetUDC=1430;

/* 借用Sensor Board的电流传感器 */
//float offsetU=2045,offsetV=2047,offsetW=2030; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.

#else
    float offsetU=2075,offsetV=2062,offsetW=2049; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
    float offsetUDC=5;
#endif
int DAC_MAX5307_FLAG=0;
//BOOL AD_offset_flag = FALSE;
BOOL AD_offset_flag2 = FALSE;
void eQEP_initialize(int m);


#if 1
    // From: D:\ti\controlSUITE\device_support\F2837xS\v210\F2837xS_examples_Cpu1\ecap_capture_pwm\cpu01\ECap_Capture_Pwm_cpu01.c
    // InitECapture - Initialize ECAP1 configurations
    void InitECapture()
    {

        /*ECAP 1*/
        ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
        ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
        ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
        ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

        //
        // Configure peripheral registers
        //
        ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
        ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
        ECap1Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
        ECap1Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
        ECap1Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
        ECap1Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
        ECap1Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
        ECap1Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
        ECap1Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
        ECap1Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
        ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
        ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
        ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

        ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
        ECap1Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
        ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
        ECap1Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt


        /*ECAP 2*/
        ECap2Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
        ECap2Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
        ECap2Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
        ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

        //
        // Configure peripheral registers
        //
        ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
        ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
        ECap2Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
        ECap2Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
        ECap2Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
        ECap2Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
        ECap2Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
        ECap2Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
        ECap2Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
        ECap2Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
        ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
        ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
        ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

        ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
        ECap2Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
        ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
        ECap2Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt

        /*ECAP 3*/
        ECap3Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
        ECap3Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
        ECap3Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
        ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

        //
        // Configure peripheral registers
        //
        ECap3Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot
        ECap3Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
        ECap3Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
        ECap3Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
        ECap3Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
        ECap3Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
        ECap3Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
        ECap3Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
        ECap3Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
        ECap3Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
        ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
        ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
        ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

        ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
        ECap3Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
        ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
        ECap3Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt
    }

    // ECCTL1 (ECAP Control Reg 1)
    //==========================
    // CAPxPOL bits
    #define EC_RISING 0x0
    #define EC_FALLING 0x1
    // CTRRSTx bits
    #define EC_ABS_MODE 0x0
    #define EC_DELTA_MODE 0x1
    // PRESCALE bits
    #define EC_BYPASS 0x0
    #define EC_DIV1 0x0
    #define EC_DIV2 0x1
    #define EC_DIV4 0x2
    #define EC_DIV6 0x3
    #define EC_DIV8 0x4
    #define EC_DIV10 0x5
    // ECCTL2 ( ECAP Control Reg 2)
    //==========================
    // CONT/ONESHOT bit
    #define EC_CONTINUOUS 0x0
    #define EC_ONESHOT 0x1
    // STOPVALUE bit
    #define EC_EVENT1 0x0
    #define EC_EVENT2 0x1
    #define EC_EVENT3 0x2
    #define EC_EVENT4 0x3
    // RE-ARM bit
    #define EC_ARM 0x1
    // TSCTRSTOP bit
    #define EC_FREEZE 0x0
    #define EC_RUN 0x1
    // SYNCO_SEL bit
    #define EC_SYNCIN 0x0
    #define EC_CTR_PRD 0x1
    #define EC_SYNCO_DIS 0x2
    // CAP/APWM mode bit
    #define EC_CAP_MODE 0x0
    #define EC_APWM_MODE 0x1
    // APWMPOL bit
    #define EC_ACTV_HI 0x0
    #define EC_ACTV_LO 0x1
    // Generic
    #define EC_DISABLE 0x0
    #define EC_ENABLE 0x1
    #define EC_FORCE 0x1
    void InitECaptureContinuousMode(){
        InitECapture();

        //        16.6.1
        // ECAP module 1 config
        //        ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CAP2POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CAP4POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
        //        ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
        //        ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
        //        ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
        //        ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
        //        ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
        //        ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

        //        16.6.2
        // ECAP module 1 config
        //        ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
        //        ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
        //        ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
        //        ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
        //        ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
        //        ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
        //        ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
        //        ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
        //        ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
        //        ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
        //        ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

        //        16.6.4
        // Code snippet for CAP mode Delta Time, Rising and Falling
        // edge triggers
        // Initialization Time
        //=======================
        // ECAP module 1 config
        ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
        ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
        ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
        ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
        ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
        ECap1Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
        ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
        ECap1Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
        ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
        ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
        ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
        ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
        ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
        ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
        ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

        // ECAP module 2 config
        ECap2Regs.ECCTL1.bit.CAP1POL = EC_RISING;
        ECap2Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
        ECap2Regs.ECCTL1.bit.CAP3POL = EC_RISING;
        ECap2Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
        ECap2Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
        ECap2Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
        ECap2Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
        ECap2Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
        ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
        ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
        ECap2Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
        ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
        ECap2Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
        ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
        ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;

        // ECAP module 3 config
        ECap3Regs.ECCTL1.bit.CAP1POL = EC_RISING;
        ECap3Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
        ECap3Regs.ECCTL1.bit.CAP3POL = EC_RISING;
        ECap3Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
        ECap3Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
        ECap3Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
        ECap3Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
        ECap3Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
        ECap3Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
        ECap3Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
        ECap3Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
        ECap3Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
        ECap3Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
        ECap3Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
        ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;
    }
#endif

void main(void){

    InitSysCtrl();
    DINT;                 // Disable CPU interrupts
    InitPieCtrl();        // Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;
    IFR = 0x0000;         // Disable CPU interrupts and clear all CPU interrupt flags
    InitPieVectTable();   // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    #if NUMBER_OF_DSP_CORES == 2
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #endif
            //    #ifdef _STANDALONE
            //    #ifdef _FLASH
            //        // 当你需要离线断电再上电运行时用这个：
            //        //  Send boot command to allow the CPU02 application to begin execution
            //        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
            //    #else
            //        //  Send boot command to allow the CPU02 application to begin execution
            //        // 这句话我不知道什么意义，可能还是不要比较好。
            //        //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
            //    #endif
            //    #endif

    Gpio_initialize();
    ePWM_initialize();
    ADC_initialize();
    eQEP_initialize(0);
    InitECaptureContinuousMode();
    experiment_init();
    #if NUMBER_OF_DSP_CORES == 1
        // GPIO配置
        InitSpiaGpio();
        InitSpi();
    #elif NUMBER_OF_DSP_CORES == 2
        /* 双核配置*/
        // 初始化SPI，用于与DAC芯片MAX5307通讯。
        EALLOW;
        DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
        EDIS;
        InitSpiaGpio();
        //InitSpi(); // this is moved to CPU02
        // 在此之前，已经把GPIO和外设的权限转给CPU2了。
        // 这里再把部分贡献内存权限给CPU2，同时告诉CPU2，你可以继续运行代码了。
        while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS0))
        {
            EALLOW;
            // Give Memory Access to GS0/ GS14 SARAM to CPU02
            MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
            EDIS;
        }
    #endif

    EALLOW;
    PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM主中断 10kKHz
    #if SYSTEM_PROGRAM_MODE != 223
    PieVectTable.EQEP1_INT = &EQEP_UTO_INT;      // eqep
    #endif
    EDIS;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      //PWM1 interrupt
    #if SYSTEM_PROGRAM_MODE != 223
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;      //QEP interrupt
    #endif

    IER |= M_INT5;
    IER |= M_INT3;                          // EPWM1_INT
    EINT;
    ERTM;

    //_gExpState=EXP_START_SYSTEM;

    STOP_LED1;
    STOP_LED2;

    DSP_ENPWM;
    DSP_2ENPWM;

    #if NUMBER_OF_DSP_CORES == 2
        Write.dac_buffer[0] = 0.5;
        Write.dac_buffer[1] = 0.5;
        Write.dac_buffer[2] = 0.5;
        Write.dac_buffer[3] = 0.5;
        Write.dac_buffer[4] = 0.5;
        Write.dac_buffer[5] = 0.5;
        Write.dac_buffer[6] = 0.5;
        Write.dac_buffer[7] = 0.5;
        Write.test = 200.0;
        IPCLtoRFlagSet(IPC_FLAG7);
    #endif

    while(1){
        //STATE_APP_MachineState();
        //System_Checking();  //状态机第一个状态 ：系统自检
        //System_Protection();//IU/IV/VOLTAGE保护

        //#define Motor_mode_START    GpioDataRegs.GPADAT.bit.GPIO26          //DI Start Button
        if (Motor_mode_START==1)
        {
            FLAG_ENABLE_PWM_OUTPUT = 1;
            //Enable_START_FLAG=1;
            //Enable_STOP_FLAG=0;
            START_LED1
            START_LED2
        }else if (Motor_mode_START==0){
            FLAG_ENABLE_PWM_OUTPUT = 0;
            //Enable_START_FLAG=0;
            //Enable_STOP_FLAG=1;
            STOP_LED1
            STOP_LED2
        }


        #if NUMBER_OF_DSP_CORES == 2
            if(IPCLtoRFlagBusy(IPC_FLAG10) == 1) // if flag
            {
                Write.test += 0.001;
                IPCRtoLFlagAcknowledge (IPC_FLAG10);
            }
        #endif
    }
}
