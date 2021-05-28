#include <All_Definition.h>
void init_experiment_offset(){
    /* ADC OFFSET */
    #ifdef _XCUBE1
        G.offsetU=2044;
        G.offsetV=2052;
        G.offsetW=2032; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
        G.offsetUDC=1430;
        // /* 借用Sensor Board的电流传感器 */
        // G.offsetU=2045;
        // G.offsetV=2047;
        // G.offsetW=2030;
    #else
        G.offsetU=2072;
        G.offsetV=2062;
        G.offsetW=2049; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
        G.offsetUDC=2; // 5.18
    #endif

    /* ADC SCALE */
    #ifdef _XCUBE1
        G.AD_scale_U   = 0.0109649;
        G.AD_scale_V   = 0.00988908653610677; // (degaussed CP030A and offset ADC) // 0.01125872551 (YX)
        G.AD_scale_W   = 0.00972075677822004; // (degaussed CP030A and offset ADC) // 0.01134558656 (YX)
        G.AD_scale_VDC = 0.3546099;
            /* 借用Sensor Board的电流传感器 */
            //#define AD_scale_W                       0.0057316444
            //#define AD_scale_V                       0.0056072670
    #else
        G.AD_scale_W   = 0.0058002658;       // 0.00867354 // 0.0096392 // 0.0057316444
        G.AD_scale_V   = 0.0064003544;       // 0.01027935 //0.0094433 // 0.0056072670
        G.AD_scale_U   = 0.006300196648;
        G.AD_scale_VDC = (0.5*0.1897533207); // 短上一个470R再补上一个470R
    #endif

    /* eQEP OFFSET */
    CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis = SYSTEM_QEP_CALIBRATED_ANGLE;
    CTRL.enc->theta_d_offset = CTRL.enc->OffsetCountBetweenIndexAndUPhaseAxis * CNT_2_ELEC_RAD;
}
void main(void){

    InitSysCtrl();
    DINT;                 // Disable CPU interrupts
    InitPieCtrl();        // Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    IER = 0x0000;         // Disable CPU __interrupts and clear all CPU __interrupt flags
    IFR = 0x0000;         // Disable CPU __interrupts and clear all CPU __interrupt flags
    InitPieVectTable();   // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    #if NUMBER_OF_DSP_CORES == 2
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #endif
    #ifdef _STANDALONE
    #ifdef _FLASH
        // 当你需要离线断电再上电运行时用这个：
        //  Send boot command to allow the CPU02 application to begin execution
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #else
        //  Send boot command to allow the CPU02 application to begin execution
        // 这句话我不知道什么意义，可能还是不要比较好。
        //IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
    #endif
    #endif

    Gpio_initialize();
    ePWM_initialize();
    ADC_initialize();
    eQEP_initialize(0);
    InitECaptureContinuousMode();
    init_experiment();
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

    /* PIE Vector Table */
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM主中断 10kKHz
    PieVectTable.ECAP1_INT = &ecap1_isr;
    PieVectTable.ECAP2_INT = &ecap2_isr;
    PieVectTable.ECAP3_INT = &ecap3_isr;
    PieVectTable.EQEP1_INT = &EQEP_UTO_INT;      // eqep
    EDIS; // This is needed to disable write to EALLOW protected registers

    /* PIE Control */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // PWM1 interrupt (Interrupt 3.1)
#if USE_ECAP_CEVT2_INTERRUPT == 1
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // 1 Enable for Interrupt 4.2
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // 2 Enable for Interrupt 4.3
#endif
#if SYSTEM_PROGRAM_MODE != 223
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;      // QEP interrupt
#endif

    /* CPU Interrupt Enable Register (IER) */
    IER |= M_INT3;  // EPWM1_INT
#if USE_ECAP_CEVT2_INTERRUPT == 1
    IER |= M_INT4;  // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
#endif
    IER |= M_INT5;  // EQEP1_INT???

    EINT;   // Enable Global __interrupt INTM
    ERTM;   // Enable Global realtime __interrupt DBGM

    STOP_LED1;
    STOP_LED2;
    DSP_ENPWM;
    DSP_2ENPWM;

    /* Test IPC to CPU02 */
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
            G.FLAG_ENABLE_PWM_OUTPUT = 1;
            //Enable_START_FLAG=1;
            //Enable_STOP_FLAG=0;
            START_LED1
            START_LED2
        }else if (Motor_mode_START==0){
            G.FLAG_ENABLE_PWM_OUTPUT = 0;
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
