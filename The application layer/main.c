#include <All_Definition.h>
__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
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

    /* PIE Vector Table */
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM主中断 10kKHz
    PieVectTable.ECAP1_INT = &ecap1_isr;
//    PieVectTable.ECAP2_INT = &ecap2_isr;
//    PieVectTable.ECAP3_INT = &ecap3_isr;
    #if SYSTEM_PROGRAM_MODE != 223
    PieVectTable.EQEP1_INT = &EQEP_UTO_INT;      // eqep
    #endif
    EDIS; // This is needed to disable write to EALLOW protected registers

    /* PIE Control */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // PWM1 interrupt (Interrupt 3.1)
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // 1 Enable for Interrupt 4.2
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // 2 Enable for Interrupt 4.3

    #if SYSTEM_PROGRAM_MODE != 223
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;      //QEP interrupt
    #endif

    /* CPU Interrupt Enable Register (IER) */
    IER |= M_INT3;  // EPWM1_INT
//    IER |= M_INT4;  // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
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


Uint32 good_capture_counter_U[4] = {0,0,0,0};
Uint32 good_capture_counter_V[4] = {0,0,0,0};
Uint32 good_capture_counter_W[4] = {0,0,0,0};
Uint64 ECap1IntCount=0;
/*U*/
u_bad_value_on1  = 0;
u_bad_value_off1 = 0;
u_bad_value_on2  = 0;
u_bad_value_off2 = 0;
__interrupt void ecap1_isr(void){

// 如果采用下面这个代码，不能保证EPWM1中断来的时候已经采集到了当前的电压，更新频率也不够快！
// 所以应该把ECAP中断改成一个脉冲中断一次，并检查其优先级。

    ECap1IntCount++;

    REAL CAP1, CAP2, CAP3, CAP4;
    CAP1 = good_capture_counter_U[3];
    CAP2 = good_capture_counter_U[4];
    CAP3 = ECap1Regs.CAP1;
    CAP4 = ECap1Regs.CAP2;

    REAL DutyOnTime1  = CAP2;
    REAL DutyOffTime1 = CAP3;
    REAL DutyOnTime2  = CAP4;
    REAL DutyOffTime2 = CAP1;
    REAL Period1 = DutyOnTime1 + DutyOffTime1;
    REAL Period2 = DutyOnTime2 + DutyOffTime2;

    /* This is not the real period, if we use symmetric PWM,
     * the center location of the square wave high
     * to the next center location of the square wave high
     * is the period. */

    //if(Period1 < SYSTEM_HALF_PWM_MAX_COUNT){
    //if(Period2 < SYSTEM_HALF_PWM_MAX_COUNT){

    // On1 Off1 一起跌落(CAP2 and CAP3)
    if( DutyOnTime1 + DutyOffTime1 < SYSTEM_HALF_PWM_MAX_COUNT ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        u_bad_value_on1  = 0;
        u_bad_value_off1 = 0;
    }
    // On2 Off2 一起跌落(CAP4 and CAP1)
    if( DutyOnTime2 + DutyOffTime2 < SYSTEM_HALF_PWM_MAX_COUNT ){
        u_bad_value_on2  = 0;
        u_bad_value_off2 = 0;
    }

    //    // 第一个脉冲的下降沿和第二个脉冲的上升，如果两者加起来特别小，则说明是跌落尖峰
    //    if(ECap1Regs.CAP2 + ECap1Regs.CAP3 < SYSTEM_HALF_PWM_MAX_COUNT){
    //        // 高电平时出现跌落尖峰，这两个CAP2/3作废，
    //        // 但是此时的CAP1和4实际上是真正的CAP1和CAP2，而真正的CAP3和CAP4并没有被抓到。
    //        good_capture_counter_U[0] = ECap1Regs.CAP1;
    //        good_capture_counter_U[1] = ECap1Regs.CAP4;
    //    }else{
    //        // 有效
    //        good_capture_counter_U[1] = ECap1Regs.CAP2;
    //        good_capture_counter_U[2] = ECap1Regs.CAP3;
    //    }
    //    // 第二个脉冲的下降沿和第一二个脉冲的上升，如果两者加起来特别小，则说明是突升尖峰
    //    if(ECap1Regs.CAP4 + ECap1Regs.CAP1 < SYSTEM_HALF_PWM_MAX_COUNT){
    //        // 低高电平时出现突升尖峰，这两个CAP4/1作废
    //    }else{
    //        // 有效
    //        good_capture_counter_U[3] = ECap1Regs.CAP4;
    //        good_capture_counter_U[0] = ECap1Regs.CAP1;
    //    }

    //    do_enhanced_capture();

    //ECap1Regs.ECCLR.bit.CEVT4 = 1; // 4 events == __interrupt
    ECap1Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap1Regs.ECCLR.bit.INT = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
