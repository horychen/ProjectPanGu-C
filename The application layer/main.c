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
        // ������Ҫ���߶ϵ����ϵ�����ʱ�������
        //  Send boot command to allow the CPU02 application to begin execution
        IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    #else
        //  Send boot command to allow the CPU02 application to begin execution
        // ��仰�Ҳ�֪��ʲô���壬���ܻ��ǲ�Ҫ�ȽϺá�
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
        // GPIO����
        InitSpiaGpio();
        InitSpi();
    #elif NUMBER_OF_DSP_CORES == 2
        /* ˫������*/
        // ��ʼ��SPI��������DACоƬMAX5307ͨѶ��
        EALLOW;
        DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
        EDIS;
        InitSpiaGpio();
        //InitSpi(); // this is moved to CPU02
        // �ڴ�֮ǰ���Ѿ���GPIO�������Ȩ��ת��CPU2�ˡ�
        // �����ٰѲ��ֹ����ڴ�Ȩ�޸�CPU2��ͬʱ����CPU2������Լ������д����ˡ�
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
    PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM;     //&MainISR;      // PWM���ж� 10kKHz
    PieVectTable.ECAP1_INT = &ecap1_isr;
    PieVectTable.ECAP2_INT = &ecap2_isr;
    PieVectTable.ECAP3_INT = &ecap3_isr;
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
    IER |= M_INT4;  // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
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
        //System_Checking();  //״̬����һ��״̬ ��ϵͳ�Լ�
        //System_Protection();//IU/IV/VOLTAGE����

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


Uint32 good_capture_U[4] = {0,0,10000,10000};
Uint32 good_capture_V[4] = {0,0,10000,10000};
Uint32 good_capture_W[4] = {0,0,10000,10000};
struct bad_values {
    Uint32 on1, off1, on2, off2;
} u_ecap_bad, v_ecap_bad, w_ecap_bad;

Uint64 ECapIntCount[3]={0,0,0};
Uint64 ECapPassCount[3]={0,0,0};

extern int password_isr_nesting;
int decipher_password[3]={0,0,0};

/*U*/
__interrupt void ecap1_isr(void){

    // isr nesting test
    decipher_password[0] = password_isr_nesting + 21;

    ECapIntCount[0]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = good_capture_U[2];
    CAP2 = good_capture_U[3];
    CAP3 = ECap1Regs.CAP1;
    CAP4 = ECap1Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3;
    Uint32 DutyOnTime2  = CAP4;
    Uint32 DutyOffTime2 = CAP1;

        //Uint32 Period1 = DutyOnTime1 + DutyOffTime1;
        //Uint32 Period2 = DutyOnTime2 + DutyOffTime2;
        /* This is not the actual period, if we use symmetric PWM,
         * the center location of the square wave high
         * to the next center location of the square wave high
         * is the period. */
        //if(Period1 < SYSTEM_HALF_PWM_MAX_COUNT){ Wrong! Period1 can be larger than 10000
        //if(Period2 < SYSTEM_HALF_PWM_MAX_COUNT){

    // On1 Off1 һ�����(CAP2 and CAP3) 
    // ��4444��ô������֣�������444������Ϊ�ҷ�����ʱ��V������õ���ON1��OFF1�ֱ�Ϊ9000+��9000+��������С��19000���ж�Ϊbad��������Ӧ�������������Ĳ���
    // ���ֵ��ǣ�V���CAP1��ʱ����Ϊ20000+����������Ϊʲô��
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        u_ecap_bad.on1  = DutyOnTime1;
        u_ecap_bad.off1 = DutyOffTime1;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        u_ecap_bad.on2  = DutyOnTime2;
        u_ecap_bad.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        ECapPassCount[0] += 1;
        good_capture_U[0] = CAP1;
        good_capture_U[1] = CAP2;
        good_capture_U[2] = CAP3;
        good_capture_U[3] = CAP4;
    }

        /* For ECap1Regs.ECEINT.bit.CEVT4 = 1; */
        // �����������������룬���ܱ�֤EPWM1�ж�����ʱ���Ѿ��ɼ����˵�ǰ�ĵ�ѹ������Ƶ��Ҳ�����죡
        // ����Ӧ�ð�ECAP�жϸĳ�һ�������ж�һ�Σ�����������ȼ���

        //    // ��һ��������½��غ͵ڶ��������������������߼������ر�С����˵���ǵ�����
        //    if(ECap1Regs.CAP2 + ECap1Regs.CAP3 < SYSTEM_HALF_PWM_MAX_COUNT){
        //        // �ߵ�ƽʱ���ֵ����壬������CAP2/3���ϣ�
        //        // ���Ǵ�ʱ��CAP1��4ʵ������������CAP1��CAP2����������CAP3��CAP4��û�б�ץ����
        //        good_capture_counter_U[0] = ECap1Regs.CAP1;
        //        good_capture_counter_U[1] = ECap1Regs.CAP4;
        //    }else{
        //        // ��Ч
        //        good_capture_counter_U[1] = ECap1Regs.CAP2;
        //        good_capture_counter_U[2] = ECap1Regs.CAP3;
        //    }
        //    // �ڶ���������½��غ͵�һ���������������������߼������ر�С����˵����ͻ�����
        //    if(ECap1Regs.CAP4 + ECap1Regs.CAP1 < SYSTEM_HALF_PWM_MAX_COUNT){
        //        // �͸ߵ�ƽʱ����ͻ����壬������CAP4/1����
        //    }else{
        //        // ��Ч
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



/*V*/
__interrupt void ecap2_isr(void){

    // isr nesting test
    decipher_password[1] = password_isr_nesting + 22;

    ECapIntCount[1]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = good_capture_V[2];
    CAP2 = good_capture_V[3];
    CAP3 = ECap2Regs.CAP1;
    CAP4 = ECap2Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3;
    Uint32 DutyOnTime2  = CAP4;
    Uint32 DutyOffTime2 = CAP1;

    // On1 Off1 һ�����(CAP2 and CAP3)
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        v_ecap_bad.on1  = DutyOnTime1;
        v_ecap_bad.off1 = DutyOffTime1;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        v_ecap_bad.on2  = DutyOnTime2;
        v_ecap_bad.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        ECapPassCount[1] += 1;
        good_capture_V[0] = CAP1;
        good_capture_V[1] = CAP2;
        good_capture_V[2] = CAP3;
        good_capture_V[3] = CAP4;
    }

    ECap2Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap2Regs.ECCLR.bit.INT = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


/*W*/
__interrupt void ecap3_isr(void){

    // isr nesting test
    decipher_password[2] = password_isr_nesting + 23;

    ECapIntCount[2]++;

    Uint32 CAP1, CAP2, CAP3, CAP4;
    CAP1 = good_capture_W[2];
    CAP2 = good_capture_W[3];
    CAP3 = ECap3Regs.CAP1;
    CAP4 = ECap3Regs.CAP2;

    Uint32 DutyOnTime1  = CAP2;
    Uint32 DutyOffTime1 = CAP3;
    Uint32 DutyOnTime2  = CAP4;
    Uint32 DutyOffTime2 = CAP1;

    // On1 Off1 һ�����(CAP2 and CAP3)
    if( ( (DutyOnTime1+DutyOffTime1<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime1+DutyOffTime1>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime1<50 || DutyOffTime1<50 ) ){
        // This clause should never be entered when ECap1Regs.ECEINT.bit.CEVT2 = 1;
        w_ecap_bad.on1  = DutyOnTime1;
        w_ecap_bad.off1 = DutyOffTime1;
    }
    // On2 Off2 һ�����(CAP4 and CAP1)
    else if( ( (DutyOnTime2+DutyOffTime2<SYSTEM_PWM_MAX_COUNT-4444) || (DutyOnTime2+DutyOffTime2>SYSTEM_PWM_MAX_COUNT+4444) )
        || // and or or? 
        ( DutyOnTime2<50 || DutyOffTime2<50 ) ){
        w_ecap_bad.on2  = DutyOnTime2;
        w_ecap_bad.off2 = DutyOffTime2;
    }
    // Everything is good?
    else{
        ECapPassCount[2] += 1;
        good_capture_W[0] = CAP1;
        good_capture_W[1] = CAP2;
        good_capture_W[2] = CAP3;
        good_capture_W[3] = CAP4;
    }

    ECap3Regs.ECCLR.bit.CEVT2 = 1; // 2 events == __interrupt
    ECap3Regs.ECCLR.bit.INT = 1;
    ECap3Regs.ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
