/*
 * main.c
 * Author: yuanxin and jiahao
 * 2021.01.11
 * @NTU-IQMATH
 */

#include <All_Definition.h>

SVGENDQ svgen1;
float Current_U=0.0, Current_V=0.0, Current_W=0.0, Voltage_DC_BUS=0.0;
int FLAG_ENABLE_PWM_OUTPUT; // 电机模式标志位
//int Enable_STOP_FLAG;  // 电机模式标志位
Uint16 Rotor_angle_selection=SYSTEM_QEP_ROTOR_ANGLE;
float Set_maunal_current_iq=0,Set_maunal_current_id=0,Set_maunal_rpm=300;
float offsetU=2070,offsetV=2088,offsetW=2055; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
int DAC_MAX5307_FLAG=0;
//BOOL AD_offset_flag = FALSE;
BOOL AD_offset_flag2 = FALSE;
void eQEP_initialize(int m);

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
