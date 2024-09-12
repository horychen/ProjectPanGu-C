#include "All_Definition.h"



Uint32 position_count_SCI_shank_fromCPU2;
Uint32 position_count_SCI_hip_fromCPU2;
Uint32 position_count_SCI_fromCPU2;
Uint32 position_count_CAN_ID0x01_fromCPU2;
Uint32 position_count_CAN_ID0x03_fromCPU2;
Uint32 position_count_CAN_fromCPU2;
Uint32 CPU2_commu_error_counter = 0;

int USE_3_CURRENT_SENSORS = TRUE;

REAL legBouncingSpeed = 50;
REAL hipBouncingFreq = 10;
REAL legBouncingIq = 2;
REAL hipBouncingIq = 1.5;
int bool_use_SCI_encoder = TRUE;

REAL target_position_cnt;
REAL target_position_cnt_shank = 58000;
REAL target_position_cnt_hip = 48000;
int bool_TEMP = FALSE;

// 声明全局变量
#if PC_SIMULATION == FALSE
REAL CpuTimer_Delta = 0;
Uint32 CpuTimer_Before = 0;
Uint32 CpuTimer_After = 0;
#endif

int counter_missing_position_measurement = 0;
int max_counter_missing_position_measurement = 0;


void EUREKA_GPIO_SETUP(){
// =========FOR EUREKA===========
    // https://www.ti.com/lit/ug/spruhm8j/spruhm8j.pdf?ts=1710763002632&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMS320F28377D%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Depd-c2x-null-44700045336317350_prodfolderdynamic-cpc-pf-google-wwe_int%2526utm_content%253Dprodfolddynamic%2526ds_k%253DDYNAMIC+SEARCH+ADS%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwzN-vBhAkEiwAYiO7oBh2RIh8aEZiiEHzGLlyxsdf34XwLyRi-Ci53QGwm9calxyVXTdsEBoC7IwQAvD_BwE%2526gclsrc%253Daw.ds
    // Chapter 8.7.1
    /*
    // GPIO62 - CANRXA
    GPIO_SetupPinMux(62, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_ASYNC);
    // GPIO19 - CANTXA
    GPIO_SetupPinMux(19, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO21 - CANRXB
    GPIO_SetupPinMux(21, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_ASYNC);
    // GPIO20 - CANTXB
    GPIO_SetupPinMux(20, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO136 - 485RX-SCIA
    GPIO_SetupPinMux(136, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(136, GPIO_INPUT, GPIO_PUSHPULL);
    // GPIO135 - 485TX-SCIA
    GPIO_SetupPinMux(135, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(135, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GPIO138 - 485RX-SCIB
    GPIO_SetupPinMux(138, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(138, GPIO_INPUT, GPIO_PUSHPULL);
    // GPIO137 - 485TX-SCIB
    GPIO_SetupPinMux(137, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(137, GPIO_OUTPUT, GPIO_PUSHPULL);

    //GPIO39 - SCIRX-C
    GPIO_SetupPinMux(39, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_PUSHPULL);
    //GPIO38 - SCITX-C
    GPIO_SetupPinMux(38, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(38, GPIO_OUTPUT, GPIO_PUSHPULL);
    */

    //CANRXA
    GPIO_SetupPinMux(62, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(62, GPIO_INPUT, GPIO_ASYNC);
    //CANTXA
    GPIO_SetupPinMux(19, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    //CANRXB
    GPIO_SetupPinMux(21, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_ASYNC);
    //CANTXB
    GPIO_SetupPinMux(20, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
    
    //485RX-SCIA
    GPIO_SetupPinMux(PIN_SCI_RXDA, GPIO_MUX_CPU2, MUX_SCI_RXDA);
    GPIO_SetupPinOptions(PIN_SCI_RXDA, GPIO_INPUT, GPIO_PUSHPULL);

    //485TX-SCIA
    GPIO_SetupPinMux(PIN_SCI_TXDA, GPIO_MUX_CPU2, MUX_SCI_TXDA);
    GPIO_SetupPinOptions(PIN_SCI_TXDA, GPIO_OUTPUT, GPIO_PUSHPULL);

    //485RX-SCIB
    GPIO_SetupPinMux(PIN_SCI_RXDB, GPIO_MUX_CPU2, MUX_SCI_RXDB);
    GPIO_SetupPinOptions(PIN_SCI_RXDB, GPIO_INPUT, GPIO_PUSHPULL);
    //485TX-SCIB
    GPIO_SetupPinMux(PIN_SCI_TXDB, GPIO_MUX_CPU2, MUX_SCI_TXDB);
    GPIO_SetupPinOptions(PIN_SCI_TXDB, GPIO_OUTPUT, GPIO_PUSHPULL);

    //SCIRX-C
    GPIO_SetupPinMux(PIN_SCI_RXDC, GPIO_MUX_CPU2, MUX_SCI_RXDC);
    GPIO_SetupPinOptions(PIN_SCI_RXDC, GPIO_INPUT, GPIO_PUSHPULL);
    //SCITX-C
    GPIO_SetupPinMux(PIN_SCI_TXDC, GPIO_MUX_CPU2, MUX_SCI_TXDC);
    GPIO_SetupPinOptions(PIN_SCI_TXDC, GPIO_OUTPUT, GPIO_PUSHPULL);

    // // GPIO31 - 485-SCIB-WE-(use SCICTX as GPIO, in UART3 pin7)
    // GPIO_SetupPinMux(31, GPIO_MUX_CPU2, 0);
    // GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_ASYNC);
    // // GPIO139 - 485-SCIA-WE-(use SCICRX as GPIO, in UART3 pin8)
    // GPIO_SetupPinMux(37, GPIO_MUX_CPU2, 0);
    // GPIO_SetupPinOptions(37, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux    (PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_MUX_CPU2, 0); // GPIO140 - 485-SCIB-WE-(use SCICTX as GPIO, in UART3 pin7)
    GPIO_SetupPinOptions(PIN_485_SCIB_WE_SCICTX_UART3pin7, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux    (PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_MUX_CPU2, 0); // GPIO139 - 485-SCIA-WE-(use SCICRX as GPIO, in UART3 pin8)
    GPIO_SetupPinOptions(PIN_485_SCIA_WE_SCICRX_UART3pin8, GPIO_OUTPUT, GPIO_ASYNC);

    // =========FOR EUREKA===========
    //        // =========TEST BOARD PIN============
    //        // =========NOT FOR EUREKA===========
    //        //GPIO30 - CANRXA
    //        GPIO_SetupPinMux(30, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_ASYNC);
    //        //GPIO31 - CANTXA
    //        GPIO_SetupPinMux(31, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    //        //GPIO28 - 485RX-SCIA
    //        GPIO_SetupPinMux(28, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    //        //GPIO29 - 485TX-SCIA
    //        GPIO_SetupPinMux(29, GPIO_MUX_CPU2, 1);
    //        GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    //        //GPIO8 - 485WE
    //        GPIO_SetupPinMux(8,GPIO_MUX_CPU2,0);
    //        GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_ASYNC);
    //        InitScicGpio();
}

void axis_basic_setup(int axisCnt){
    Axis->ID = 100 + axisCnt;
    Axis->pCTRL = CTRL;
    Axis->Pdebug = debug;

    allocate_CTRL(CTRL);
    init_experiment();
    init_experiment_AD_gain_and_offset();

    // Axis->use_first_set_three_phase = 1; // -1;
    Axis->Set_current_loop = FALSE;
    Axis->Set_x_suspension_current_loop = FALSE;
    Axis->Set_y_suspension_current_loop = FALSE;
    Axis->Set_manual_rpm = 50.0;
    Axis->Set_manual_current_iq = 0.0;
    Axis->Set_manual_current_id = 0.0; // id = -1 A is the magic number to get more torque! cjh 2024-02-29
    Axis->Select_exp_operation = 0;    // 200; //202; //200; //101;
    // Axis->pFLAG_INVERTER_NONLINEARITY_COMPENSATION = &Axis->pCTRL->g->FLAG_INVERTER_NONLINEARITY_COMPENSATION;
    Axis->flag_overwrite_theta_d = FALSE;
    Axis->Overwrite_Current_Frequency = 0;
    Axis->Overwrite_Suspension_Current_Frequency = 0.5;
    Axis->used_theta_d_elec = 0.0;
    Axis->angle_shift_for_first_inverter = ANGLE_SHIFT_FOR_FIRST_INVERTER;
    Axis->angle_shift_for_second_inverter = ANGLE_SHIFT_FOR_SECOND_INVERTER;
    Axis->OverwriteSpeedOutLimitDuringInit = 6; // 10; // A

    Axis->FLAG_ENABLE_PWM_OUTPUT = FALSE;

    Axis->channels_preset = 7; // 9; // 101;    }

    Axis->pCTRL->enc->sum_qepPosCnt = 0;
    Axis->pCTRL->enc->cursor = 0;
    Axis->pCTRL->enc->flag_absolute_encoder_powered = FALSE;
}

void init_experiment_AD_gain_and_offset()
{
    /* ADC OFFSET */
    //    // LEM1
    //    Axis->adc_offset[0] = OFFSET_VDC_BUS_IPM1;
    //    Axis->adc_offset[4] = OFFSET_LEM1_L; // b7
    //    Axis->adc_offset[5] = OFFSET_LEM1_R; // b8
    //    Axis->adc_offset[6] = OFFSET_LEM1_M; // b9
    //
    //    // LEM2
    //    Axis->adc_offset[1] = OFFSET_LEM2_L; // a1
    //    Axis->adc_offset[2] = OFFSET_LEM2_R; // a2
    //    Axis->adc_offset[3] = OFFSET_LEM2_M; // a3

    Axis->adc_scale[0] = SCALE_VDC_BUS_IPM1;
    Axis->adc_scale[1] = SCALE_LEM_A1;
    Axis->adc_scale[2] = SCALE_LEM_A2;
    Axis->adc_scale[3] = SCALE_LEM_A3;
    Axis->adc_scale[4] = SCALE_LEM_B7;
    Axis->adc_scale[5] = SCALE_LEM_B8;
    Axis->adc_scale[6] = SCALE_LEM_B9;

    Axis->adc_offset[0] = OFFSET_VDC_BUS_IPM1;
    Axis->adc_offset[1] = OFFSET_LEM_A1;
    Axis->adc_offset[2] = OFFSET_LEM_A2;
    Axis->adc_offset[3] = OFFSET_LEM_A3;
    Axis->adc_offset[4] = OFFSET_LEM_B7;
    Axis->adc_offset[5] = OFFSET_LEM_B8;
    Axis->adc_offset[6] = OFFSET_LEM_B9;

    /* two motor OFFSET */
    #if NUMBER_OF_AXES == 2
        #if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP                      /* eQEP OFFSET */
            if(axisCnt==0){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
            if(axisCnt==0){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK) || (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            if(axisCnt==0){
                Axis_1.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
            if(axisCnt==1){
                Axis_2.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            }
        #endif
    #else
        #if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP                  /* eQEP OFFSET */
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
            Axis_1.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
            // Axis_2.pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK)
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            Axis->pCTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis = HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS;
        #endif
    #endif
}


/* compute CLA task vectors */
void compute_CLA_task_vectors(){
    Cla1Regs.MVECT1 = (Uint16)((Uint32)&Cla1Task1 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT2 = (Uint16)((Uint32)&Cla1Task2 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT3 = (Uint16)((Uint32)&Cla1Task3 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT4 = (Uint16)((Uint32)&Cla1Task4 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT5 = (Uint16)((Uint32)&Cla1Task5 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT6 = (Uint16)((Uint32)&Cla1Task6 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 - (Uint32)&Cla1Prog_Start);
}


void init_spi() {
    // 初始化SPI，用于与DAC芯片MAX5307通讯。
    EALLOW;
    DevCfgRegs.CPUSEL6.bit.SPI_A = 1; // assign spi-a to cpu2
    DevCfgRegs.CPUSEL6.bit.SPI_C = 1; // assign spi-c to cpu2

    DevCfgRegs.CPUSEL5.bit.SCI_A = 1; // assign sci-a to cpu2
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1; // assign sci-b to cpu2

    DevCfgRegs.CPUSEL8.bit.CAN_A = 1; // assign can-a to cpu2
    DevCfgRegs.CPUSEL8.bit.CAN_B = 1; // assign can-b to cpu2

    // Allows CPU01 bootrom to take control of clock
    // configuration registers
    ClkCfgRegs.CLKSEM.all = 0xA5A50000;
    ClkCfgRegs.LOSPCP.all = 0x0001; // LSPCLK=100MHz table 3-173
    EDIS;

    // 同步！！！！！
    InitHighSpeedSpiGpio();
    // InitSpicGpio();
    // InitSpiaGpio();
    // InitSpi(); // this is moved to CPU02
}

void handle_interrupts() {
        /* Re-map PIE Vector Table to user defined ISR functions. */
    EALLOW;                                   // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &SYSTEM_PROGRAM; //&MainISR;      // PWM主中断 10kKHz
#if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
    PieVectTable.ECAP1_INT = &ecap1_isr;
    PieVectTable.ECAP2_INT = &ecap2_isr;
    PieVectTable.ECAP3_INT = &ecap3_isr;
#endif
#if SYSTEM_PROGRAM_MODE != 223
    PieVectTable.EQEP1_INT = &EQEP_UTO_INT; // eQEP
#endif
#if NUMBER_OF_DSP_CORES == 1
// PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;   // SCI Receive (Part 1/3)
// PieVectTable.SCIC_TX_INT = &scicTxFifoIsr;   // SCI Transmit
#endif
    EDIS; // This is needed to disable write to EALLOW protected registers
          /* PIE Control */
    /* ePWM */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // PWM1 interrupt (Interrupt 3.1)
/* SCI */
#if NUMBER_OF_DSP_CORES == 1
                                       // PieCtrlRegs.PIEIER8.bit.INTx5 = 1;   // PIE Group 8, INT5, SCI-C Rx (Part 2/3)
                                       // PieCtrlRegs.PIEIER8.bit.INTx6 = 1;   // PIE Group 8, INT6, SCI-C Tx
#endif
/* eCAP */
#if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1; // Enable eCAP INTn in the PIE: Group 3 __interrupt 1--6 (Interrupt 4.1)
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1; // 1 Enable for Interrupt 4.2
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1; // 2 Enable for Interrupt 4.3
#endif
/* eQEP */
#if SYSTEM_PROGRAM_MODE != 223
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1; // QEP interrupt
#endif
    /* CPU Interrupt Enable Register (IER) */
    IER |= M_INT3; // EPWM1_INT
#if NUMBER_OF_DSP_CORES == 1
                   // IER |= M_INT8; // SCI-C (Part 3/3)
#endif
#if USE_ECAP_CEVT2_INTERRUPT == 1 && ENABLE_ECAP
    IER |= M_INT4; // ECAP1_INT // CPU INT4 which is connected to ECAP1-4 INT
#endif
#if SYSTEM_PROGRAM_MODE != 223 // used by yuanxin
    IER |= M_INT5;             // EQEP1_INT???
#endif
    EINT; // Enable Global __interrupt INTM
    ERTM; // Enable Global realtime __interrupt DBGM
}

#if (FALSE)
    {
        SendChar = 24;
        for (;;)
        {
            // SendChar = 0;
            ScicRegs.SCITXBUF.all = (SendChar);

            //
            // wait for RRDY/RXFFST =1 for 1 data available in FIFO
            //
            while (ScicRegs.SCIFFRX.bit.RXFFST == 0)
            {
            }
            (*CTRL).s->go_sensorless = 100;

            //
            // Check received character
            //
            ReceivedChar = ScicRegs.SCIRXBUF.all;
            if (ReceivedChar != SendChar)
            {
                // error();
            }

            //
            // Move to the next character and repeat the test
            //
            SendChar++;

            //
            // Limit the character to 8-bits
            //
            SendChar &= 0x00F;
            LoopCount++;
        }
    }
#endif

void main_loop() {
    while (1){
        //        mainWhileLoopCounter1++;
        //        mainWhileLoopCounter2=2992;
        //        if (Motor_mode_START==1){
        //            Axis_1.FLAG_ENABLE_PWM_OUTPUT = 1;
        //            DSP_START_LED1
        //            DSP_START_LED2
        //            mainWhileLoopCounter1 = Axis_1.FLAG_ENABLE_PWM_OUTPUT +5;
        //        }else if (Motor_mode_START==0){
        //            Axis_1.FLAG_ENABLE_PWM_OUTPUT = 0;
        //            DSP_STOP_LED1
        //            DSP_STOP_LED2
        //            mainWhileLoopCounter1 = Axis_1.FLAG_ENABLE_PWM_OUTPUT +5;
        //        }
        //        mainWhileLoopCounter1 = mainWhileLoopCounter2;

        //        mainWhileLoopCounter3 += 1;
        //        Axis_1.ID += 1;
        //        mainWhileLoopCounter2 += 1;

        #if NUMBER_OF_DSP_CORES == 1
            single_core_dac();
        #endif
    }
}

REAL wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION = 500;

void DeadtimeCompensation(REAL Current_U, REAL Current_V, REAL Current_W, REAL CMPA[], REAL CMPA_DBC[])
{

    // TODO: Add transitional linear range to dead time compensation

    int temp = 0;
    // ------------U--------------
    if (Current_U >= 0)
    {
        temp = (int)(CMPA[0] + wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp >= SYSTEM_TBPRD)
            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    else
    {
        temp = (int)(CMPA[0] - wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp <= 0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[0] = (Uint16)temp;
    temp = 0;

    // --------------V--------------
    if (Current_V >= 0)
    {
        temp = (int)(CMPA[1] + wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp >= SYSTEM_TBPRD)
            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    else
    {
        temp = (int)(CMPA[1] - wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp <= 0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[1] = (Uint16)temp;
    temp = 0;

    // --------------W--------------
    if (Current_W >= 0)
    {
        temp = (int)(CMPA[2] + wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp >= SYSTEM_TBPRD)
            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    else
    {
        temp = (int)(CMPA[2] - wubo_debug_SYSTEM_PWM_DEADTIME_COMPENSATION);
        if (temp <= 0)
            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
    }
    CMPA_DBC[2] = (Uint16)temp;
}

//void DeadtimeCompensation(REAL Current_U, REAL Current_V, REAL Current_W, REAL CMPA[], REAL CMPA_DBC[])
//{
//
//    // TODO: Add transitional linear range to dead time compensation
//
//    int temp = 0;
//    // ------------U--------------
//    if (Current_U >= 0)
//    {
//        temp = (int)(CMPA[0] + SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp >= SYSTEM_TBPRD)
//            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    else
//    {
//        temp = (int)(CMPA[0] - SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp <= 0)
//            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    CMPA_DBC[0] = (Uint16)temp;
//    temp = 0;
//
//    // --------------V--------------
//    if (Current_V >= 0)
//    {
//        temp = (int)(CMPA[1] + SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp >= SYSTEM_TBPRD)
//            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    else
//    {
//        temp = (int)(CMPA[1] - SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp <= 0)
//            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    CMPA_DBC[1] = (Uint16)temp;
//    temp = 0;
//
//    // --------------W--------------
//    if (Current_W >= 0)
//    {
//        temp = (int)(CMPA[2] + SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp >= SYSTEM_TBPRD)
//            temp = (int)((SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    else
//    {
//        temp = (int)(CMPA[2] - SYSTEM_PWM_DEADTIME_COMPENSATION);
//        if (temp <= 0)
//            temp = (int)((-SYSTEM_PWM_UDC_UTILIZATION * 0.5 + 0.5) * SYSTEM_TBPRD);
//    }
//    CMPA_DBC[2] = (Uint16)temp;
//}

REAL vvvf_voltage = 4;
REAL vvvf_frequency = 5;
REAL enable_vvvf = FALSE;

REAL wubo_debug_USE_DEATIME_PRECOMP = 0;

void voltage_commands_to_pwm()
{
    if (axisCnt == 0){
        // SVPWM of the motor 3-phase
        (*CTRL).svgen1.Ualpha = (*CTRL).o->cmd_uAB_to_inverter[0];
        (*CTRL).svgen1.Ubeta = (*CTRL).o->cmd_uAB_to_inverter[1];

        if (enable_vvvf){
            (*CTRL).svgen1.Ualpha = vvvf_voltage * cos(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
            (*CTRL).svgen1.Ubeta  = vvvf_voltage * sin(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
        }

        SVGEN_Drive(&(*CTRL).svgen1);
        (*CTRL).svgen1.CMPA[0] = (*CTRL).svgen1.Ta * SYSTEM_TBPRD;
        (*CTRL).svgen1.CMPA[1] = (*CTRL).svgen1.Tb * SYSTEM_TBPRD;
        (*CTRL).svgen1.CMPA[2] = (*CTRL).svgen1.Tc * SYSTEM_TBPRD;
        //#if USE_DEATIME_PRECOMP
        if(wubo_debug_USE_DEATIME_PRECOMP){
            DeadtimeCompensation(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2], (*CTRL).svgen1.CMPA, (*CTRL).svgen1.CMPA_DBC);
            EPwm1Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[0];
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[1];
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA_DBC[2];
        }
        else{
            EPwm1Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[0];
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[1];
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen1.CMPA[2];
        }
    }

    if (axisCnt == 1){
        // SVPWM of the suspension 3-phase
        (*CTRL).svgen2.Ualpha = (*CTRL).o->cmd_uAB_to_inverter[0];
        (*CTRL).svgen2.Ubeta = (*CTRL).o->cmd_uAB_to_inverter[1];

        if (enable_vvvf){
            (*CTRL).svgen2.Ualpha = vvvf_voltage * cos(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
            (*CTRL).svgen2.Ubeta = vvvf_voltage * sin(vvvf_frequency * 2 * M_PI * (*CTRL).timebase);
        }
        SVGEN_Drive(&(*CTRL).svgen2); //, -(*CTRL).UNot);
        (*CTRL).svgen2.CMPA[0] = (*CTRL).svgen2.Ta * SYSTEM_TBPRD;
        (*CTRL).svgen2.CMPA[1] = (*CTRL).svgen2.Tb * SYSTEM_TBPRD;
        (*CTRL).svgen2.CMPA[2] = (*CTRL).svgen2.Tc * SYSTEM_TBPRD;
        #if USE_DEATIME_PRECOMP
            DeadtimeCompensation(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5], (*CTRL).svgen2.CMPA, (*CTRL).svgen2.CMPA_DBC);
            EPwm4Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[0];
            EPwm5Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[1];
            EPwm6Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA_DBC[2];
        #else
            EPwm4Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[0];
            EPwm5Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[1];
            EPwm6Regs.CMPA.bit.CMPA = (Uint16)(*CTRL).svgen2.CMPA[2];
        #endif
    }

    //    svgen2.Ualpha = svgen1.Ualpha*0.5        + svgen1.Ubeta*0.8660254; // rotate 60 deg
    //    svgen2.Ubeta  = svgen1.Ualpha*-0.8660254 + svgen1.Ubeta*0.5;
}

void voltage_measurement_based_on_eCAP()
{
    CAP.terminal_voltage[0] = (CAP.terminal_DutyOnRatio[0]) * Axis->vdc - Axis->vdc * 0.5; // -0.5 is due to duty ratio calculation; - vdc * 0.5 is referring to the center of dc bus capacitor.
    CAP.terminal_voltage[1] = (CAP.terminal_DutyOnRatio[1]) * Axis->vdc - Axis->vdc * 0.5;
    CAP.terminal_voltage[2] = (CAP.terminal_DutyOnRatio[2]) * Axis->vdc - Axis->vdc * 0.5;

    CAP.line_to_line_voltage[0] = CAP.terminal_voltage[0] - CAP.terminal_voltage[1];
    CAP.line_to_line_voltage[1] = CAP.terminal_voltage[1] - CAP.terminal_voltage[2];
    CAP.line_to_line_voltage[2] = CAP.terminal_voltage[2] - CAP.terminal_voltage[0];

    if (CAP.flag_bad_U_capture == FALSE && CAP.flag_bad_V_capture == FALSE && CAP.flag_bad_W_capture == FALSE)
    {
        // Use ecap feedback
        CAP.uab0[0] = 0.33333 * (2 * CAP.terminal_voltage[0] - CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[1] = 0.57735 * (CAP.terminal_voltage[1] - CAP.terminal_voltage[2]);
        CAP.uab0[2] = 0.33333 * (CAP.terminal_voltage[0] + CAP.terminal_voltage[1] + CAP.terminal_voltage[2]);
        CAP.dq[0] = (*CTRL).s->cosT * CAP.uab0[0] + (*CTRL).s->sinT * CAP.uab0[1];
        CAP.dq[1] = -(*CTRL).s->sinT * CAP.uab0[0] + (*CTRL).s->cosT * CAP.uab0[1];
    }
    else
    {
        // Assume the voltage vector is rtoating at a constant speed when ecap measurement is disturbed.
        CAP.uab0[0] = (*CTRL).s->cosT * CAP.dq[0] - (*CTRL).s->sinT * CAP.dq[1];
        CAP.uab0[1] = (*CTRL).s->sinT * CAP.dq[0] + (*CTRL).s->cosT * CAP.dq[1];
    }

    // 电压测量
    if (G.flag_use_ecap_voltage == 2 || G.flag_use_ecap_voltage == 1)
    {
        /*Use original ecap measured voltage*/
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];
    }
    //    else if(G.flag_use_ecap_voltage==3){
    //        ecap_moving_average();
    //    }
    if (G.flag_use_ecap_voltage == 10)
    {
        /*Use lpf ecap measured voltage*/
        CAP.dq_lpf[0] = _lpf(CAP.dq[0], CAP.dq_lpf[0], 800);
        CAP.dq_lpf[1] = _lpf(CAP.dq[1], CAP.dq_lpf[1], 800);
        CAP.uab0[0] = (*CTRL).s->cosT * CAP.dq_lpf[0] - (*CTRL).s->sinT * CAP.dq_lpf[1];
        CAP.uab0[1] = (*CTRL).s->sinT * CAP.dq_lpf[0] + (*CTRL).s->cosT * CAP.dq_lpf[1];
        US_P(0) = US_C(0);
        US_P(1) = US_C(1);
        US_C(0) = CAP.uab0[0];
        US_C(1) = CAP.uab0[1];
    }
    else if (G.flag_use_ecap_voltage == 0)
    {
        /*Use command voltage for feedback*/
        US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
        US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
        US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
        US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
    }

    // (for watch only) Mismatch between ecap measurement and command to inverter
    (*CTRL).o->cmd_uDQ_to_inverter[0] = (*CTRL).s->cosT * (*CTRL).o->cmd_uAB_to_inverter[0] + (*CTRL).s->sinT * (*CTRL).o->cmd_uAB_to_inverter[1];
    (*CTRL).o->cmd_uDQ_to_inverter[1] = -(*CTRL).s->sinT * (*CTRL).o->cmd_uAB_to_inverter[0] + (*CTRL).s->cosT * (*CTRL).o->cmd_uAB_to_inverter[1];
    CAP.dq_mismatch[0] = (*CTRL).o->cmd_uDQ_to_inverter[0] - CAP.dq[0];
    CAP.dq_mismatch[1] = (*CTRL).o->cmd_uDQ_to_inverter[1] - CAP.dq[1];
}


void test_ipc_tocpu02()
{
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
}

void cla_test_codes(){
    /* initialize PI controller */
    pi1.Kp = 5.5f;
    pi1.Ki = 0.015f;
    pi1.i10 = 0.0f;
    pi1.i6 = 1.0f;
    pi1.Umax = 10.2f;
    pi1.Umin = -10.2f;

    /* compute CLA task vectors */
    EALLOW;
    compute_CLA_task_vectors();

    /* CLA task triggers */
    Cla1Regs.MIER.all = 0x00FF;

    /* Switch the CLA program space to the CLA and enable software forcing
     * Also switch over CLA data ram 0,1 and 2
     * CAUTION: The RAMxCPUE bits can only be enabled by writing to the register
     * and not the individual bit field. Furthermore, the status of these bitfields
     * is not reflected in either the watch or register views - they always read as
     * zeros. This is a known bug and the user is advised to test CPU accessibilty
     * first before proceeding
     */
    Cla1Regs.MCTL.bit.IACKE = 1;
    EDIS;
}



extern REAL wubo_debug_motor_enc_dirc[2];

/* 读取编码器的位置 */
void measurement_position_count_axisCnt0(){
    #if (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK)
            position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
    #elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
            position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
    #endif

    #if NUMBER_OF_AXES == 2
        position_count_SCI_fromCPU2 = position_count_SCI_shank_fromCPU2;
    #endif
        // 正电流导致编码器读数增大：
        CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[0] * (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
}


void measurement_position_count_axisCnt1(){
    #if NUMBER_OF_AXES == 2
        position_count_SCI_fromCPU2 = position_count_SCI_hip_fromCPU2;
    #endif
        // 正电流导致编码器读数减小
        CTRL->enc->encoder_abs_cnt = wubo_debug_motor_enc_dirc[1] * ( (int32)position_count_SCI_fromCPU2 - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis );
        // dq变化中，d轴理论上指向永磁体的北极，
}

/* 编码器位置信息转换为速度信息 */
void measurement_enc(){
    if (!bool_use_SCI_encoder){
        // 正转电流导致编码器读数减小：
        //CTRL->enc->encoder_abs_cnt = -((int32)cnt_four_bar_map_motor_encoder_angle + CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis);
        // 正转电流导致编码器读数增大：

        CTRL->enc->encoder_abs_cnt = (int32)cnt_four_bar_map_motor_encoder_angle - CTRL->enc->OffsetCountBetweenIndexAndUPhaseAxis;
    }

    // 给CTRL->enc->encoder_abs_cnt_previous赋值的操作在measurement()函数中进行
    
    // ignore this please
    // if (CTRL->enc->flag_absolute_encoder_powered == FALSE)
    // {
    //     CTRL->enc->flag_absolute_encoder_powered = TRUE;
    //     // assume there motor is at still when it is powered on
    //     CTRL->enc->encoder_abs_cnt_previous = CTRL->enc->encoder_abs_cnt;
    // }

    while (CTRL->enc->encoder_abs_cnt > SYSTEM_QEP_QPOSMAX_PLUS_1){ // SYSTEM_QEP_QPOSMAX_PLUS_1为编码器的一圈的脉冲数+1
        CTRL->enc->encoder_abs_cnt -= SYSTEM_QEP_QPOSMAX_PLUS_1;
    }
    while (CTRL->enc->encoder_abs_cnt < 0){
        CTRL->enc->encoder_abs_cnt += SYSTEM_QEP_QPOSMAX_PLUS_1;
    }

    CTRL->enc->theta_d__state = CTRL->enc->encoder_abs_cnt * CNT_2_ELEC_RAD;
    while (CTRL->enc->theta_d__state > M_PI)
        CTRL->enc->theta_d__state -= 2 * M_PI;
    while (CTRL->enc->theta_d__state < -M_PI)
        CTRL->enc->theta_d__state += 2 * M_PI;
    CTRL->enc->theta_d_elec = CTRL->enc->theta_d__state;

    CTRL->enc->encoder_incremental_cnt = CTRL->enc->encoder_abs_cnt - CTRL->enc->encoder_abs_cnt_previous;
    if (CTRL->enc->encoder_incremental_cnt < -0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
        CTRL->enc->encoder_incremental_cnt += (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;
    else if (CTRL->enc->encoder_incremental_cnt > 0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
        CTRL->enc->encoder_incremental_cnt -= (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;

    // CTRL->enc->encoder_incremental_cnt * SYSTEM_QEP_REV_PER_PULSE表示在一次通讯周期内，转子转过的角度，既可以看成是转子的转速（离散系统的pointview2）
    // * 1e4 * 60转换为1分钟内的转速，即将转速单位变为RPM
    CTRL->enc->rpm_raw = CTRL->enc->encoder_incremental_cnt * SYSTEM_QEP_REV_PER_PULSE * 1e4 * 60; // 1e4指的是EPWM1_ISR中断的频率：10kHz


    //                #define ENC(X) (*Axis[X].pCTRL->enc)
    //                ENC(0)
    //                ENC(1)

    // moving average filtering
    CTRL->enc->sum_qepPosCnt -= CTRL->enc->MA_qepPosCnt[CTRL->enc->cursor];
    CTRL->enc->sum_qepPosCnt += CTRL->enc->rpm_raw;                  // CTRL->enc->encoder_incremental_cnt;
    CTRL->enc->MA_qepPosCnt[CTRL->enc->cursor] = CTRL->enc->rpm_raw; // CTRL->enc->encoder_incremental_cnt;

    CTRL->enc->cursor += 1;
    if (CTRL->enc->cursor >= MA_SEQUENCE_LENGTH){
        CTRL->enc->cursor = 0; // Reset CTRL->enc->cursor
    }

    CTRL->enc->rpm = CTRL->enc->sum_qepPosCnt * MA_SEQUENCE_LENGTH_INVERSE; // CL_TS_INVERSE;
    // CTRL->enc->rpm = CTRL->enc->rpm_raw;

    CTRL->enc->varOmega = CTRL->enc->rpm * RPM_2_MECH_RAD_PER_SEC;

    // end of axiscnt

    // 转子位置和转速接口 以及 转子位置和转速测量
    //    int32 QPOSCNT;
    //    if(ENCODER_TYPE == INCREMENTAL_ENCODER_QEP){
    //        QPOSCNT = EQep1Regs.QPOSCNT;
    //    }
    //    // 20240123 腿部电机测试
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_SCI){
    //        QPOSCNT =  - position_count_SCI_fromCPU2; // TODO: 开环电流矢量正转的时候，电机的编码器读数在减小，所以取个负号。
    //        //QPOSCNT = position_count_SCI_fromCPU2;
    //    }
    //    // 使用can_ID0x01编码器
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01){
    //        QPOSCNT = position_count_CAN_ID0x01_fromCPU2;
    //    }
    //    // 使用can_ID0x03编码器
    //    if(ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03){
    //        QPOSCNT = position_count_CAN_ID0x03_fromCPU2;
    //    }

    // CTRL->enc->rpm = PostionSpeedMeasurement_MovingAvergage(QPOSCNT, CTRL->enc);
}

void measurement_current_axisCnt0(){
    // LEM1
    Axis->iuvw[0] = ((REAL)(AdcaResultRegs.ADCRESULT1) - Axis->adc_offset[1]) * Axis->adc_scale[1]; //
    Axis->iuvw[1] = ((REAL)(AdcaResultRegs.ADCRESULT2) - Axis->adc_offset[2]) * Axis->adc_scale[2]; //
    Axis->iuvw[2] = ((REAL)(AdcaResultRegs.ADCRESULT3) - Axis->adc_offset[3]) * Axis->adc_scale[3]; //

    // 电流接口
    if (USE_3_CURRENT_SENSORS)
    {
        // Axis->iabg[0] = UVW2A_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
        // Axis->iabg[1] = UVW2B_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
        // Axis->iabg[2] = UVW2G_AI(Axis->iuvw[0], Axis->iuvw[1], Axis->iuvw[2]);
        Axis->iabg[0] = UVW2A_AI(Axis->iuvw[PIN_ADCA_U], Axis->iuvw[PIN_ADCA_V], Axis->iuvw[PIN_ADCA_W]);
        Axis->iabg[1] = UVW2B_AI(Axis->iuvw[PIN_ADCA_U], Axis->iuvw[PIN_ADCA_V], Axis->iuvw[PIN_ADCA_W]);
        Axis->iabg[2] = UVW2G_AI(Axis->iuvw[PIN_ADCA_U], Axis->iuvw[PIN_ADCA_V], Axis->iuvw[PIN_ADCA_W]);
        
    }
    else
    {
        REAL phase_V_current = -Axis->iuvw[0] - Axis->iuvw[2];
        Axis->iabg[0] = UV2A_AI(Axis->iuvw[0], phase_V_current);
        Axis->iabg[1] = UV2B_AI(Axis->iuvw[0], phase_V_current);
    }
}

void measurement_current_axisCnt1()
{
    // LEM2
    Axis->iuvw[3] = ((REAL)(AdcbResultRegs.ADCRESULT7) - Axis->adc_offset[4]) * Axis->adc_scale[4]; //
    Axis->iuvw[4] = ((REAL)(AdcbResultRegs.ADCRESULT8) - Axis->adc_offset[5]) * Axis->adc_scale[5]; //
    Axis->iuvw[5] = ((REAL)(AdcbResultRegs.ADCRESULT9) - Axis->adc_offset[6]) * Axis->adc_scale[6]; //

    // 电流接口
    if (USE_3_CURRENT_SENSORS)
    {
        // Axis->iabg[0] = UVW2A_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        // Axis->iabg[1] = UVW2B_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        // Axis->iabg[2] = UVW2G_AI(Axis->iuvw[3], Axis->iuvw[4], Axis->iuvw[5]);
        Axis->iabg[0] = UVW2A_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
        Axis->iabg[1] = UVW2B_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
        Axis->iabg[2] = UVW2G_AI(Axis->iuvw[PIN_ADCB_U], Axis->iuvw[PIN_ADCB_V], Axis->iuvw[PIN_ADCB_W]);
    }
    else
    {
        REAL phase_V_current = -Axis->iuvw[3] - Axis->iuvw[5];
        Axis->iabg[0] = UV2A_AI(Axis->iuvw[3], phase_V_current);
        Axis->iabg[1] = UV2B_AI(Axis->iuvw[3], phase_V_current);
    }
}

void control_two_motor_position()
{
    Axis->flag_overwrite_theta_d = FALSE;
    Axis->Set_current_loop = FALSE;

    if (axisCnt == 0)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
        PID_Position->Ref = hip_shank_angle_to_can(
            curycontroller.theta1 + curycontroller.theta2,
            SHANK_TYPE);
#endif
    }
    if (axisCnt == 1)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
        PID_Position->Ref = hip_shank_angle_to_can(
            curycontroller.theta1,
            HIP_TYPE);
#endif
    }
    // 位置环
    // 长弧和短弧，要选短的。
    PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
    if (PID_Position->Err > (CAN_QMAX * 0.5))
    {
        PID_Position->Err -= CAN_QMAX;
    }
    if (PID_Position->Err < -(CAN_QMAX * 0.5))
    {
        PID_Position->Err += CAN_QMAX;
    }
    if (axisCnt == 0)
    {
    #if NUMBER_OF_AXES == 2
        PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * (curycontroller.dot_theta1+curycontroller.dot_theta2) * 60 * 0.15915494309189535;
    #endif
    }
    if (axisCnt == 1)
    {
    #if NUMBER_OF_AXES == 2
        PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * curycontroller.dot_theta1* 60 * 0.15915494309189535;
    #endif
    }
    if (PID_Position->Out > PID_Position->OutLimit)
    {
        PID_Position->Out = PID_Position->OutLimit;
    }
    if (PID_Position->Out < -PID_Position->OutLimit)
    {
        PID_Position->Out = -PID_Position->OutLimit;
    }
    if (axisCnt == 1)
        Axis->Set_manual_rpm = -PID_Position->Out;
    else
        Axis->Set_manual_rpm = PID_Position->Out;
}

void control_single_motor_position()
{
    Axis->flag_overwrite_theta_d = FALSE;
    Axis->Set_current_loop = FALSE;
    // 位置环
    // 长弧和短弧，要选短的。
    if (axisCnt == 0)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
        PID_Position->Ref = target_position_cnt_shank;
#endif
    }
    if (axisCnt == 1)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
        PID_Position->Ref = target_position_cnt_hip;
#endif
    }
    PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
    if (PID_Position->Err > (CAN_QMAX * 0.5))
    {
        PID_Position->Err -= CAN_QMAX;
    }
    if (PID_Position->Err < -(CAN_QMAX * 0.5))
    {
        PID_Position->Err += CAN_QMAX;
    }
    PID_Position->Out = PID_Position->Err * PID_Position->Kp;
    if (PID_Position->Out > PID_Position->OutLimit)
    {
        PID_Position->Out = PID_Position->OutLimit;
    }
    if (PID_Position->Out < -PID_Position->OutLimit)
    {
        PID_Position->Out = -PID_Position->OutLimit;
    }
    if (axisCnt == 1)
        Axis->Set_manual_rpm = -PID_Position->Out;
    else
        Axis->Set_manual_rpm = PID_Position->Out;
}

void run_shank_loop()
{ // shank motor only
    if (axisCnt == 0)
    {
        Axis->flag_overwrite_theta_d = FALSE;

        //            Axis->Set_current_loop = TRUE;
        //            if (position_count_CAN_ID0x03_fromCPU2 > 62000)
        //            {
        //                Axis->Set_manual_current_iq = -1;
        //            }
        //            else if (position_count_CAN_ID0x03_fromCPU2 < 33000)
        //            {
        //                Axis->Set_manual_current_iq = 1;
        //            }

        Axis->Set_current_loop = FALSE;
        if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MIN)
        {
            Axis->Set_manual_rpm = -legBouncingSpeed;
        }
        else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MAX)
        {
            Axis->Set_manual_rpm = legBouncingSpeed;
        }
    }
    if (axisCnt == 1)
    {
        Axis->flag_overwrite_theta_d = FALSE;

        //            Axis->Set_current_loop = TRUE;
        //            if (position_count_CAN_ID0x03_fromCPU2 > 62000)
        //            {
        //                Axis->Set_manual_current_iq = -1;
        //            }
        //            else if (position_count_CAN_ID0x03_fromCPU2 < 33000)
        //            {
        //                Axis->Set_manual_current_iq = 1;
        //            }

        Axis->Set_current_loop = FALSE;
        if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
        {
            Axis->Set_manual_rpm = legBouncingSpeed;
        }
        else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
        {
            Axis->Set_manual_rpm = -legBouncingSpeed;
        }
    }

}

void run_hip_loop()
{ // hip motor only
#if NUMBER_OF_AXES == 2
    if (axisCnt == 1)
    {

        Axis->flag_overwrite_theta_d = FALSE;

        if (bool_TEMP == TRUE)
        {
            Axis->flag_overwrite_theta_d = FALSE;
            Axis->Set_current_loop = TRUE;
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
            {
                Axis->Set_manual_current_iq = hipBouncingIq;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
            {
                Axis->Set_manual_current_iq = -hipBouncingIq;
            }
        }
        else
        {
            Axis->flag_overwrite_theta_d = FALSE;
            Axis->Set_current_loop = FALSE;
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
            {
                Axis->Set_manual_rpm = -legBouncingSpeed;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
            {
                Axis->Set_manual_rpm = legBouncingSpeed;
            }
        }

        //
        //                Axis_2.flag_overwrite_theta_d = TRUE;
        //                Axis_2.Set_current_loop = TRUE;
        //                Axis_2.Set_manual_current_iq = 5;
        //                if (position_count_CAN_ID0x01_fromCPU2 > 58000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = 10;
        //                }
        //                else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = -10;
        //                }
    }
#else
    Axis->flag_overwrite_theta_d = TRUE;
    Axis->Set_current_loop = TRUE;
    Axis->Set_manual_current_iq = 5;
    if (position_count_CAN_ID0x01_fromCPU2 > 58000)
    {
        Axis->Overwrite_Current_Frequency = hipBouncingFreq;
    }
    else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
    {
        Axis->Overwrite_Current_Frequency = -hipBouncingFreq;
    }
#endif
}

void run_both_loop()
{
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0)
    {
        // CAN03越大，小腿越伸出，legBouncingIq, rpm为正数，小腿伸出
        // CAN03越小，小腿越收起，legBouncingIq, rpm为负数，小腿收起
        Axis_1.flag_overwrite_theta_d = FALSE;
        Axis_1.Set_current_loop = TRUE;
        if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MAX)
        {
            Axis_1.Set_manual_current_iq = legBouncingIq; // CAN03太大，小腿伸出过头，需要iq为负数，收起小腿
        }
        else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MIN)
        {
            Axis_1.Set_manual_current_iq = -legBouncingIq;
        }
    }
    // 大腿电机
    if (axisCnt == 1)
    {
        // CAN01越小,大腿越抬起,hipBouncingIq为正数，大腿抬起
        // CAN01越大,大腿越放下,hipBouncingIq为负数，大腿放下
        Axis_2.flag_overwrite_theta_d = FALSE;
        Axis_2.Set_current_loop = TRUE;
        if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MAX)
        {
            Axis_2.Set_manual_current_iq = hipBouncingIq; // CAN01太大,大腿放下过多,需要iq为正数,抬起大腿
        }
        else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MIN)
        {
            Axis_2.Set_manual_current_iq = -hipBouncingIq;
        }
    }
#endif
}

void run_impedance_control()
{
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0)
    {
        Axis_1.flag_overwrite_theta_d = FALSE;
        Axis_1.Set_current_loop = TRUE;
        Axis_1.Set_manual_current_iq = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
    if (axisCnt == 1)
    {
        Axis_2.flag_overwrite_theta_d = FALSE;
        Axis_2.Set_current_loop = TRUE;
        Axis_2.Set_manual_current_iq = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
#endif
}

// this offset is moved to ACMconfig.h
// #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19

#define NO_POSITION_CONTROL 0
#define TWOMOTOR_POSITION_CONTROL 1
#define SINGLE_POSITION_CONTROL 2
#define SHANK_LOOP_RUN 3
#define HIP_LOOP_RUN 4
#define BOTH_LOOP_RUN 5
#define IMPEDANCE_CONTROL 6

// moved to global variable [debug]
//int positionLoopType = 0;           // TWOMOTOR_POSITION_CONTROL; //SHANK_LOOP_RUN; // SHANK_LOOP_RUN; //BOTH_LOOP_RUN;
//int use_first_set_three_phase = 1; //-1 for both motors

REAL call_position_loop_controller(int positionLoopType)
{
    CTRL_1.s->Speed->Kp = TEST_SHANK_SPD_KP;
    CTRL_1.s->Speed->Ki_CODE = TEST_SHANK_SPD_KI;

    CTRL_2.s->Speed->Kp = TEST_HIP_SPD_KP;
    CTRL_2.s->Speed->Ki_CODE = TEST_HIP_SPD_KI;


    CTRL_1.s->Position->Kp = TEST_SHANK_KP;
    CTRL_2.s->Position->Kp = TEST_HIP_KP;

    CTRL_1.s->Position->OutLimit = TEST_SHANK_POS_OUTLIMIT;
    CTRL_2.s->Position->OutLimit = TEST_HIP_POS_OUTLIMIT;

    run_iecon_main((*CTRL).timebase_counter);

    if (positionLoopType == TWOMOTOR_POSITION_CONTROL) control_two_motor_position();

    else if (positionLoopType == SINGLE_POSITION_CONTROL) control_single_motor_position();

    else if (positionLoopType == SHANK_LOOP_RUN) run_shank_loop();

    else if (positionLoopType == HIP_LOOP_RUN) run_hip_loop();

    else if (positionLoopType == BOTH_LOOP_RUN) run_both_loop();

    else if (positionLoopType == IMPEDANCE_CONTROL) run_impedance_control();


    return Axis->Set_manual_rpm;
}

void DISABLE_PWM_OUTPUT(int use_first_set_three_phase)
{
    DSP_PWM_DISABLE
    DSP_2PWM_DISABLE

    /* Only init once for easy debug */
    /* wubo：每次执行ENABLE PWM OUT时候会将下面的flag置为0 从而每次关闭开启开关都会执行下面的操作！上面说的有问题？*/
    if (!G.flag_experimental_initialized)
    {
        G.flag_experimental_initialized = TRUE;

        init_experiment();
        // init_experiment_AD_gain_and_offset();
        // init_experiment_overwrite();

        // TODO: use a function for this purpose!
        // 清空积分缓存
        PID_Speed->OutPrev = 0;
        PID_iD->OutPrev = 0;
        PID_iQ->OutPrev = 0;

        // 清空速度InnerLoop缓存
        PID_Speed->KFB_Term_Prev = 0;
        PID_Speed->KFB_Term = 0;
        // PID_iX->OutPrev = 0;
        // PID_iy->OutPrev = 0;

        EPwm1Regs.CMPA.bit.CMPA = 2500;
        EPwm2Regs.CMPA.bit.CMPA = 2500;
        EPwm3Regs.CMPA.bit.CMPA = 2500;
        EPwm4Regs.CMPA.bit.CMPA = 2500;
        EPwm5Regs.CMPA.bit.CMPA = 2500;
        EPwm6Regs.CMPA.bit.CMPA = 2500;

        //            CTRL_2.s->iD  = &_pid_iD_2;
        //            CTRL_2.s->iQ  = &_pid_iQ_2;
        //            CTRL_2.s->spd = &_pid_spd_2;
        //            CTRL_2.s->pos = &_pid_pos_2;

        if ((*CTRL).g->overwrite_vdc < 5)
        {
            (*CTRL).g->overwrite_vdc = 28;
        }
        (*CTRL).g->flag_overwite_vdc = 0;

        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        #if WHO_IS_USER == USER_WB
            *CTRL).timebase_counter = 0;   
            *CTRL).timebase = 0.0;
            *debug).CMD_CURRENT_SINE_AMPERE                              = d_sim.user.CMD_CURRENT_SINE_AMPERE;
            *debug).CMD_SPEED_SINE_RPM                                   = d_sim.user.CMD_SPEED_SINE_RPM;
            *debug).CMD_SPEED_SINE_HZ                                    = d_sim.user.CMD_SPEED_SINE_HZ;
            *debug).CMD_SPEED_SINE_STEP_SIZE                             = d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
            *debug).CMD_SPEED_SINE_LAST_END_TIME                         = d_sim.user.CMD_SPEED_SINE_LAST_END_TIME;
            *debug).CMD_SPEED_SINE_END_TIME                              = d_sim.user.CMD_SPEED_SINE_END_TIME;
            *debug).CMD_SPEED_SINE_HZ_CEILING                            = d_sim.user.CMD_SPEED_SINE_HZ_CEILING;
        #endif
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
    }

    /* 在不输出PWM波形的时候，也就是“开关”为OFF的时候，更新SpeedInnerLoop的参数*/
    #if WHO_IS_USER == USER_WB
        if ((*debug).who_is_user == USER_WB && (*debug).bool_apply_WC_tunner_for_speed_loop == TRUE){
        _user_wubo_WC_Tuner_Online(); // 和_user_wubo_WC_Tuner函数区别
        _user_wubo_TI_Tuner_Online();
    #else
        //TODO: Here needs a online tuning function for public use
    #endif

    DELAY_US(5);
    GpioDataRegs.GPDCLEAR.bit.GPIO106 = 1; // TODO: What is this doing?
}

REAL RPM_wave_conunter = 0;
REAL RPM_wave = 0;
REAL flag_RPM_wave = 0;

void ENABLE_PWM_OUTPUT(int positionLoopType, int use_first_set_three_phase)
{
    G.flag_experimental_initialized = FALSE;

    if (use_first_set_three_phase == 1){
        DSP_PWM_ENABLE
        // 第一套逆变器工作的时候，确保第二套逆变器关闭
        EPwm4Regs.CMPA.bit.CMPA = 2500;
        EPwm5Regs.CMPA.bit.CMPA = 2500;
        EPwm6Regs.CMPA.bit.CMPA = 2500;

    } else if (use_first_set_three_phase == 2){
        DSP_2PWM_ENABLE
        // 同上
        EPwm1Regs.CMPA.bit.CMPA = 2500;
        EPwm2Regs.CMPA.bit.CMPA = 2500;
        EPwm3Regs.CMPA.bit.CMPA = 2500;
    } 
    else if (use_first_set_three_phase == -1){
        DSP_PWM_ENABLE
        DSP_2PWM_ENABLE
    }
    #if WHO_IS_USER == USER_YZZ
        if (FE.htz.u_offset[0] > 0.1){
            FE.htz.u_offset[0] = 0;
        }
    #endif
    // DSP中控制器的时间
    (*CTRL).timebase_counter += 1;
    (*CTRL).timebase = CL_TS * (*CTRL).timebase_counter; //(*CTRL).timebase += CL_TS; // 2048 = float/REAL max

    // 根据指令，产生控制输出（电压）
    #if ENABLE_COMMISSIONING == FALSE
        // 根据Axis自己的debug切换相应的operation mode
        //* 现在Axis中有许多一样的参数，需要化简
        Axis->Select_exp_operation = (*debug).Select_exp_operation;
        //(*CTRL).s->Motor_or_Gnerator = sign((*CTRL).i->cmd_iDQ[1]) == sign(CTRL->enc->rpm); // sign((*CTRL).i->cmd_iDQ[1]) != sign((*CTRL).i->cmd_speed_rpm))
        runtime_command_and_tuning(Axis->Select_exp_operation);
        // 0x03 is shank
        //    position_count_CAN_fromCPU2 = position_count_CAN_ID0x03_fromCPU2;
        // 0x01 is hip
        // position_count_CAN_fromCPU2 = position_count_CAN_ID0x01_fromCPU2;

        if (positionLoopType == 0){
            // do nothing
        }
        else{
            // do position loop
            Axis->Set_manual_rpm = call_position_loop_controller(positionLoopType);
        }

        if (flag_RPM_wave == 1){
            Axis->Set_manual_rpm = (*CTRL).timebase * 20;
            if ( (*CTRL).timebase * 20 > 400){
                Axis->Set_manual_rpm = 400;
            }
        }

        main_switch((*debug).mode_select);

    #else
        commissioning(); // 参数辨识用
    #endif

    //(*CTRL).o->cmd_uAB_to_inverter[0]

    // operation mode为5的时候，执行下面测试逆变器输出电压的代码
    if (Axis->Select_exp_operation == XCUBE_TaTbTc_DEBUG_MODE){
        if (axisCnt == 0){
            EPwm1Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Ta * 50000000 * CL_TS; // 0-5000，5000表示0%的占空比
            EPwm2Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Tb * 50000000 * CL_TS;
            EPwm3Regs.CMPA.bit.CMPA = (*CTRL).svgen1.Tc * 50000000 * CL_TS;
        }
        if (axisCnt == 1){
            EPwm4Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Ta * 50000000 * CL_TS;
            EPwm5Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Tb * 50000000 * CL_TS;
            EPwm6Regs.CMPA.bit.CMPA = (*CTRL).svgen2.Tc * 50000000 * CL_TS;
        }
    }
    else // 否则根据上面的控制率controller()由voltage_commands_to_pwm()计算出的电压，输出到逆变器
        voltage_commands_to_pwm();
}

void read_count_from_cpu02_dsp_cores_2()
{
#if NUMBER_OF_DSP_CORES == 2
    /* CPU02 (Remote) to CPU01 (Local)
     * The register to check is IPCSTS.
     * */
    counter_missing_position_measurement += 1;
    if (IPCRtoLFlagBusy(IPC_FLAG10) == 1) // if flag
    {
        max_counter_missing_position_measurement = counter_missing_position_measurement;
        counter_missing_position_measurement = 0;
        position_count_SCI_shank_fromCPU2 = Read.SCI_shank_position_count;
        position_count_SCI_hip_fromCPU2 = Read.SCI_hip_position_count;
        //            position_count_CAN_ID0x01_fromCPU2 = Read.CAN_position_count_ID0x01;
        //            position_count_CAN_ID0x03_fromCPU2 = Read.CAN_position_count_ID0x03;
        IPCRtoLFlagAcknowledge(IPC_FLAG10);

        // CAN encoder convert to motor built-in encoder
        deg_four_bar_map_motor_encoder_angle = get_motorpos(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125); // 1/131072.0*360.0
        // deg_four_bar_map_motor_encoder_angle = lookup(position_count_CAN_ID0x03_fromCPU2 * 0.00274658203125, &ZJL_table);
        rad_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 0.017453292519943295;
        cnt_four_bar_map_motor_encoder_angle = deg_four_bar_map_motor_encoder_angle * 23301.68888888889;

// 这段放需要测时间的代码后面，观察CpuTimer_Delta的取值，代表经过了多少个 1/200e6 秒。
#if PC_SIMULATION == FALSE
        CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
        CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
// EALLOW;
// CpuTimer1.RegsAddr->TCR.bit.TSS = 1; // stop (not needed because of the line TRB=1)
// EDIS;
#endif

// 这段放需要测时间的代码前面
#if PC_SIMULATION == FALSE
        EALLOW;
        CpuTimer1.RegsAddr->TCR.bit.TRB = 1;           // reset cpu timer to period value
        CpuTimer1.RegsAddr->TCR.bit.TSS = 0;           // start/restart
        CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
        EDIS;
#endif
    }
    else
    {
        CPU2_commu_error_counter++;
    }

    if (IPCRtoLFlagBusy(IPC_FLAG11) == 1) // if flag
    {
        max_counter_missing_position_measurement = counter_missing_position_measurement;
        counter_missing_position_measurement = 0;
        position_count_CAN_ID0x01_fromCPU2 = Read.CAN_position_count_ID0x01;
        position_count_CAN_ID0x03_fromCPU2 = Read.CAN_position_count_ID0x03;
        IPCRtoLFlagAcknowledge(IPC_FLAG11);
    }
#endif
}
