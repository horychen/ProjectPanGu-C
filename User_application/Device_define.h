#ifndef DEVICE_DEFINE_H
#define DEVICE_DEFINE_H
#ifdef _PROJECT_FORMULA_GROUP // mmlab drive version 4 （陈艺铭等）

    // Abs encoder comm. via 485 tamagawa protocol
    #define PIN_485_SCIB_WE_SCICTX_UART3pin7 31
    #define PIN_485_SCIA_WE_SCICRX_UART3pin8 37
    //SCI pin config

    #define PIN_SCI_TXDA 135
    #define PIN_SCI_RXDA 136
    #define MUX_SCI_TXDA 6
    #define MUX_SCI_RXDA 6

    #define PIN_SCI_TXDB 137
    #define PIN_SCI_RXDB 138
    #define MUX_SCI_TXDB 6
    #define MUX_SCI_RXDB 6

    #define PIN_SCI_TXDC 38
    #define PIN_SCI_RXDC 39
    #define MUX_SCI_TXDC 5
    #define MUX_SCI_RXDC 5

    //ADC UVW to PIN config
    #define PIN_ADCA_U 2 // Pay attention here! Swapped
    #define PIN_ADCA_V 1
    #define PIN_ADCA_W 0

    #define PIN_ADCB_U 3
    #define PIN_ADCB_V 4
    #define PIN_ADCB_W 5

    // ADC
    #define OFFSET_VDC_BUS_IPM1 8
    #define USER_OFFSET_LEM_B7 2034 // 2024-09-17
    #define USER_OFFSET_LEM_B8 2069 // 2024-09-02
    #define USER_OFFSET_LEM_B9 2058 // 2024-09-02
    #define USER_OFFSET_LEM_A1 2025 // 2024-09-17
    #define USER_OFFSET_LEM_A2 2060 // 2024-09-17
    #define USER_OFFSET_LEM_A3 2054 // 2024-09-17

    #define SCALE_VDC_BUS_IPM1 0.140625 // ???
    #define USER_SCALE_LEM_B7 0.03076297
    #define USER_SCALE_LEM_B8 0.03038256
    #define USER_SCALE_LEM_B9 0.03039058
    #define USER_SCALE_LEM_A1 0.0305
    #define USER_SCALE_LEM_A2 0.030334
    #define USER_SCALE_LEM_A3 0.02983
#endif

#ifdef _MOTOR_GROUP // mmlab drive version 2 （吴波、严政章、杨子恺等）

    // Basic Setup for Load Sweeping Board
    #define BOOL_LOAD_SWEEPING_ON FALSE

    // Abs encoder comm. via 485 tamagawa protocol
    #define PIN_485_SCIB_WE_SCICTX_UART3pin7 31
    #define PIN_485_SCIA_WE_SCICRX_UART3pin8 37

    //SCI pin config

    #define PIN_SCI_TXDA 135
    #define PIN_SCI_RXDA 136
    #define MUX_SCI_TXDA 6
    #define MUX_SCI_RXDA 6

    #define PIN_SCI_TXDB 137
    #define PIN_SCI_RXDB 138
    #define MUX_SCI_TXDB 6
    #define MUX_SCI_RXDB 6

    #define PIN_SCI_TXDC 38
    #define PIN_SCI_RXDC 39
    #define MUX_SCI_TXDC 5
    #define MUX_SCI_RXDC 5

    // DC BUS
    #if BOOL_LOAD_SWEEPING_ON == FALSE
        #define USER_INVERTER1_SCALE_VDC_BUS_IPM 0.17943925233644858
        #define USER_INVERTER2_SCALE_VDC_BUS_IPM 0.17943925233644858
        #define USER_INVERTER3_SCALE_VDC_BUS_IPM 0.17943925233644858
        #define USER_INVERTER4_SCALE_VDC_BUS_IPM 0.17943925233644858
        #define USER_INVERTER1_OFFSET_VDC_BUS_IPM 8
        #define USER_INVERTER2_OFFSET_VDC_BUS_IPM 8
        #define USER_INVERTER3_OFFSET_VDC_BUS_IPM 8
        #define USER_INVERTER4_OFFSET_VDC_BUS_IPM 8
    #else if BOOL_LOAD_SWEEPING_ON
        #define USER_INVERTER1_SCALE_VDC_BUS_IPM 0.15384615
        #define USER_INVERTER2_SCALE_VDC_BUS_IPM 0
        #define USER_INVERTER3_SCALE_VDC_BUS_IPM 0
        #define USER_INVERTER4_SCALE_VDC_BUS_IPM 0
        #define USER_INVERTER1_OFFSET_VDC_BUS_IPM 8
        #define USER_INVERTER2_OFFSET_VDC_BUS_IPM 0
        #define USER_INVERTER3_OFFSET_VDC_BUS_IPM 0
        #define USER_INVERTER4_OFFSET_VDC_BUS_IPM 0
    #endif

    //ADC UVW to PIN config
    #define PIN_ADCA_U 0
    #define PIN_ADCA_V 1
    #define PIN_ADCA_W 2

    #define PIN_ADCB_U 3
    #define PIN_ADCB_V 4
    #define PIN_ADCB_W 5

    // Lem 2的三个蓝色块块分别是adc a1 a2 a3
    // In fact A is the first inverter in MOTOR_GROUP
    #if BOOL_LOAD_SWEEPING_ON == FALSE
        #define USER_OFFSET_LEM_A1 2038 // 2035.0 // WuBo tuned in 20241027 //2010  // 2034  // 2029.57894737 // ADCA1
        #define USER_OFFSET_LEM_A2 2050 // 2047.0 // WuBo tuned in 20241027 //2038   // 2049  // 2043.08771930 // ADCA2
        #define USER_OFFSET_LEM_A3 2057 // 2057.0 // WuBo tuned in 20241027 //2029   // 2050  // 2042.98245614 // ADCA3
        #define USER_SCALE_LEM_A1 0.0305   // 0.03080704 // ADCA1
        #define USER_SCALE_LEM_A2 0.030334 // 0.03060669 // ADCA2
        #define USER_SCALE_LEM_A3 0.031633  // 0.03045988 // ADCA3
    #else if BOOL_LOAD_SWEEPING_ON
        #define USER_OFFSET_LEM_A1 2035.0 // WuBo tuned in 20241117 //2010  // 2034  // 2029.57894737 // ADCA1
        #define USER_OFFSET_LEM_A2 2045.0 // WuBo tuned in 20241117 //2038   // 2049  // 2043.08771930 // ADCA2
        #define USER_OFFSET_LEM_A3 2040.0 // WuBo tuned in 20241117 //2029   // 2050  // 2042.98245614 // ADCA3
        #define USER_SCALE_LEM_A1 0.030769   // 0.03080704 // ADCA1
        #define USER_SCALE_LEM_A2 0.029947 // 0.03060669 // ADCA2
        #define USER_SCALE_LEM_A3 0.030761  // 0.03045988 // ADCA3
    #endif

    // Lem 1的三个蓝色块块分别是adc b7 b8 b9 // 令逆变器输出端指向电机为正方向，若LEM上的箭头与正方向相同，则SCALE为正数，若LEM上的箭头与正方向相反，则SCALE为负数，
    // B is the second inverter in MOTOR_GROUP
    #define USER_OFFSET_LEM_B7 2020 // 2023.89473684 // ADCB7
    #define USER_OFFSET_LEM_B8 2029 // 2042.33333333 // ADCB8
    #define USER_OFFSET_LEM_B9 2038 // 2043.43859649 // ADCB9
    #define USER_SCALE_LEM_B7 0.03076297 // ADCB7
    #define USER_SCALE_LEM_B8 0.03038256 // ADCB8
    #define USER_SCALE_LEM_B9 0.03039058 // ADCB9


#endif
#ifdef _INDUCTION_MOTOR_GROUP // mmlab drive version 3 （王千等）

    // Abs encoder comm. via 485 tamagawa protocol
    #define PIN_485_SCIB_WE_SCICTX_UART3pin7 31
    #define PIN_485_SCIA_WE_SCICRX_UART3pin8 37
    //SCI pin config

    #define PIN_SCI_TXDA 135
    #define PIN_SCI_RXDA 136
    #define MUX_SCI_TXDA 6
    #define MUX_SCI_RXDA 6

    #define PIN_SCI_TXDB 137
    #define PIN_SCI_RXDB 138
    #define MUX_SCI_TXDB 6
    #define MUX_SCI_RXDB 6

    #define PIN_SCI_TXDC 38
    #define PIN_SCI_RXDC 39
    #define MUX_SCI_TXDC 5
    #define MUX_SCI_RXDC 5

    //ADC UVW to PIN config
    #define PIN_ADCA_U 0
    #define PIN_ADCA_V 1
    #define PIN_ADCA_W 2
    
    #define PIN_ADCB_U 3
    #define PIN_ADCB_V 4
    #define PIN_ADCB_W 5

    // ADC
    #define OFFSET_VDC_BUS_IPM1 8
    #define USER_OFFSET_LEM_B7 2005 // 2024-09-02
    #define USER_OFFSET_LEM_B8 2038 // 2024-09-02
    #define USER_OFFSET_LEM_B9 2043 // 2024-09-02
    #define USER_OFFSET_LEM_A1 2008 // 2024-10-05 cjh tuned
    #define USER_OFFSET_LEM_A2 2036 // 2024-10-05 cjh tuned
    #define USER_OFFSET_LEM_A3 2028 // 2024-10-05 cjh tuned

    #define SCALE_VDC_BUS_IPM1 0.169014 // ???
    #define USER_SCALE_LEM_B7 0.03076297
    #define USER_SCALE_LEM_B8 0.03038256
    #define USER_SCALE_LEM_B9 0.03039058
    #define USER_SCALE_LEM_A1 0.0305
    #define USER_SCALE_LEM_A2 0.030334
    #define USER_SCALE_LEM_A3 0.02983

    // Sensor Coil
    #define OFFSET_PLACE_RIGHT    4461341  
    #define OFFSET_PLACE_DOWN     4639271  

    #define OFFSET_PLACE_LEFT     4561234  
    #define OFFSET_PLACE_UP       4513212  

    #define SCALE_PLACE_X         4.2894311E-5
    #define SCALE_PLACE_Y         4.3412197E-5

#endif

#ifdef _LEG_GROUP // _LEG_GROUP // mmlab drive version 1 (陈艺铭、朱俊磊、胡瑀等)

    // Abs encoder comm. via 485 tamagawa protocol
    #define PIN_485_SCIB_WE_SCICTX_UART3pin7 140
    #define PIN_485_SCIA_WE_SCICRX_UART3pin8 139

    //SCI pin config

    #define PIN_SCI_TXDA 135
    #define PIN_SCI_RXDA 136
    #define MUX_SCI_TXDA 6
    #define MUX_SCI_RXDA 6

    #define PIN_SCI_TXDB 137
    #define PIN_SCI_RXDB 138
    #define MUX_SCI_TXDB 6
    #define MUX_SCI_RXDB 6

    #define PIN_SCI_TXDC 38
    #define PIN_SCI_RXDC 39
    #define MUX_SCI_TXDC 5
    #define MUX_SCI_RXDC 5


    // DC BUS
    #define OFFSET_VDC_BUS_IPM1 8 // -1.01456189
    #define SCALE_VDC_BUS_IPM1 0.183916 // 0.17604031


    //ADC UVW to PIN config
    #define PIN_ADCA_U 0
    #define PIN_ADCA_V 1
    #define PIN_ADCA_W 2
    
    #define PIN_ADCB_U 3
    #define PIN_ADCB_V 4
    #define PIN_ADCB_W 5

    // Lem 1的三个蓝色块块分别是adc b7 b8 b9 // 令逆变器输出端指向电机为正方向，若LEM上的箭头与正方向相同，则SCALE为正数，若LEM上的箭头与正方向相反，则SCALE为负数，
    #define USER_OFFSET_LEM_B7 2025 // 2023.89473684 // ADCB7
    #define USER_OFFSET_LEM_B8 2041 // 2042.33333333 // ADCB8
    #define USER_OFFSET_LEM_B9 2045 // 2043.43859649 // ADCB9
    #define USER_SCALE_LEM_B7 0.03076297 // ADCB7
    #define USER_SCALE_LEM_B8 0.03038256 // ADCB8
    #define USER_SCALE_LEM_B9 0.03039058 // ADCB9

    // Lem 2的三个蓝色块块分别是adc a1 a2 a3
    #define USER_OFFSET_LEM_A1 2030      // 2029.57894737 // ADCA1
    #define USER_OFFSET_LEM_A2 2043      // 2043.08771930 // ADCA2
    #define USER_OFFSET_LEM_A3 2042      // 2042.98245614 // ADCA3
    #define USER_SCALE_LEM_A1 0.03080704 // ADCA1
    #define USER_SCALE_LEM_A2 0.03060669 // ADCA2
    #define USER_SCALE_LEM_A3 0.03045988 // ADCA3

    // GongWang Encoder

#endif


//TODO: 下面的内容需要合理分配到上面不同的group中去
/* Encoder QEP TODO: should read from excel */
#define ABSOLUTE_ENCODER_SCI_SHANK 1
#define ABSOLUTE_ENCODER_SCI_HIP  2
#define ABSOLUTE_ENCODER_CAN_ID0x01 3
#define ABSOLUTE_ENCODER_CAN_ID0x03 4
#define RESOLVER_1 5
#define RESOLVER_2 6
#define ABSOLUTE_ENCODER_MD1 7
#define INCREMENTAL_ENCODER_QEP 8


//#define ENCODER_TYPE INCREMENTAL_ENCODER_QEP
//#define ENCODER_TYPE ABSOLUTE_ENCODER_MD1 // ABSOLUTE_ENCODER_SCI_SHANK

#ifdef _MOTOR_GROUP
    #define ENCODER_TYPE ABSOLUTE_ENCODER_MD1
//    #define ENCODER_TYPE INCREMENTAL_ENCODER_QEP
//    #define ENCODER_TYPE ABSOLUTE_ENCODER_SCI_A // sci-A HIP
//    #define ENCODER_TYPE ABSOLUTE_ENCODER_SCI_B // sci-B SHANK
#endif

#ifdef _LEG_GROUP
    #define ENCODER_TYPE ABSOLUTE_ENCODER_SCI_HIP   // sci-A HIP
    #define ENCODER_TYPE ABSOLUTE_ENCODER_SCI_SHANK // sci-B SHANK
#endif



#ifdef _INDUCTION_MOTOR_GROUP
    #define ENCODER_TYPE INCREMENTAL_ENCODER_QEP
#endif

#ifdef _PROJECT_FORMULA_GROUP /* Careful! Formula groups' board is on the PMSM SD80 since 20240919 hence it takes incremental enc */
    #define ENCODER_TYPE INCREMENTAL_ENCODER_QEP
#endif

#define INIT_NPP d_sim.init.npp

#if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
    #define SYSTEM_QEP_PULSES_PER_REV (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
    #define SYSTEM_QEP_QPOSMAX (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (10000)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2358 // wb tuned with id=3A 20241027 // 2353 // cjh tuned with id_cmd = 4A @2024-10-05 // This value need an average value due to XXX(CJH told me one day but here space is not enough to print it out)
    #define positive_current_QPOSCNT_counting_down (-1) // 正向旋转的电流导致增量式编码器QEP读数减少 则填 -1，否则默认为 1。


#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_A) || (ENCODER_TYPE ==ABSOLUTE_ENCODER_SCI_B)
    // F130-16-KV20 for TEST
    #define SYSTEM_QEP_PULSES_PER_REV (8388608)
    #define SYSTEM_QEP_REV_PER_PULSE (1.1920929e-7)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define ABS_ENC_SCI_A__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS   4106211 // WB tunned with id_cmd = 3A in 20250418
    #define ABS_ENC_SCI_B__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS   340755  // wait for a value

    #define USER_MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 4106211
    #define USER_MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 340755
    #define USER_MOTOR3_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 0
    #define USER_MOTOR4_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 0

#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_SHANK) || (ENCODER_TYPE == ABSOLUTE_ENCODER_SCI_HIP)
    // F130-16-KV20
    #define SYSTEM_QEP_PULSES_PER_REV (8388608)
    #define SYSTEM_QEP_REV_PER_PULSE (1.1920929e-7)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define SHANK__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 7365433 // WB tunned with id_cmd = 3A in 20250414 // 340755 1051014 // 3494662 // ym tuned with id_cmd = 3A 2024-03-12
    #define HIP__OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS   340755  //    ym tuned with id_cmd = 3A 2024-03-12

#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01)
    #define SYSTEM_QEP_PULSES_PER_REV (131072)
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 49476 // ym tuned with
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03)
    #define SYSTEM_QEP_PULSES_PER_REV (131072)
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 51203 // ym tuned with id_cmd = 4A 2024-02-27
#elif ENCODER_TYPE == RESOLVER_1
    #define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
    #define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
    #define SYSTEM_QEP_QPOSMAX (65535)                  // (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (65536)
    #define SYSTEM_QEP_PULSES_PER_REV (65536 * RESOLVER_NUMBER_OF_POLE_PAIRS)                     // (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (1.52587890625e-05 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
#elif ENCODER_TYPE == RESOLVER_2
    #define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
    #define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
    #define SYSTEM_QEP_QPOSMAX (4095)                   // (9999)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (4096)
    #define SYSTEM_QEP_PULSES_PER_REV (4096 * RESOLVER_NUMBER_OF_POLE_PAIRS)                   // (10000)
    #define SYSTEM_QEP_REV_PER_PULSE (0.000244140625 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
#elif ENCODER_TYPE == ABSOLUTE_ENCODER_MD1
    #define SYSTEM_QEP_PULSES_PER_REV (131072) // 2^17
    #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6) // 1 / 2^17
    #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * INIT_NPP)
    #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
    #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
    #define MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 110288
    #define MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 5151

    #define USER_MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 110288
    #define USER_MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 110288
    #define USER_MOTOR3_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 0
    #define USER_MOTOR4_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 0
    // 30144 wb tuned with id_cmd = 2A, 20240715
    // MOTOR1 30190 wb tuned with id_cmd = 3A, 20240719
    // MOTOR2 41668 wb tuned with id_cmd = 3A, 20240719

    // MOTOR1 110228 wb tuned with id_cmd = 3A, 20240902
    // MOTOR2 5151 wb tuned with id_cmd = 3A, 20240902
#endif



#endif
