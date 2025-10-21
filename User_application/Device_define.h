#ifndef DEVICE_DEFINE_H
#define DEVICE_DEFINE_H
#ifdef _PROJECT_FORMULA_GROUP // mmlab drive version 4 ï¼ˆé™ˆè‰ºé“­ç­‰ï¼‰

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
    #define OFFSET_LEM_B7 2034 // 2024-09-17
    #define OFFSET_LEM_B8 2069 // 2024-09-02
    #define OFFSET_LEM_B9 2058 // 2024-09-02
    #define OFFSET_LEM_A1 2025 // 2024-09-17
    #define OFFSET_LEM_A2 2060 // 2024-09-17
    #define OFFSET_LEM_A3 2054 // 2024-09-17

    #define SCALE_VDC_BUS_IPM1 0.140625 // ???
    #define SCALE_LEM_B7 0.03076297
    #define SCALE_LEM_B8 0.03038256
    #define SCALE_LEM_B9 0.03039058
    #define SCALE_LEM_A1 0.0305
    #define SCALE_LEM_A2 0.030334
    #define SCALE_LEM_A3 0.02983
#endif

#ifdef _MOTOR_GROUP // mmlab drive version 2 ï¼ˆå�´æ³¢ã€�ä¸¥æ”¿ç« ã€�æ�¨å­�æ�ºç­‰ï¼‰

    // Basic Setup for Load Sweeping Board
    /*
    * BOOL_LOAD_SWEEPING_ON for Bezier Exp
    * BOOL_TELEOPERARION_WITH_FORMULA_BOARD for Huang Zhenzheng's EE275 Project, aka Bilateral Teleopration
    */
    #define BOOL_LOAD_SWEEPING_ON FALSE
    #define BOOL_TELEOPERARION_WITH_FORMULA_BOARD FALSE

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
    #if BOOL_LOAD_SWEEPING_ON
        #define OFFSET_VDC_BUS_IPM1 8
        #define SCALE_VDC_BUS_IPM1 0.15384615s
    #elif BOOL_TELEOPERARION_WITH_FORMULA_BOARD
        #define OFFSET_VDC_BUS_IPM1 8
        #define SCALE_VDC_BUS_IPM1 0.15625
    #else
        #define OFFSET_VDC_BUS_IPM1 8
        #define SCALE_VDC_BUS_IPM1 0.16943925233644858
    #endif

    //ADC UVW to PIN config
    /*
    * æ—§çš„Formulaæ�¿å­�ï¼ŒLEMçš„UWä¸¤é¡¹æŽ¥å��äº†ï¼Œå¯¼è‡´AdcaResultRegs.ADCRESULT1å®žé™…æµ‹è¯•çš„Vé¡¹ç”µæµ�
    * æš‚æ—¶çš„è§£å†³æ–¹æ¡ˆï¼šå°†Uå’ŒWå¯¹äºŽåˆ°Axis->iuvw[]çš„ä½�æ•°è°ƒæ�¢ï¼Œi.e,
    * #define PIN_ADCA_U 2
    * #define PIN_ADCA_V 1
    * #define PIN_ADCA_W 0
    * #define PIN_ADCB_U 5
    * #define PIN_ADCB_V 4
    * #define PIN_ADCB_W 3
    */
    #define PIN_ADCA_U 0
    #define PIN_ADCA_V 1
    #define PIN_ADCA_W 2

    #define PIN_ADCB_U 3
    #define PIN_ADCB_V 4
    #define PIN_ADCB_W 5
//    #define PIN_ADCA_U 0
//    #define PIN_ADCA_V 1
//    #define PIN_ADCA_W 2
//
//    #define PIN_ADCB_U 3
//    #define PIN_ADCB_V 4
//    #define PIN_ADCB_W 5

    

    // Lem 2çš„ä¸‰ä¸ªè“�è‰²å�—å�—åˆ†åˆ«æ˜¯adc a1 a2 a3
    // In fact A is the first inverter in MOTOR_GROUP
    #if BOOL_LOAD_SWEEPING_ON
        #define OFFSET_LEM_A1 2035.0 // WuBo tuned in 20241117 //2010  // 2034  // 2029.57894737 // ADCA1
        #define OFFSET_LEM_A2 2045.0 // WuBo tuned in 20241117 //2038   // 2049  // 2043.08771930 // ADCA2
        #define OFFSET_LEM_A3 2053.0 // WuBo tuned in 20241117 //2029   // 2050  // 2042.98245614 // ADCA3
        #define SCALE_LEM_A1 0.030769   // 0.03080704 // ADCA1
        #define SCALE_LEM_A2 0.029947 // 0.03060669 // ADCA2
        #define SCALE_LEM_A3 0.030761  // 0.03045988 // ADCA3
    #elif BOOL_TELEOPERARION_WITH_FORMULA_BOARD
        #define OFFSET_LEM_A1 2030  // ADCA1
        #define OFFSET_LEM_A2 2060  // ADCA2
        #define OFFSET_LEM_A3 2055  // ADCA3
        #define SCALE_LEM_A1 0.0305    // ADCA1
        #define SCALE_LEM_A2 0.030334  // ADCA2
        #define SCALE_LEM_A3 0.031633  // ADCA3
    #else // Default Setup for XXXX (Waiting for a name as a gift for Little Black Board)
    //LEM1-100(äº§å“�ä»£å�·)
        #define OFFSET_LEM_A1 2015 // 2035.0 // WuBo tuned in 20241027 //2010  // 2034  // 2029.57894737 // ADCA1
        #define OFFSET_LEM_A2 2043 // 2047.0 // WuBo tuned in 20241027 //2038   // 2049  // 2043.08771930 // ADCA2
        #define OFFSET_LEM_A3 2037 // 2057.0 // WuBo tuned in 20241027 //2029   // 2050  // 2042.98245614 // ADCA3
        #define SCALE_LEM_A1 0.0305   // 0.03080704 // ADCA1
        #define SCALE_LEM_A2 0.030334 // 0.03060669 // ADCA2
        #define SCALE_LEM_A3 0.031633  // 0.03045988 // ADCA3
    #endif


    #define ADC_OFFSET_0 32768
    #define ADC_OFFSET_1 32768
    #define ADC_OFFSET_2 32768
    #define ADC_OFFSET_3 32768
    #define ADC_OFFSET_4 32768
    #define ADC_OFFSET_5 32768
    #define ADC_OFFSET_6 32768
    #define ADC_OFFSET_7 32768

    #define ADC_SCALE_0 3.125e-4 // 10.24/2^16
    #define ADC_SCALE_1 3.125e-4
    #define ADC_SCALE_2 3.125e-4
    #define ADC_SCALE_3 3.125e-4
    #define ADC_SCALE_4 3.125e-4
    #define ADC_SCALE_5 3.125e-4
    #define ADC_SCALE_6 3.125e-4
    #define ADC_SCALE_7 3.125e-4



    // #if (BOOL_LOAD_SWEEPING_ON == FALSE) || (BOOL_TELEOPERARION_WITH_FORMULA_BOARD == FALSE)
    //     #define OFFSET_LEM_A1 2038 // 2035.0 // WuBo tuned in 20241027 //2010  // 2034  // 2029.57894737 // ADCA1
    //     #define OFFSET_LEM_A2 2050 // 2047.0 // WuBo tuned in 20241027 //2038   // 2049  // 2043.08771930 // ADCA2
    //     #define OFFSET_LEM_A3 2057 // 2057.0 // WuBo tuned in 20241027 //2029   // 2050  // 2042.98245614 // ADCA3
    //     #define SCALE_LEM_A1 0.0305   // 0.03080704 // ADCA1
    //     #define SCALE_LEM_A2 0.030334 // 0.03060669 // ADCA2
    //     #define SCALE_LEM_A3 0.031633  // 0.03045988 // ADCA3
    // #else if BOOL_LOAD_SWEEPING_ON
    //     #define OFFSET_LEM_A1 2035.0 // WuBo tuned in 20241117 //2010  // 2034  // 2029.57894737 // ADCA1
    //     #define OFFSET_LEM_A2 2045.0 // WuBo tuned in 20241117 //2038   // 2049  // 2043.08771930 // ADCA2
    //     #define OFFSET_LEM_A3 2040.0 // WuBo tuned in 20241117 //2029   // 2050  // 2042.98245614 // ADCA3
    //     #define SCALE_LEM_A1 0.030769   // 0.03080704 // ADCA1
    //     #define SCALE_LEM_A2 0.029947 // 0.03060669 // ADCA2
    //     #define SCALE_LEM_A3 0.030761  // 0.03045988 // ADCA3
    // #else if BOOL_TELEOPERARION_WITH_FORMULA_BOARD
    //     #define OFFSET_LEM_A1 2038 // 2035.0 // WuBo tuned in 20241027 //2010  // 2034  // 2029.57894737 // ADCA1
    //     #define OFFSET_LEM_A2 2050 // 2047.0 // WuBo tuned in 20241027 //2038   // 2049  // 2043.08771930 // ADCA2
    //     #define OFFSET_LEM_A3 2057 // 2057.0 // WuBo tuned in 20241027 //2029   // 2050  // 2042.98245614 // ADCA3
    //     #define SCALE_LEM_A1 0.0305   // 0.03080704 // ADCA1
    //     #define SCALE_LEM_A2 0.030334 // 0.03060669 // ADCA2
    //     #define SCALE_LEM_A3 0.031633  // 0.03045988 // ADCA3
    // #endif

    // Lem 1çš„ä¸‰ä¸ªè“�è‰²å�—å�—åˆ†åˆ«æ˜¯adc b7 b8 b9 // ä»¤é€†å�˜å™¨è¾“å‡ºç«¯æŒ‡å�‘ç”µæœºä¸ºæ­£æ–¹å�‘ï¼Œè‹¥LEMä¸Šçš„ç®­å¤´ä¸Žæ­£æ–¹å�‘ç›¸å�Œï¼Œåˆ™SCALEä¸ºæ­£æ•°ï¼Œè‹¥LEMä¸Šçš„ç®­å¤´ä¸Žæ­£æ–¹å�‘ç›¸å��ï¼Œåˆ™SCALEä¸ºè´Ÿæ•°ï¼Œ
    // B is the second inverter in MOTOR_GROUP
    #if BOOL_TELEOPERARION_WITH_FORMULA_BOARD
        #define OFFSET_LEM_B7 2036  // ADCB7
        #define OFFSET_LEM_B8 2062  // ADCB8
        #define OFFSET_LEM_B9 2063  // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
    #else
    // LEM2-101(äº§å“�ä»£å�·)
        #define OFFSET_LEM_B7 2005 //2020 // 2023.89473684 // ADCB7
        #define OFFSET_LEM_B8 2028 // 2029 // 2042.33333333 // ADCB8
        #define OFFSET_LEM_B9 2038 // 2038 // 2043.43859649 // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
    #endif

#endif
#ifdef _INDUCTION_MOTOR_GROUP // mmlab drive version 3 ï¼ˆçŽ‹å�ƒç­‰ï¼‰

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
    #define OFFSET_LEM_B7 2005 // 2024-09-02
    #define OFFSET_LEM_B8 2038 // 2024-09-02
    #define OFFSET_LEM_B9 2043 // 2024-09-02
    #define OFFSET_LEM_A1 2008 // 2024-10-05 cjh tuned
    #define OFFSET_LEM_A2 2036 // 2024-10-05 cjh tuned
    #define OFFSET_LEM_A3 2028 // 2024-10-05 cjh tuned

    #define SCALE_VDC_BUS_IPM1 0.169014 // ???
    #define SCALE_LEM_B7 0.03076297
    #define SCALE_LEM_B8 0.03038256
    #define SCALE_LEM_B9 0.03039058
    #define SCALE_LEM_A1 0.0305
    #define SCALE_LEM_A2 0.030334
    #define SCALE_LEM_A3 0.02983

    // Sensor Coil
    #define OFFSET_PLACE_RIGHT    4461341  
    #define OFFSET_PLACE_DOWN     4639271  

    #define OFFSET_PLACE_LEFT     4561234  
    #define OFFSET_PLACE_UP       4513212  

    #define SCALE_PLACE_X         4.2894311E-5
    #define SCALE_PLACE_Y         4.3412197E-5

#endif

#ifdef _LEG_GROUP // _LEG_GROUP // mmlab drive version 1 (é™ˆè‰ºé“­ã€�æœ±ä¿Šç£Šã€�èƒ¡ç‘€ç­‰)

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

    // Lem 1çš„ä¸‰ä¸ªè“�è‰²å�—å�—åˆ†åˆ«æ˜¯adc b7 b8 b9 // ä»¤é€†å�˜å™¨è¾“å‡ºç«¯æŒ‡å�‘ç”µæœºä¸ºæ­£æ–¹å�‘ï¼Œè‹¥LEMä¸Šçš„ç®­å¤´ä¸Žæ­£æ–¹å�‘ç›¸å�Œï¼Œåˆ™SCALEä¸ºæ­£æ•°ï¼Œè‹¥LEMä¸Šçš„ç®­å¤´ä¸Žæ­£æ–¹å�‘ç›¸å��ï¼Œåˆ™SCALEä¸ºè´Ÿæ•°ï¼Œ
    #define OFFSET_LEM_B7 2025 // 2023.89473684 // ADCB7
    #define OFFSET_LEM_B8 2041 // 2042.33333333 // ADCB8
    #define OFFSET_LEM_B9 2045 // 2043.43859649 // ADCB9
    #define SCALE_LEM_B7 0.03076297 // ADCB7
    #define SCALE_LEM_B8 0.03038256 // ADCB8
    #define SCALE_LEM_B9 0.03039058 // ADCB9

    // Lem 2çš„ä¸‰ä¸ªè“�è‰²å�—å�—åˆ†åˆ«æ˜¯adc a1 a2 a3
    #define OFFSET_LEM_A1 2030      // 2029.57894737 // ADCA1
    #define OFFSET_LEM_A2 2043      // 2043.08771930 // ADCA2
    #define OFFSET_LEM_A3 2042      // 2042.98245614 // ADCA3
    #define SCALE_LEM_A1 0.03080704 // ADCA1
    #define SCALE_LEM_A2 0.03060669 // ADCA2
    #define SCALE_LEM_A3 0.03045988 // ADCA3

    // GongWang Encoder

#endif

#if 0 // Device Lib

    //LEM1-100(äº§å“�ä»£å�·)
        #define OFFSET_LEM_B7 2020 // 2023.89473684 // ADCB7
        #define OFFSET_LEM_B8 2029 // 2042.33333333 // ADCB8
        #define OFFSET_LEM_B9 2038 // 2043.43859649 // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
    // LEM2-101
        #define OFFSET_LEM_B7 2020 // 2023.89473684 // ADCB7
        #define OFFSET_LEM_B8 2029 // 2042.33333333 // ADCB8
        #define OFFSET_LEM_B9 2038 // 2043.43859649 // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
#endif


#endif
