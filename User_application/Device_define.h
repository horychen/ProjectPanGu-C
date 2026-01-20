
#ifndef DEVICE_DEFINE_H
#define DEVICE_DEFINE_H
#ifdef _PROJECT_FORMULA_GROUP // mmlab drive version 4 鑼傚綍钘涜寘閳┾懞鍠€劉锟藉ゥ濂解斁锟芥簩顒碱灎顓涳拷鎳婎嚪灏栵拷锟�

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

#ifdef _MOTOR_GROUP // mmlab drive version 2 鑼傚綍钘涙皳閿熼摪鐤滴查檲鈷氾絺鍋撻敓鐭仮鍢庛儍锔癸拷婢涙５顬狀伩鐘嗭絺鍋撻敓鐭讣鎷风瘬姘撻閿熺煫锔兼嫹娼炶幗棰呴垾鎳婎嚪灏栵拷锟�

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
        #define OFFSET_VDC_BUS_IPM1 9
        #define SCALE_VDC_BUS_IPM1 0.160679996
    #endif

    //ADC UVW to PIN config
    /*
    * 蹇欓垾鏂咁灐顬ｂ檧锟芥楷ormula蹇欓敓閾版５銉傤叏鎷疯寕褰曡墬LEM鑾芥嫥閳ユ拷W鐩茶祩闄囪寘闅嗛箍蹇欐菠妤兼皳閿熸枻鎷风洸娼為垾鐘嗩嚪瀵傛崡銉傤嚪娴庛劉锟矫风dcaResultRegs.ADCRESULT1姘撳簮鍟寘閳┾懇锟轿澄茬锟藉顭婎垪锟解挌顬ｂ檧锟芥烤鑼呴殕楣胯幗閳ユ緵寰椢茬鎷�
    * 蹇欐嫥閳ユ锔癸拷鏂呯澒顬ｂ檧锟界伝顭婎灎锛ｃ儮锟界妳瓒侊腹锟芥拃濮懳测埛鍠㈩嚪瀵傗埗銉傛壋锟界嚡姘撻垾娅壓姘撶倝楣跨洸娼炴菠姘撹棝鎺矨xis->iuvw[]鑾芥嫥閳ョ伝銇㈡枻鎷峰繖閳モ挋鎳婎煀鎹屾崡锔兼嫹鍨勮寕褰曡墬i.e,
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

    

    // Lem 2鑾芥嫥閳ョ伝銇㈢硶锟芥噴銇㈠槑顎€劉锟芥粣鎷风尗閳ュゥ瑁併儻鎷烽垾鏂嗐儻鎷烽垾鏂嗐儖鍡忥拷鐘嗐儖鍠☆個位婧岀棏dc a1 a2 a3
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
    //LEM1-100(鐩叉綖鎼傛皳閳ユ粣鎷风洸绂勬嫝姘撻敓閾帮拷)
        #define OFFSET_LEM_A1 2013 // 2035.0 // WuBo tuned in 20241027 //2010  // 2034  // 2029.57894737 // ADCA1
        #define OFFSET_LEM_A2 2039 // 2047.0 // WuBo tuned in 20241027 //2038   // 2049  // 2043.08771930 // ADCA2
        #define OFFSET_LEM_A3 2030 // 2057.0 // WuBo tuned in 20241027 //2029   // 2050  // 2042.98245614 // ADCA3
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

    // Lem 1鑾芥嫥閳ョ伝銇㈢硶锟芥噴銇㈠槑顎€劉锟芥粣鎷风尗閳ュゥ瑁併儻鎷烽垾鏂嗐儻鎷烽垾鏂嗐儖鍡忥拷鐘嗐儖鍠☆個位婧岀棏dc b7 b8 b9 // 鐩茬闄囪寘閳ь兘锟界妴銉嫹钘存皳閳┾挋顭嬵煀閿拷婧嶃儮锟矫峰ソ顬狀伩顕肝垫巻锟解埗銉嫹閳ユ◤褉锟芥緵寰椢垫拃濂姐仮鍢庡ソ尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯忣嚪瀵傛崡銊拷瀛ゎ檼EM鐩茶祩鑹╄幗鎷ч垾鐏活灎顔欘劶銉傘仮鐤点仮姒傜煫尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯徰�锟介儩璇ャ儻鎷疯墬鑼傚綍鑹楁皳钘涢埄顢笴ALE鐩茶祩娼炲繖棰呮嫝蹇欓垾鈷欐噴顕峰瘋鎹椼劉锟藉顧扙M鐩茶祩鑹╄幗鎷ч垾鐏活灎顔欘劶銉傘仮鐤点仮姒傜煫尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯徰�锟介儩璇ャ儻鎷烽敓鐭嚪瀵傛崡銉嬪棌鍔塖CALE鐩茶祩娼炵尗楹撻弗蹇欓垾鈷欐噴顕峰瘋锟�
    // B is the second inverter in MOTOR_GROUP
    #if BOOL_TELEOPERARION_WITH_FORMULA_BOARD
        #define OFFSET_LEM_B7 2036  // ADCB7
        #define OFFSET_LEM_B8 2062  // ADCB8
        #define OFFSET_LEM_B9 2063  // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
    #else
    // LEM2-101(鐩叉綖鎼傛皳閳ユ粣鎷风洸绂勬嫝姘撻敓閾帮拷)
        #define OFFSET_LEM_B7 2012 //2020 // 2023.89473684 // ADCB7
        #define OFFSET_LEM_B8 2042 // 2029 // 2042.33333333 // ADCB8
        #define OFFSET_LEM_B9 2033 // 2038 // 2043.43859649 // ADCB9
        #define SCALE_LEM_B7 0.03076297 // ADCB7
        #define SCALE_LEM_B8 0.03038256 // ADCB8
        #define SCALE_LEM_B9 0.03039058 // ADCB9
    #endif

#endif
#ifdef _INDUCTION_MOTOR_GROUP // mmlab drive version 3 鑼傚綍钘涜幗娌ら垾濮戙儻鎷疯尃鑾介閳ユ噴顕峰皷锟斤拷

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
    #define OFFSET_PLACE_RIGHT    6033004
    #define OFFSET_PLACE_DOWN     6033004

    #define OFFSET_PLACE_LEFT     6033004  //4561234
    #define OFFSET_PLACE_UP       6033004  //4513212
    //X:right and left; Y:down and up.
    #define SCALE_PLACE_X         4.8114889989487935e-06  //4.2894311E-5
    #define SCALE_PLACE_Y         4.8114889989487935e-06  //4.3412197E-5
#endif

#ifdef _LEG_GROUP // _LEG_GROUP // mmlab drive version 1 (鑼呴埄鈶哄枹銊拷濂ュソ鈹撅拷婧岊劶锝傚亾閿熺煫蔚鎾�娉点仮棰楃妴顬狅讥鐘嗭絺鍋撻敓鐭剢鎹栤埗褉锟芥ǚ鍋撹幗棰呴垾锟�)

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

    // Lem 1鑾芥嫥閳ョ伝銇㈢硶锟芥噴銇㈠槑顎€劉锟芥粣鎷风尗閳ュゥ瑁併儻鎷烽垾鏂嗐儻鎷烽垾鏂嗐儖鍡忥拷鐘嗐儖鍠☆個位婧岀棏dc b7 b8 b9 // 鐩茬闄囪寘閳ь兘锟界妴銉嫹钘存皳閳┾挋顭嬵煀閿拷婧嶃儮锟矫峰ソ顬狀伩顕肝垫巻锟解埗銉嫹閳ユ◤褉锟芥緵寰椢垫拃濂姐仮鍢庡ソ尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯忣嚪瀵傛崡銊拷瀛ゎ檼EM鐩茶祩鑹╄幗鎷ч垾鐏活灎顔欘劶銉傘仮鐤点仮姒傜煫尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯徰�锟介儩璇ャ儻鎷疯墬鑼傚綍鑹楁皳钘涢埄顢笴ALE鐩茶祩娼炲繖棰呮嫝蹇欓垾鈷欐噴顕峰瘋鎹椼劉锟藉顧扙M鐩茶祩鑹╄幗鎷ч垾鐏活灎顔欘劶銉傘仮鐤点仮姒傜煫尾顒伙迹锔癸拷鎾�濮戙儻鎷烽垾妯徰�锟介儩璇ャ儻鎷烽敓鐭嚪瀵傛崡銉嬪棌鍔塖CALE鐩茶祩娼炵尗楹撻弗蹇欓垾鈷欐噴顕峰瘋锟�
    #define OFFSET_LEM_B7 2025 // 2023.89473684 // ADCB7
    #define OFFSET_LEM_B8 2041 // 2042.33333333 // ADCB8
    #define OFFSET_LEM_B9 2045 // 2043.43859649 // ADCB9
    #define SCALE_LEM_B7 0.03076297 // ADCB7
    #define SCALE_LEM_B8 0.03038256 // ADCB8
    #define SCALE_LEM_B9 0.03039058 // ADCB9

    // Lem 2鑾芥嫥閳ョ伝銇㈢硶锟芥噴銇㈠槑顎€劉锟芥粣鎷风尗閳ュゥ瑁併儻鎷烽垾鏂嗐儻鎷烽垾鏂嗐儖鍡忥拷鐘嗐儖鍠☆個位婧岀棏dc a1 a2 a3
    #define OFFSET_LEM_A1 2030      // 2029.57894737 // ADCA1
    #define OFFSET_LEM_A2 2043      // 2043.08771930 // ADCA2
    #define OFFSET_LEM_A3 2042      // 2042.98245614 // ADCA3
    #define SCALE_LEM_A1 0.03080704 // ADCA1
    #define SCALE_LEM_A2 0.03060669 // ADCA2
    #define SCALE_LEM_A3 0.03045988 // ADCA3

    // GongWang Encoder

#endif

#if 0 // Device Lib

    //LEM1-100(鐩叉綖鎼傛皳閳ユ粣鎷风洸绂勬嫝姘撻敓閾帮拷)
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
