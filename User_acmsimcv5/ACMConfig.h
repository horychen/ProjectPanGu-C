#ifndef ACMCONFIG_H
#define ACMCONFIG_H
/* 缂佸繐鐖剁憰浣锋叏閺�鍦畱 */
#define INVERTER_NONLINEARITY_COMPENSATION_INIT 0 // 5閿涳拷9閺堬拷1閺冦儱寮锋禒銉ュ瀹勶絾婢冪�圭偤鐛欐稉锟介惄瀵告暏閻拷5閿涳拷 // 4 // 1:ParkSul12, 2:Sigmoid, 3:LUT(Obsolete), 4:LUT(by index), 5 Slessinv-a2a3Model
#define INVERTER_NONLINEARITY                   0 // 4 // 1:ModelSul96, 2:ModelExpSigmoid, 3: ModelExpLUT, 4:LUT(by index)
#define SENSORLESS_CONTROL FALSE
#define SENSORLESS_CONTROL_HFSI FALSE
/* ParkSul2012 濮婎垰鑸板▔锟� */
#define GAIN_THETA_TRAPEZOIDAL (40) //(500) // 20

/* 閻㈠灚婧�缁鐎� */ //閿涘湵ODO閿涙岸銈遍崪灞灸侀崹瀣櫡闂堛垻鏁ら惃鍕箷閺勶拷 IM.rr 閼板奔绗夐弰锟� IM.rreq閿涳拷
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 2
	// 閻㈠灚婧�閸欏倹鏆�
	#define PMSM_RESISTANCE                    0.1195
	#define PMSM_D_AXIS_INDUCTANCE             0.000455
	#define PMSM_Q_AXIS_INDUCTANCE             0.0005600000000000001
	#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE 0.01934
	// 闁绢厾澧濋崐锟�
	#define MOTOR_NUMBER_OF_POLE_PAIRS         26
	#define MOTOR_RATED_CURRENT_RMS            17
	#define MOTOR_RATED_POWER_WATT             830
	#define MOTOR_RATED_SPEED_RPM              760
	#define MOTOR_SHAFT_INERTIA                0.000364
	// 閸欏倹鏆熺拠顖氭▕
		#define MISMATCH_R   100
		#define MISMATCH_LD  100
		#define MISMATCH_LQ  100
		#define MISMATCH_KE  100


#if MACHINE_TYPE % 10 == 2
	#define CORRECTION_4_SHARED_FLUX_EST PMSM_PERMANENT_MAGNET_FLUX_LINKAGE
    #define U_MOTOR_KE                   PMSM_PERMANENT_MAGNET_FLUX_LINKAGE
#else
	#define CORRECTION_4_SHARED_FLUX_EST    IM_FLUX_COMMAND_DC_PART
    #define U_MOTOR_R                       IM_STAOTR_RESISTANCE    // typo!
    #define U_MOTOR_RREQ                    IM_ROTOR_RESISTANCE
#endif

/* Algorithms */

    /* Select [Shared Flux Estimator] */
    // #define AFE_USED FE.AFEOE
    // #define AFE_USED FE.huwu
    // #define AFE_USED FE.htz // this is for ESO speed estimation
    #define AFE_USED FE.picorr // this is for ESO speed estimation

    /* Tuning [Shared Flux Estimator] */
        /* AFEOE or CM-VM Fusion */
        #define AFEOE_OMEGA_ESTIMATOR 5 // [rad/s] //0.5 // 5 for slow reversal
            #define AFEOE_KP (200) // (200) // ONLY KP
            #define AFEOE_KI (0) // ONLY KP
        #define AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED FALSE // no need

        /* Hu Wu 1998 recommend tau_1_inv=20 rad/s */
        // #define AFE_21_HUWU_TAU_1_INVERSE (20)
        #define AFE_21_HUWU_TAU_1_INVERSE (200.0) // [rad/s] Alg 2: 0.1 rad/s gives better performance near zero speed, but the converging rate is slower comapred to 0.9 rad/s.
        // #define AFE_21_HUWU_TAU_1_INVERSE (7.5) // [rad/s] Alg 3: increase this will reduce the transient converging time
        #define AFE_21_HUWU_KP (0.2)  //[rad/s]
        #define AFE_21_HUWU_KI (0.5) //[rad/s]

        /* Holtz 2002 */
        // #define HOLTZ_2002_GAIN_OFFSET 20

#if /* PM Motor Observer */ MACHINE_TYPE % 10 == 2
    /* Commissioning */
    #define EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B TRUE
    #if PC_SIMULATION
        #define ENABLE_COMMISSIONING FALSE /*Simulation*/
        #define SELF_COMM_INVERTER FALSE
        #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE
    #else
        #define ENABLE_COMMISSIONING FALSE /*Experiment*/
        #define SELF_COMM_INVERTER FALSE
        #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE
        /*As we use CTRL.O->iab_cmd for look up, now dead-time compensation during ENABLE_COMMISSIONING is not active*/
    #endif

    /* Select Algorithm 2*/
        #define ALG_NSOAF 1
        #define ALG_Park_Sul 2
        #define ALG_Chi_Xu 3
        #define ALG_Qiao_Xia 4
        #define ALG_CJH_EEMF 5
        #define ALG_Farza_2009 6
        #define ALG_Harnefors_2006 7
        #define ALG_ESOAF 10

    // #define SELECT_ALGORITHM ALG_ESOAF
    #define SELECT_ALGORITHM ALG_NSOAF
    // #define SELECT_ALGORITHM ALG_Chi_Xu

        #if SELECT_ALGORITHM == ALG_Chi_Xu
            #define ELECTRICAL_SPEED_FEEDBACK    chixu.xOmg
            #define ELECTRICAL_POSITION_FEEDBACK chixu.theta_d
        #elif SELECT_ALGORITHM == ALG_NSOAF
            #define ELECTRICAL_SPEED_FEEDBACK    nsoaf.xOmg // harnefors.omg_elec
            #define ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d // harnefors.theta_d
        #elif SELECT_ALGORITHM == ALG_ESOAF
            #define ELECTRICAL_SPEED_FEEDBACK    (-esoaf.xOmg) // 閽栧嫮澧栭悽鍨簚鐎圭偤鐛欏顤痲娴溠呮晸鐠愮喕娴嗛柅锟�
            #define ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d
        #else
            // #define ELECTRICAL_SPEED_FEEDBACK    G.omg_elec
            // #define ELECTRICAL_POSITION_FEEDBACK G.theta_d

            // #define ELECTRICAL_SPEED_FEEDBACK    parksul.xOmg
            // #define ELECTRICAL_POSITION_FEEDBACK parksul.theta_d

            // #define ELECTRICAL_SPEED_FEEDBACK    qiaoxia.xOmg
            // #define ELECTRICAL_POSITION_FEEDBACK qiaoxia.theta_d

            #define ELECTRICAL_SPEED_FEEDBACK    CTRL.I->omg_elec
            #define ELECTRICAL_POSITION_FEEDBACK CTRL.I->theta_d_elec
        #endif

    /* Tuning Algorithm 2 */
    #define LOW_SPEED_OPERATION  1
    #define HIGH_SPEED_OPERATION 2
    #define OPERATION_MODE HIGH_SPEED_OPERATION

        /* Park.Sul 2014 FADO in replace of CM */
        #define PARK_SUL_OPT_1 (2*M_PI*60)
        #define PARK_SUL_OPT_2 (2*M_PI*35)
            #define PARK_SUL_T2S_1_KP (PARK_SUL_OPT_1*2)
            #define PARK_SUL_T2S_1_KI (PARK_SUL_OPT_1*PARK_SUL_OPT_1)
            #define PARK_SUL_T2S_2_KP (PARK_SUL_OPT_2*2)
            #define PARK_SUL_T2S_2_KI (PARK_SUL_OPT_2*PARK_SUL_OPT_2)
        #define PARK_SUL_CM_OPT 5 // [rad/s] pole placement
            #define PARK_SUL_CM_KP (PARK_SUL_CM_OPT*2)
            #define PARK_SUL_CM_KI (PARK_SUL_CM_OPT*PARK_SUL_CM_OPT)

    /* Chi.Xu 2009 SMO for EMF of SPMSM (Coupled position estimation via MRAS) */
    #define CHI_XU_USE_CONSTANT_SMO_GAIN TRUE
        #define CHI_XU_SIGMOID_COEFF  500 /*濮ｏ拷200婢堆備簰閸氬函绱濋崷銊ョ杽妤犲奔鑵戦弮鐘冲妳闁喎瀹崇粙铏拷浣筋嚖瀹割喕绗夋导姘晙閸戝繐鐨禍鍡礉娴ｅ棙妲告导姘閸濆秵鍙冮崣宥堟祮*/
    #if OPERATION_MODE == LOW_SPEED_OPERATION
        /* note ell4Zeq is -0.5 */
        #define CHI_XU_USE_CONSTANT_LPF_POLE TRUE
        #if PC_SIMULATION
            #define CHI_XU_SMO_GAIN_SCALE 10.0  //2
            #define CHI_XU_LPF_4_ZEQ    5.0   //10.0
        #else
            #define CHI_XU_SMO_GAIN_SCALE 10 /*閸欙拷2鐎圭偤鐛欓弮鐘冲妳缁嬭櫕锟戒椒绗夌粙绛圭礉閸欙拷5閹便垹寮芥潪顒�濯哄鐑樺灇閸旂噦绱濋崣锟�10閹便垹寮芥潪顒佸灇閸旓拷*/
            #define CHI_XU_LPF_4_ZEQ    (5.0) /*鏉╂瑩銆嶆潻鍥с亣閿涘潒g=100閿涘绱扮�佃壈鍤х憴鎺戝缁嬭櫕锟戒浇顕ゅ顕嗙礉韫囨顔囨禍鍡曠稑鐏忚精鐦拠鏇犳箙閿涘苯褰�=2閿涳拷=5閿涳拷=10閿涳拷=100閸掑棗鍩嗘禒鍖＄磼閻噦绱掗惇瀣箙閵嗭拷*/
        #endif

        #define CHI_XU_SPEED_PLL_KP (500*2.0) // [rad/s]
        #define CHI_XU_SPEED_PLL_KI (500*500.0)
    #elif OPERATION_MODE == HIGH_SPEED_OPERATION
        /* note ell4Zeq will become 1 */
        #define CHI_XU_SMO_GAIN_SCALE  1.5
        #define CHI_XU_LPF_4_ZEQ       10.0
        #define CHI_XU_USE_CONSTANT_LPF_POLE FALSE
        #define CHI_XU_SPEED_PLL_KP (2*500) // [rad/s] 3000 = 闂冩儼绌潪顒勶拷鐔剁瑝闂囧洩宕遍敍锟�8000=闂冩儼绌潪顒勶拷鐔风发闂囧洩宕�
        #define CHI_XU_SPEED_PLL_KI (10e4) // 娴狅拷350000閸戝繐鐨稉锟�150000閿涘苯褰叉禒銉ュ櫤鐏忔垹菙閹椒鍙婄拋陇娴嗛柅鐔烘畱濞夈垹濮�
    #endif

    /* Qiao.Xia 2013 SMO for EMF of SPMSM */
        #define QIAO_XIA_SIGMOID_COEFF  5000 //200 // 20
        #define QIAO_XIA_SMO_GAIN       1.5 //1.5     // 1.5
        #define QIAO_XIA_MRAS_GAIN      500 //500       // 50
        #define QIAO_XIA_ADAPT_GAIN     500 //2000 // 250 // 100

    /* CHEN 2020 NSO with Active Flux Concept */
        // #define NSOAF_SPMSM // use AP Error
        #define NSOAF_IPMSM // use only OE
        #define TUNING_IGNORE_UQ TRUE
        #define NSOAF_OMEGA_OBSERVER 300 // >150 [rad/s] // cannot be too small (e.g., 145, KP will be negative), 
            #define NSOAF_TL_P (1) // 1 for experimental starting // 4 for 1500 rpm // 2 for 800 rpm
            #define NSOAF_TL_I (20)
            #define NSOAF_TL_D (0)

    /* CHEN 2021 ESO with Active Flux Concept */
        // #define ESOAF_OMEGA_OBSERVER 10
        // #define ESOAF_OMEGA_OBSERVER 30 // 30 gives acceptable steady state speed ripple, 200
        // #define ESOAF_OMEGA_OBSERVER 150 // 150 gives acceptable disturbance rejection when sudden 3 A load is applied and keeps the system not stop when Vdc changes from 150 V to 300 V.
        #define ESOAF_OMEGA_OBSERVER 200 // 200 gives acceptable disturbance rejection when load changes between 1.5 A and 3 A.

    /* Farza 2009 for EMMF */
        #define FARZA09_HGO_EEMF_VARTHETA 10
        #define FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE 10

    /* CJH EEMF AO Design */
        #define CJH_EEMF_K1 (100)
        #define CJH_EEMF_K2 (CJH_EEMF_K1*CJH_EEMF_K1*0.25) // see my TCST paper@(18)
        #define CJH_EEMF_GAMMA_OMEGA (5e6)

    /* Harnefors 2006 */

#elif /* Induction Motor Observer */ MACHINE_TYPE % 10 == 1
    // Marino05 鐠嬪啫寮� /// default: (17143), (2700.0), (1000), (1), (0)
    #define GAMMA_INV_xTL 17142.85714285714
    #define LAMBDA_INV_xOmg 1000 // 2700.0 is too large, leading to unstable flux amplitude contorl
    #define DELTA_INV_alpha (0*500) // 1000
    #define xAlpha_LAW_TERM_D 1 // regressor is commanded d-axis rotor current, and error is d-axis flux control error.
    #define xAlpha_LAW_TERM_Q 0 // regressor is commanded q-axis stator current, and error is q-axis flux control error.
    // 绾句線鎽奸崣宥夘洯閻€劏鐨� /// "htz",,ohtani",picorr",lascu",clest",harnefors
    // #define IFE FE.picorr
    #define IFE FE.htz
    #define FLUX_FEEDBACK_ALPHA         IFE.psi_2[0]
    #define FLUX_FEEDBACK_BETA          IFE.psi_2[1]
    #define OFFSET_COMPENSATION_ALPHA   IFE.u_offset[0]
    #define OFFSET_COMPENSATION_BETA    IFE.u_offset[1]

    // Ohtani 绾句線鎽肩憴鍌涚ゴ缁粯鏆熼柊宥囩枂/// default: 5
    // Ohtani 瀵ら缚顔呴崣鏍э拷鐓庢嫲鏉烆剙鐡欓弮鍫曟？鐢憡鏆熼惄鍝ョ搼
    #define GAIN_OHTANI (5)
    #define VM_OHTANI_CORRECTION_GAIN_P (5)
    /* B *//// default: P=5, I=2.5
    #define VM_PROPOSED_PI_CORRECTION_GAIN_P 50 // 20閺嗗倹锟戒浇鍏樼捄鐔剁瑐娴ｅ棙妲搁崣鍫熺哺閸氬函绱�200閸氬酣娼伴弳鍌涳拷浣告皑鐠虹喍绗夋稉濠佺啊  //10 // (5)
    #define VM_PROPOSED_PI_CORRECTION_GAIN_I 0.0 //2.5 //2  // (2.5)
    /* C *//// default: P=0.125*5, I=0.125*2.5, KCM=0
    #define OUTPUT_ERROR_CLEST_GAIN_KP (0.125*5)
    #define OUTPUT_ERROR_CLEST_GAIN_KI (0.125*2.5)
    #define OUTPUT_ERROR_CLEST_GAIN_KCM (0*0.8)
    /* Holtz 2002 */// default: 20
    #define HOLTZ_2002_GAIN_OFFSET 10 //1 // 20 is too large, causing unstable control during reversal
    /* Harnefors SCVM 2003 */// default: 2
    #define GAIN_HARNEFORS_LAMBDA 2

    #define SELECT_ALGORITHM -1111
	// #if 1
        #define ELECTRICAL_SPEED_FEEDBACK    marino.xOmg // CTRL.I->omg_elec
        #define ELECTRICAL_POSITION_FEEDBACK marino.xRho // CTRL.I->theta_d_elec
	// #endif
#endif

/* 閹貉冨煑缁涙牜鏆� */
	#define NULL_D_AXIS_CURRENT_CONTROL -1
	#define MTPA -2 // not supported
#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
#define NUMBER_OF_STEPS 50000
    #define DOWN_SAMPLE 1
    #define USE_QEP_RAW FALSE
    #define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT FALSE
    #define SATURATED_MAGNETIC_CIRCUIT FALSE
#define CL_TS          (0.0001)
#define CL_TS_INVERSE  (10000)
    #define TS_UPSAMPLING_FREQ_EXE 1.0 //0.5
    #define TS_UPSAMPLING_FREQ_EXE_INVERSE 1 //2
#define VL_TS          (0.0005)
    #define PL_TS VL_TS
    #define SPEED_LOOP_CEILING ((int)(VL_TS*CL_TS_INVERSE))
    #define MACHINE_TS         (CL_TS*TS_UPSAMPLING_FREQ_EXE)
    #define MACHINE_TS_INVERSE (CL_TS_INVERSE*TS_UPSAMPLING_FREQ_EXE_INVERSE)

#define LOAD_INERTIA    0.0
#define LOAD_TORQUE     0.0
#define VISCOUS_COEFF   0.0007

#define CL_SERIES_KP (31.2903)
#define CL_SERIES_KI (122.088)
#define VL_SERIES_KP (1.79885)
#define VL_SERIES_KI (29.7429)

#define CURRENT_KP (0.79091)
#define CURRENT_KI (213.393)
    #define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define CURRENT_LOOP_LIMIT_VOLTS (48)

#define SPEED_KP (-0.00403304)
#define SPEED_KI (33.4282)
    #define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
    #define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (MOTOR_RATED_CURRENT_RMS*1.414) )
    #define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / MOTOR_NUMBER_OF_POLE_PAIRS )
    #define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/MOTOR_NUMBER_OF_POLE_PAIRS/2/3.1415926*60) )

    #define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)
    #define SPEED_LOOP_LIMIT_NEWTON_METER (1.0*MOTOR_RATED_TORQUE)
    #define SPEED_LOOP_LIMIT_AMPERE       (1.0*1.414*MOTOR_RATED_CURRENT_RMS)
    // increase to 3 times because of the bug in dynamics clamping

/* Encoder QEP TODO: should read from excel */
#define INCREMENTAL_ENCODER_QEP 1
#define ABSOLUTE_ENCODER_SCI 2
#define ABSOLUTE_ENCODER_CAN_ID0x01 3
#define ABSOLUTE_ENCODER_CAN_ID0x03 4
#define RESOLVER_1 5
#define RESOLVER_2 6

#define ENCODER_TYPE ABSOLUTE_ENCODER_CAN_ID0x01

#if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
#define SYSTEM_QEP_PULSES_PER_REV (10000)
#define SYSTEM_QEP_REV_PER_PULSE (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#define SYSTEM_QEP_QPOSMAX (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (10000)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19
#elif ENCODER_TYPE == ABSOLUTE_ENCODER_SCI
#define SYSTEM_QEP_PULSES_PER_REV (8388608)
#define SYSTEM_QEP_REV_PER_PULSE (1.1920929e-7)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2731723 // ym tuned with id_cmd = 2A 2024-01-31
#elif (ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x01)||(ENCODER_TYPE == ABSOLUTE_ENCODER_CAN_ID0x03)
#define SYSTEM_QEP_PULSES_PER_REV (131072)
#define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 27317 // ym tuned with id_cmd = 2A 2024-01-31
#elif ENCODER_TYPE == RESOLVER_1
#define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
#define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
#define SYSTEM_QEP_QPOSMAX (65535)                  // (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (65536)
#define SYSTEM_QEP_PULSES_PER_REV (65536 * RESOLVER_NUMBER_OF_POLE_PAIRS)                     // (10000)
#define SYSTEM_QEP_REV_PER_PULSE (1.52587890625e-05 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#elif ENCODER_TYPE == RESOLVER_2
#define RESOLVER_NUMBER_OF_POLE_PAIRS 4             // Receive 4 Z-pulses per mechnical revolution from the resolver
#define ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS 0.25 // 1/RESOLVER_NUMBER_OF_POLE_PAIRS
#define SYSTEM_QEP_QPOSMAX (4095)                   // (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (4096)
#define SYSTEM_QEP_PULSES_PER_REV (4096 * RESOLVER_NUMBER_OF_POLE_PAIRS)                   // (10000)
#define SYSTEM_QEP_REV_PER_PULSE (0.000244140625 * ONE_OVER_RESOLVER_NUMBER_OF_POLE_PAIRS) // (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#endif

/* 閹稿洣鎶ょ猾璇茬�� */
    #define EXCITATION_POSITION 0
    #define EXCITATION_VELOCITY 1
    #define EXCITATION_SWEEP_FREQUENCY 2
#define EXCITATION_TYPE (1)

/* Sweep Frequency */
#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#define DATA_FILE_NAME "../dat/TEST_PHIL_LAB_PID_CODES-225-1000-2-12068.dat"

#endif
