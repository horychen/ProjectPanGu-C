#ifndef ACMCONFIG_H
#define ACMCONFIG_H
/* 经常要修改的 */
#define INVERTER_NONLINEARITY_COMPENSATION_INIT 2 // 1:ParkSul12, 2:Sigmoid, 3:LUT
#define INVERTER_NONLINEARITY                   2 // 1:ModelSul96, 2:ModelExpSigmoid, 3: ModelExpLUT
#define SENSORLESS_CONTROL TRUE
#define SENSORLESS_CONTROL_HFSI FALSE
/* ParkSul2012 梯形波 */
#define GAIN_THETA_TRAPEZOIDAL (40) //(500) // 20

/* 电机类型 */ //（TODO：饱和模型里面用的还是 IM.rr 而不是 IM.rreq）
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 2
	// 电机参数
	#define PMSM_RESISTANCE                    1.179
	#define PMSM_D_AXIS_INDUCTANCE             0.004957
	#define PMSM_Q_AXIS_INDUCTANCE             0.004957
	#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE 0.1342
	// 铭牌值
	#define MOTOR_NUMBER_OF_POLE_PAIRS         4
	#define MOTOR_RATED_CURRENT_RMS            3
	#define MOTOR_RATED_POWER_WATT             750
	#define MOTOR_RATED_SPEED_RPM              1500
	#define MOTOR_SHAFT_INERTIA                0.0006168
	// 参数误差
		#define MISMATCH_R   100
		#define MISMATCH_LD  100
		#define MISMATCH_LQ  100
		#define MISMATCH_KE  100

/* Algorithms */
#if /* PM Motor */ MACHINE_TYPE % 10 == 2
    #if PC_SIMULATION
        #define ENABLE_COMMISSIONING FALSE /*Simulation*/
    #else
        #define ENABLE_COMMISSIONING FALSE /*Experiment*/
        /*As we use CTRL.O->iab_cmd for look up, now dead-time compensation during ENABLE_COMMISSIONING is not active*/
    #endif

    /* Select Algorithm 1 */
    #define AFE_USED AFEOE
    // #define AFE_USED huwu

    /* Tuning Algorithm 1 */
        /* AFEOE or CM-VM Fusion */
        #define AFEOE_OMEGA_ESTIMATOR 5 // [rad/s] //0.5 // 5 for slow reversal
            #define AFEOE_KP (200) // ONLY KP
            #define AFEOE_KI (0.0) // ONLY KP
        #define AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED FALSE // no need

        /* Hu Wu 1998 recommend tau_1_inv=20 rad/s */
        // #define AFE_21_HUWU_TAU_1_INVERSE (20)
        #define AFE_21_HUWU_TAU_1_INVERSE (200.0) // [rad/s] Alg 2: 0.1 rad/s gives better performance near zero speed, but the converging rate is slower comapred to 0.9 rad/s.
        // #define AFE_21_HUWU_TAU_1_INVERSE (7.5) // [rad/s] Alg 3: increase this will reduce the transient converging time
        #define AFE_21_HUWU_KP (0.2)  //[rad/s]
        #define AFE_21_HUWU_KI (0.5) //[rad/s]

    /* Select Algorithm 2*/
        #define ALG_NSOAF 1
        #define ALG_Park_Sul 2
        #define ALG_Chi_Xu 3
        #define ALG_Qiao_Xia 4
        #define ALG_CJH_EEMF 5
        #define ALG_Farza_2009 6
        #define ALG_Harnefors_2006 7
    #define SELECT_ALGORITHM ALG_NSOAF
    // #define SELECT_ALGORITHM ALG_Chi_Xu
        #if SELECT_ALGORITHM == ALG_Chi_Xu
            #define ELECTRICAL_SPEED_FEEDBACK    chixu.xOmg
            #define ELECTRICAL_POSITION_FEEDBACK chixu.theta_d
        #elif SELECT_ALGORITHM == ALG_NSOAF
            #define ELECTRICAL_SPEED_FEEDBACK    nsoaf.xOmg // harnefors.omg_elec
            #define ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d // harnefors.theta_d
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
        #define CHI_XU_SIGMOID_COEFF  500 /*比200大以后，在实验中无感速度稳态误差不会再减小了，但是会影响慢反转*/
    #if OPERATION_MODE == LOW_SPEED_OPERATION
        /* note ell4Zeq is -0.5 */
        #define CHI_XU_USE_CONSTANT_LPF_POLE TRUE
        #if PC_SIMULATION
            #define CHI_XU_SMO_GAIN_SCALE 10.0  //2
            #define CHI_XU_LPF_4_ZEQ    5.0   //10.0
        #else
            #define CHI_XU_SMO_GAIN_SCALE 10 /*取2实验无感稳态不稳，取5慢反转勉强成功，取10慢反转成功*/
            #define CHI_XU_LPF_4_ZEQ    (5.0) /*这项过大（eg=100）会导致角度稳态误差，忘记了你就试试看，取=2，=5，=10，=100分别仿！真！看看。*/
        #endif

        #define CHI_XU_SPEED_PLL_KP (500*2.0) // [rad/s]
        #define CHI_XU_SPEED_PLL_KI (500*500.0)
    #elif OPERATION_MODE == HIGH_SPEED_OPERATION
        /* note ell4Zeq will become 1 */
        #define CHI_XU_SMO_GAIN_SCALE  1.5
        #define CHI_XU_LPF_4_ZEQ       10.0
        #define CHI_XU_USE_CONSTANT_LPF_POLE FALSE
        #define CHI_XU_SPEED_PLL_KP (2*500) // [rad/s] 3000 = 阶跃转速不震荡，8000=阶跃转速很震荡
        #define CHI_XU_SPEED_PLL_KI (10e4) // 从350000减少为150000，可以减少稳态估计转速的波动
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

    /* Farza 2009 for EMMF */
        #define FARZA09_HGO_EEMF_VARTHETA 10
        #define FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE 10

    /* CJH EEMF AO Design */
        #define CJH_EEMF_K1 (100)
        #define CJH_EEMF_K2 (CJH_EEMF_K1*CJH_EEMF_K1*0.25) // see my TCST paper@(18)
        #define CJH_EEMF_GAMMA_OMEGA (5e6)

    /* Harnefors 2006 */

#elif /* Induction Motor */ MACHINE_TYPE % 10 == 1
    // Marino05 调参 /// default: (17143), (2700.0), (1000), (1), (0)
    #define GAMMA_INV_xTL 17142.85714285714
    #define LAMBDA_INV_xOmg 2700.0
    #define DELTA_INV_alpha (0*1000)
    #define xAlpha_LAW_TERM_D 1 // regressor is commanded d-axis rotor current, and error is d-axis flux control error.
    #define xAlpha_LAW_TERM_Q 0 // regressor is commanded q-axis stator current, and error is q-axis flux control error.
    // 磁链反馈用谁 /// "htz",,ohtani",picorr",lascu",clest",harnefors
    #define IFE htz
    #define FLUX_FEEDBACK_ALPHA         IFE.psi_2[0]
    #define FLUX_FEEDBACK_BETA          IFE.psi_2[1]
    #define OFFSET_COMPENSATION_ALPHA   IFE.u_offset[0]
    #define OFFSET_COMPENSATION_BETA    IFE.u_offset[1]
    // Ohtani 磁链观测系数配置/// default: 5
    // Ohtani 建议取值和转子时间常数相等
    #define GAIN_OHTANI (5)
    #define VM_OHTANI_CORRECTION_GAIN_P (5)
    /* B *//// default: P=5, I=2.5
    #define VM_PROPOSED_PI_CORRECTION_GAIN_P (5)
    #define VM_PROPOSED_PI_CORRECTION_GAIN_I (2.5)
    /* C *//// default: P=0.125*5, I=0.125*2.5, KCM=0
    #define OUTPUT_ERROR_CLEST_GAIN_KP (0.125*5)
    #define OUTPUT_ERROR_CLEST_GAIN_KI (0.125*2.5)
    #define OUTPUT_ERROR_CLEST_GAIN_KCM (0*0.8)
    /* Holtz 2002 */// default: 20
    #define HOLTZ_2002_GAIN_OFFSET 20
    /* Harnefors SCVM 2003 */// default: 2
    #define GAIN_HARNEFORS_LAMBDA 2
#endif

/* 控制策略 */
	#define NULL_D_AXIS_CURRENT_CONTROL -1
	#define MTPA -2 // not supported
#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
#define NUMBER_OF_STEPS 150000
    #define DOWN_SAMPLE 1
    #define USE_QEP_RAW TRUE
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
#define LOAD_TORQUE     2.5
#define VISCOUS_COEFF   0.0007

#define CURRENT_KP (6.39955)
#define CURRENT_KI (237.845)
    #define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define CURRENT_LOOP_LIMIT_VOLTS (110)

#define SPEED_KP (0.0380362)
#define SPEED_KI (30.5565)
    #define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
    #define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (MOTOR_RATED_CURRENT_RMS*1.414) )
    #define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / MOTOR_NUMBER_OF_POLE_PAIRS )
    #define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/MOTOR_NUMBER_OF_POLE_PAIRS/2/3.1415926*60) )

    #define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)
    #define SPEED_LOOP_LIMIT_NEWTON_METER (3.0*MOTOR_RATED_TORQUE)
    #define SPEED_LOOP_LIMIT_AMPERE       (3.0*1.414*MOTOR_RATED_CURRENT_RMS)
    // increase to 3 times because of the bug in dynamics clamping

/* Encoder QEP TODO: should read from excel */
#define SYSTEM_QEP_PULSES_PER_REV  (10000)
#define SYSTEM_QEP_REV_PER_PULSE  (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2*M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)


/* 指令类型 */
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

#define DATA_FILE_NAME "../dat/PMSM_NTUMotor-205-1000-10-1982.dat"
#endif
