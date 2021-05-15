#ifndef ACMCONFIG_H
#define ACMCONFIG_H
/* Consistent with DSP Codes */
// 电机类型（TODO：饱和模型里面用的还是 IM.rr 而不是 IM.rreq）
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
	#define MOTOR_RATED_CURRENT_RMS            4.2
	#define MOTOR_RATED_POWER_WATT             750
	#define MOTOR_RATED_SPEED_RPM              1500
	#define MOTOR_SHAFT_INERTIA                0.0006168
	// 参数误差
		#define MISMATCH_R   100
		#define MISMATCH_LD  100
		#define MISMATCH_LQ  100
		#define MISMATCH_KE  100

// 指令类型
    #define EXCITATION_POSITION 0
    #define EXCITATION_VELOCITY 1
    #define EXCITATION_SWEEP_FREQUENCY 2
#define EXCITATION_TYPE (1)

// 控制策略
	#define NULL_D_AXIS_CURRENT_CONTROL -1
	#define MTPA -2 // not supported
#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
#define NUMBER_OF_STEPS 300000
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
#define SPEED_LOOP_CEILING (4)

// 经常要修改的
#define SENSORLESS_CONTROL_HFSI FALSE
#define SENSORLESS_CONTROL FALSE
#define INVERTER_NONLINEARITY_COMPENSATION 1 // 1:ParkSul12, 2:Sigmoid, 3:LUT
#define INVERTER_NONLINEARITY              2 // 1:ModelSul96, 2:ModelExpSigmoid, 3: ModelExpLUT
/* ParkSul2012 梯形波 */
#define GAIN_THETA_TRAPEZOIDAL (40) //(500) // 20

#define LOAD_INERTIA    0.0
#define LOAD_TORQUE     2
#define VISCOUS_COEFF   0.0007

#define CURRENT_KP (6.39955)
#define CURRENT_KI (237.845)
#define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define SPEED_KP (0.0380362)
#define SPEED_KI (30.5565)
#define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)

/* Encoder QEP */
#define SYSTEM_QEP_PULSES_PER_REV  (10000)
#define SYSTEM_QEP_REV_PER_PULSE  (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2*M_PI * MOTOR_NUMBER_OF_POLE_PAIRS)
#ifdef _XCUBE1
    #define SYSTEM_QEP_CALIBRATED_ANGLE -2668 // for MOTOR2
#else
    // #define SYSTEM_QEP_CALIBRATED_ANGLE -2668 // for MOTOR2
    #define SYSTEM_QEP_CALIBRATED_ANGLE -976 // for MOTOR1 (w/ hall sensor) // -968 for MOTOR1
#endif

#if MACHINE_TYPE % 10 == 2

    #define ENABLE_COMMISSIONING FALSE

    #define ELECTRICAL_SPEED_FEEDBACK    nsoaf.xOmg // harnefors.omg_elec
    #define ELECTRICAL_POSITION_FEEDBACK nsoaf.theta_d // harnefors.theta_d

    /* CHEN 2020 NSO with Active Flux Concept */
    // #define NSOAF_SPMSM // use AP Error
    #define NSOAF_IPMSM // use only OE
    #define NSOAF_TL_P (1) // 1 for experimental starting // 4 for 1500 rpm // 2 for 800 rpm
    #define NSOAF_TL_I (20)
    #define NSOAF_TL_D (0)
    #define OUTPUT_ERROR_CLEST_GAIN_KP (1.0*0.1) // (0.5) // (2*5)
    #define OUTPUT_ERROR_CLEST_GAIN_KI (1.0*0.02) // 0.02 for 10 rpm // 0.1 for 40 rpm //(2.0) for 300 rpm
    /* Farza 2009 for EMMF */
    #define FARZA09_HGO_EEMF_VARTHETA 10
    #define FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE 10
    /* CJH EEMF AO Design */
    #define CJH_EEMF_K1 (100)
    #define CJH_EEMF_K2 (CJH_EEMF_K1*CJH_EEMF_K1*0.25) // see my TCST paper@(18)
    #define CJH_EEMF_GAMMA_OMEGA (5e6)
    /* Harnefors 2006 */
#endif

#if MACHINE_TYPE % 10 == 1
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

#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#define DATA_FILE_NAME "../dat/PMSM_NTUMotor-205-1000-10-1982.dat"
#endif
