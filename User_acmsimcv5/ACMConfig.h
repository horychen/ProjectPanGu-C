#ifndef ACMCONFIG_H
#define ACMCONFIG_H

/* User */
#include <user_defined_functions.h>
#include <super_config.h>

#define MODE_SELECT_PWM_DIRECT 1
#define MODE_SELECT_VOLTAGE_OPEN_LOOP 11
#define MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE 2
#define MODE_SELECT_FOC 3
#define MODE_SELECT_FOC_SENSORLESS 31
#define MODE_SELECT_VELOCITY_LOOP 4
#define MODE_SELECT_VELOCITY_LOOP_SENSORLESS 41
#define MODE_SELECT_TESTING_SENSORLESS 42
#define MODE_SELECT_VELOCITY_LOOP_WC_TUNER 43
#define MODE_SELECT_POSITION_LOOP 5
#define MODE_SELECT_COMMISSIONING 9


/* 经常要修改的 */
// #define INVERTER_NONLINEARITY_COMPENSATION_INIT 0 // 5（9月1日及以前峣杰实验一直用的5） // 4 // 1:ParkSul12, 2:Sigmoid, 3:LUT(Obsolete), 4:LUT(by index), 5 Slessinv-a2a3Model
// #define INVERTER_NONLINEARITY                   0 // 4 // 1:ModelSul96, 2:ModelExpSigmoid, 3: ModelExpLUT, 4:LUT(by index)
// #define SENSORLESS_CONTROL FALSE
// #define SENSORLESS_CONTROL_HFSI FALSE

/* ParkSul2012 梯形波 */
#define GAIN_THETA_TRAPEZOIDAL (40) //(500) // 20

/* 电机类型 */ //（TODO：饱和模型里面用的还是 IM.rr 而不是 IM.rreq）
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 1

	// 参数误差
		#define MISMATCH_R    100.0
		#define MISMATCH_RREQ 100.0
		#define MISMATCH_LD   100.0
		#define MISMATCH_LQ   100.0
		#define MISMATCH_KE   100.0
        #define MISMATCH_LMU  100.0


	// 磁链给定
	#define IM_MAGNETIZING_INDUCTANCE   (d_sim.init.Ld - d_sim.init.Lq)
	#define IM_FLUX_COMMAND_DC_PART     1.3593784874408608
	#define IM_FLUX_COMMAND_SINE_PART   0.0
	#define IM_FLUX_COMMAND_SINE_HERZ   10

#if MACHINE_TYPE % 10 == 2
	#define CORRECTION_4_SHARED_FLUX_EST INIT_KE
    #define U_MOTOR_KE                   INIT_KE
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
        /*As we use (*CTRL).o->iab_cmd for look up, now dead-time compensation during ENABLE_COMMISSIONING is not active*/
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
            #define ELECTRICAL_SPEED_FEEDBACK    (-esoaf.xOmg) // 薄片电机实验正iq产生负转速
            #define ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d
        #else
            // #define ELECTRICAL_SPEED_FEEDBACK    G.omg_elec
            // #define ELECTRICAL_POSITION_FEEDBACK G.theta_d

            // #define ELECTRICAL_SPEED_FEEDBACK    parksul.xOmg
            // #define ELECTRICAL_POSITION_FEEDBACK parksul.theta_d

            // #define ELECTRICAL_SPEED_FEEDBACK    qiaoxia.xOmg
            // #define ELECTRICAL_POSITION_FEEDBACK qiaoxia.theta_d

            #define ELECTRICAL_SPEED_FEEDBACK    (*CTRL).i->omg_elec
            #define ELECTRICAL_POSITION_FEEDBACK (*CTRL).i->theta_d_elec
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
    // Marino05 调参 /// default: (17143), (2700.0), (1000), (1), (0)
    #define GAMMA_INV_xTL 17142.85714285714
    #define LAMBDA_INV_xOmg 1000 // 2700.0 is too large, leading to unstable flux amplitude contorl
    #define DELTA_INV_alpha (0*500) // 1000
    #define xAlpha_LAW_TERM_D 1 // regressor is commanded d-axis rotor current, and error is d-axis flux control error.
    #define xAlpha_LAW_TERM_Q 0 // regressor is commanded q-axis stator current, and error is q-axis flux control error.
    // 磁链反馈用谁 /// "htz",,ohtani",picorr",lascu",clest",harnefors
    // #define IFE FE.picorr
    #define IFE FE.htz
    #define FLUX_FEEDBACK_ALPHA         IFE.psi_2[0]
    #define FLUX_FEEDBACK_BETA          IFE.psi_2[1]
    #define OFFSET_COMPENSATION_ALPHA   IFE.u_offset[0]
    #define OFFSET_COMPENSATION_BETA    IFE.u_offset[1]

    // Ohtani 磁链观测系数配置/// default: 5
    // Ohtani 建议取值和转子时间常数相等
    #define GAIN_OHTANI (5)
    #define VM_OHTANI_CORRECTION_GAIN_P (5)
    /* B *//// default: P=5, I=2.5
    #define VM_PROPOSED_PI_CORRECTION_GAIN_P 50 // 20暂态能跟上但是又滞后，200后面暂态就跟不上了  //10 // (5)
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
        #define ELECTRICAL_SPEED_FEEDBACK    marino.xOmg // (*CTRL).i->omg_elec
        #define ELECTRICAL_POSITION_FEEDBACK marino.xRho // (*CTRL).i->theta_d_elec
	// #endif
#endif

/* 控制策略 */
	#define NULL_D_AXIS_CURRENT_CONTROL -1
	#define MTPA -2 // not supported
// #define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
    #define MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL 11
#define CONTROL_STRATEGY MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL
// #define NUMBER_OF_STEPS 25000
    #define DOWN_SAMPLE 1
    #define USE_QEP_RAW FALSE
    #define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT FALSE
    #define SATURATED_MAGNETIC_CIRCUIT FALSE
#define CL_TS          (d_sim.sim.CL_TS)
#define CL_TS_INVERSE  (1.0 / CL_TS)
#define VL_TS          (d_sim.FOC.VL_EXE_PER_CL_EXE*CL_TS)
#define PL_TS          VL_TS
#define SPEED_LOOP_CEILING (d_sim.FOC.VL_EXE_PER_CL_EXE)
    // #define TS_UPSAMPLING_FREQ_EXE (CL_TS / MACHINE_TS)
    // #define TS_UPSAMPLING_FREQ_EXE_INVERSE (1.0/TS_UPSAMPLING_FREQ_EXE)
    #define MACHINE_TS         (CL_TS*d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD)
    #define MACHINE_TS_INVERSE (1.0/MACHINE_TS)

// #define LOAD_INERTIA    0.0
#define LOAD_TORQUE     0.2 // 0.0
#define VISCOUS_COEFF   0.0007 // 0.0007


// #if APPLY_WCTUNER
//     #define CURRENT_KP (0.3063)   // (1.19)    // (0.6)  // (0.84)   // (0.84)   // (0.84)
//     #define CURRENT_KI (231.578947368421) // (194.74) // (194.74)  // (194.74) // (194.74) // (194.74)
//     #define SPEED_KP   (1.3099)   // (0.19)   // (2.59)  // (0.81)   // (0.41)   // (0.36)
//     #define SPEED_KI   (763.5600)   // (3.93)    // (13.96)  // (11.00)  // (2.75)   // (19.55)
//     #define SPEED_KFB  (2.36)
// #else
//     #define CURRENT_KP (0.9550441666912972)   // (1.19)    // (0.6)  // (0.84)   // (0.84)   // (0.84)
//     #define CURRENT_KI (231.578947368421) // (194.74) // (194.74)  // (194.74) // (194.74) // (194.74)
//     #define SPEED_KP   (0.5896379954298365)   // (0.19)   // (2.59)  // (0.81)   // (0.41)   // (0.36)
//     #define SPEED_KI   (2.0106192982974678)   // (3.93)    // (13.96)  // (11.00)  // (2.75)   // (19.55)
//     #define SPEED_KFB  (0)
// #endif



// 速度环 increase to 3 times because of the bug in dynamics clamping
#define VL_SERIES_KI_CODE (VL_SERIES_KI*VL_SERIES_KP*VL_TS)
#define SPEED_LOOP_LIMIT_AMPERE       (1 * d_sim.init.npp) // (1.0*1.414*INIT_IN)


    // #define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
    // #define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (INIT_IN*1.414) )
    // #define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / INIT_NPP )
    // #define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/INIT_NPP/2/3.1415926*60) )
    // #define SPEED_LOOP_LIMIT_NEWTON_METER (1.0*MOTOR_RATED_TORQUE)


/* 指令类型 */
    #define EXCITATION_POSITION 0
    #define EXCITATION_SWEEP_FREQUENCY 2
    #define EXCITATION_VELOCITY 1
#define EXCITATION_TYPE (1)

/* Sweep Frequency */
#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#endif
