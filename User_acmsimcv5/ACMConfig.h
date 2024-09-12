#ifndef ACMCONFIG_H
#define ACMCONFIG_H

/* User */
#include "super_config.h"
#include "main_switch.h"




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
	#define IM_FLUX_COMMAND_DC_PART     d_sim.init.KE // 1.3593784874408608
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
    // #define AFE_USED FE.clfe4PMSM
    #define AFE_USED FE.no_sat
    // #define AFE_USED FE.huwu
    // #define AFE_USED FE.htz // this is for ESO speed estimation
    // #define AFE_USED FE.picorr // this is for ESO speed estimation

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

#elif /* Induction Motor Observer */ MACHINE_TYPE % 10 == 1
    // Marino05 调参 /// default: (17143), (2700.0), (1000), (1), (0)
    #define GAMMA_INV_xTL 17142.85714285714
    #define LAMBDA_INV_xOmg 10000 // 2700.0 is too large, leading to unstable flux amplitude contorl
    #define DELTA_INV_alpha (0*500) // 1000
    #define xAlpha_LAW_TERM_D 1 // regressor is commanded d-axis rotor current, and error is d-axis flux control error.
    #define xAlpha_LAW_TERM_Q 0 // regressor is commanded q-axis stator current, and error is q-axis flux control error.
    // 磁链反馈用谁 /// "htz",,ohtani",picorr",lascu",clest",harnefors
    #define IFE FE.picorr
    // #define IFE FE.htz
    #define FLUX_FEEDBACK_ALPHA         IFE.psi_2[0]
    #define FLUX_FEEDBACK_BETA          IFE.psi_2[1]
    #define OFFSET_COMPENSATION_ALPHA   IFE.u_offset[0]
    #define OFFSET_COMPENSATION_BETA    IFE.u_offset[1]

    // Ohtani 磁链观测系数配置/// default: 5
    // Ohtani 建议取值和转子时间常数相等
    #define GAIN_OHTANI (5)
    #define VM_OHTANI_CORRECTION_GAIN_P (5)
    /* B *//// default: P=5, I=2.5
    #define VM_PROPOSED_PI_CORRECTION_GAIN_P 1// 
    #define VM_PROPOSED_PI_CORRECTION_GAIN_I 2.5//80000//2.5 //2  // (2.5)
    /* No Saturation */
    #define VM_NOSAT_PI_CORRECTION_GAIN_P 7// 难调
    #define VM_NOSAT_PI_CORRECTION_GAIN_I 0//80000//2.5 //2  // (2.5)
    /* Saturation_time_Without_Limiting */
    #define STWL_GAIN_KP 
    #define STWL_GAIN_KI 
    /* C *//// default: P=0.125*5, I=0.125*2.5, KCM=0
    #define OUTPUT_ERROR_CLEST_GAIN_KP (0.04)
    #define OUTPUT_ERROR_CLEST_GAIN_KI (0.5)
    #define OUTPUT_ERROR_CLEST_GAIN_KCM (0.0002*0.8)
    /* Closed Loop flux estimator for PMSM*/
    #define OUTPUT_ERROR_CLEF4PMSM_GAIN_KP (0.04)
    #define OUTPUT_ERROR_CLEF4PMSM_GAIN_KI (0.5)
    /* Holtz 2002 */// default: 20
    #define HOLTZ_2002_GAIN_OFFSET 20 //1 // 20 is too large, causing unstable control during reversal
    /* Harnefors SCVM 2003 */// default: 2
    #define GAIN_HARNEFORS_LAMBDA 2

	// #if 1
        #define IM_ELECTRICAL_SPEED_FEEDBACK    marino.xOmg // (*CTRL).i->omg_elec
        #define IM_ELECTRICAL_POSITION_FEEDBACK marino.xRho // (*CTRL).i->theta_d_elec
	// #endif
#endif

/* 控制策略 */
	// #define NULL_D_AXIS_CURRENT_CONTROL -1
	// #define MTPA -2 // not supported
// #define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
//     #define MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL 11
// #define CONTROL_STRATEGY MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL
// #define NUMBER_OF_STEPS 25000

    #define DOWN_SAMPLE 1
    #define USE_QEP_RAW FALSE

    // #define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT FALSE
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

#define MOTOR_RATED_POWER_WATT 750
#define MOTOR_RATED_SPEED_RPM 3000
#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE d_sim.init.KE
    #define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
    #define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (d_sim.init.IN*1.414) )
    #define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / d_sim.init.npp )
    #define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/d_sim.init.npp/2/3.1415926*60) )
    #define SPEED_LOOP_LIMIT_NEWTON_METER (1.0*MOTOR_RATED_TORQUE)


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
