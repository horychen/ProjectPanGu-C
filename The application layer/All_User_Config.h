/*
*  Created on: 2021Äê1ÔÂ15ÈÕ
 *      Author: yuanxin and jiahao
 *       */
#ifndef ALL_USER_CONFIG_H
#define ALL_USER_CONFIG_H

/*  SYSTEM Configuration -----------------------------------------------------------------------------------*/
    #define NUMBER_OF_DSP_CORES 2  // 1 or 2
    #define SYSTEM_PROGRAM_MODE 223  //223 for CJHMainISR //0 stands for MainISR ,1 stands for Motor_Parameter_Compute_ISR
    #define SYSTEM_QEP_ROTOR_ANGLE              1      //  MEASURE_QEP_D_AXIS:0;       REAL_QEP_ROTOR_ANGLE:1       VIRTUAL_QEP_ROTOR_ANGLE:2;
    #define SYSTEM_CURRENT_LOOP                 0 //    1 Means current loop;   0 Means speed loop;

/*  USER Configuration -----------------------------------------------------------------------------------*/
    //MOTOR parameter page
    #define MOTOR_POLE                          4              // motor 4
    #define MOTOR_RS                            1.1            // motor_RS ohm
    #define MOTOR_LS                            5.7             // motor_LS mH
    #define MOTOR_FLUXR                         0.092            // motor_fluxr wb
    #define MOTOR_RATED_SPEED                   3000              // motor_ speed 2500r/min
    #define MOTOR_INERTIA                       0.00162          // motor INERTIA kg/m3
    //INVERTER parameter page
    #define IGBT_PWM_TON                        0.038                 // IGBT TURN ON-DELAY(us)
    #define IGBT_PWM_TOFF                       0.07                  // IGBT TURN OFF-DELAY(us)
    #define IGBT_VCE0_VOLTAGE                   0                    // IGBT VCE0 INITIAL VALUE (V)
    #define IGBT_DIODE0_VOLTAGE                 0                  // IGBT Diode Forward Voltage INITIAL VALUE(V)
    //PI CONFIGURATION
    #define SYSTEM_PI_SPD_KP                 0.4               //KP     SPEED
    #define SYSTEM_PI_SPD_KI                 10.01               //Ki     SPEED
    #define SYSTEM_PI_SPD_UMAX                6              //1-1A   Saturate
    #define SYSTEM_PI_IQ_UMAX                  180             //30V   Saturate
    #define SYSTEM_PI_ID_UMAX                  180              //30V   Saturate

/* Code Selection */
    //program mode selection parameter_compute_program and main_program
    #if SYSTEM_PROGRAM_MODE==0
        #define SYSTEM_PROGRAM                MainISR     // MainISR
        #define SYSTEM_PWM_FREQUENCY               10     // KHz  Define the ISR timr  20K
    #elif SYSTEM_PROGRAM_MODE==1
        #define SYSTEM_PROGRAM                Motor_Parameter_ISR //   Motor_Parameter_Compute_ISR
        #define SYSTEM_PWM_FREQUENCY               10             // KHz  Define the ISR timr  10K
    #elif SYSTEM_PROGRAM_MODE==223
        #define SYSTEM_PROGRAM                CJHMainISR 
        #define SYSTEM_PWM_FREQUENCY               10    
    #endif

#endif
