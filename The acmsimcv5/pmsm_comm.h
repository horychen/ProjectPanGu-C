#ifndef PMSM_COMMISSIONING_H
#define PMSM_COMMISSIONING_H
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE
#if ENABLE_COMMISSIONING

#define COMM_IV_SIZE_R1 100 // 300
#define COMM_IV_SIZE_L1 30
struct CommissioningDataStruct{

    REAL timebase;
    int32 counterEntered;
    int16 i;

    REAL R;
    REAL L;
    REAL R3;
    REAL L3;
    REAL KE;
    REAL Js; // shaft inertia

    // R
    REAL current_command;
    REAL current_sum;
    REAL voltage_sum;
    int32 counterSS;
    int16 bool_collecting;
    REAL i_data_R1[COMM_IV_SIZE_R1];
    REAL v_data_R1[COMM_IV_SIZE_R1];
    REAL i_data_L1[COMM_IV_SIZE_L1];
    REAL t_data_L1[COMM_IV_SIZE_L1];
    REAL inverter_voltage_drop;

    // L
    REAL last_voltage_command;
    REAL id_prev;
    REAL iq_prev;

    // Js
    int16 number_of_repeats_Js;
};
extern struct CommissioningDataStruct COMM;
extern int16 bool_single_phase_excitation;
extern int16 bool_comm_status;

void COMM_PI_tuning(REAL LL, REAL RR, REAL BW_current, REAL delta, REAL JJ, REAL KE, REAL npp);

void init_COMM();
void COMM_resistanceId(REAL id_fb, REAL iq_fb);

void COMM_inductanceId(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver2(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver3(REAL id_fb, REAL iq_fb);

void COMM_PMFluxId(REAL id_fb, REAL iq_fb, REAL omg_elec_fb);

void COMM_inertiaId(REAL id_fb, REAL iq_fb, REAL cosPark, REAL sinPark, REAL omg_elec_fb);
void COMM_end(REAL id_fb, REAL iq_fb);

// Main Procedure
void StepByStepCommissioning();

#endif

// #define PMSM_NUMBER_OF_POLE_PAIRS 4
// #define PMSM_RESISTANCE 0.95
// #define PMSM_D_AXIS_INDUCTANCE 0.0041
// #define PMSM_Q_AXIS_INDUCTANCE 0.0041
// #define PMSM_SHAFT_INERTIA (0.035*1e-4)
// #define PMSM_RATED_CURRENT_RMS 3.5
// #define PMSM_RATED_POWER_WATT  100
// #define PMSM_RATED_SPEED_RPM   3000

//             /* Start *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

//             // 使用测量得到的电流、转速、位置等信息，生成电压指令CTRL.ual，CTRL.ube
//             // commissioning();
//             if(bool_comm_status == 0){
//                 // 初始化
//                 COMM_init();
//             }
//             else if(bool_comm_status == 1){
//                 // 电阻辨识
//                 COMM_resistanceId();
//             }else if(bool_comm_status == 2){
//                 // 电阻辨识
//                 COMM_inductanceId();
//             }else{
                
//             }

//             /* End *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/
#endif
#endif
