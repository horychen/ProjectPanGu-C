#ifndef PMSM_CONTROLLER_H
#define PMSM_CONTROLLER_H
#if MACHINE_TYPE == 2
typedef struct { 
    float  Ualpha; // Input: reference alpha-axis phase voltage
    float  Ubeta;  // Input: reference beta-axis phase voltage
    float  Ta;     // Output: reference phase-a switching function
    float  Tb;     // Output: reference phase-b switching function
    float  Tc;     // Output: reference phase-c switching function
} SVGENDQ;

typedef struct {
    // commands
    REAL cmd_position_rad;  // mechanical
    REAL cmd_speed_rpm;     // mechanical
    REAL cmd_rotor_flux_Wb;
    REAL idq_cmd[2];
    REAL Tem_cmd;
    // feedback
    REAL rpm;
    REAL omg_elec;
    REAL theta_d_elec;
    REAL theta_d_elec_previous;
    REAL iab[2];
    REAL idq[2];
    REAL psi_mu[2];
    REAL Tem;
    // ECAP support
    REAL ecap_terminal_DutyOnRatio[3];
    REAL ecap_terminal_voltage[3];
    REAL ecap_line_to_line_voltage[3]; // uv, vw, wu
    REAL ecap_pwm_time;
    REAL ecap_uab0[3];
    REAL ecap_dq[2];
    REAL ecap_dq_lpf[2];
    REAL ecap_dq_mismatch[2];
} st_controller_inputs;
typedef struct {
    // controller strategy
    int ctrl_strategy;
    int go_sensorless;
    // field oriented control
    REAL cosT;
    REAL sinT;
    REAL omega_syn;
    // states
    st_pid_regulator *iM;
    st_pid_regulator *iT;
    st_pid_regulator *pos;
    st_pid_regulator *spd;
    int the_vc_count;
    // Status of Detection
    int PSD_Done;
    int IPD_Done;
} st_controller_states;
typedef struct {
    // voltage commands
    REAL uab_cmd[2];
    REAL uab_cmd_to_inverter[2]; // foc control output + inverter nonlinearity compensation
    REAL udq_cmd[2];
    REAL udq_cmd_to_inverter[2];
} st_controller_outputs;
typedef struct {
    // electrical 
    REAL R;
    REAL KE;
    REAL Ld;
    REAL Lq;
    REAL Lq_inv;
    REAL DeltaL; // Ld - Lq for IPMSM
    // mechanical
    REAL npp;
    REAL npp_inv;
    REAL Js;
    REAL Js_inv;    
} st_pmsm_parameters;
// #define MA_SEQUENCE_LENGTH         20 // 20 * CL_TS = window of moving average in seconds
// #define MA_SEQUENCE_LENGTH_INVERSE 0.05 // 20 MA gives speed resolution of 3 rpm for 2500 ppr encoder
#define MA_SEQUENCE_LENGTH         80
#define MA_SEQUENCE_LENGTH_INVERSE 0.0125
typedef struct {
    // Moving Average for speed calculation
    REAL MA_qepPosCnt[MA_SEQUENCE_LENGTH];
    REAL sum_qepPosCnt;
    unsigned int cursor; // cursor is between 0~79, and it decides which element in MA queue should be kicked out.
    // for IPD
    REAL theta_d__state;
    REAL theta_d_offset;
    REAL excitation_angle_deg; // in deg
    // in experiment
    int32 encoder_abs_cnt;
    int32 encoder_abs_cnt_previous;
    int32 encoder_incremental_cnt;
    int16 encoder_direction;
    int32 OffsetCountBetweenIndexAndUPhaseAxis;
    // output
    REAL rpm;
    REAL omg_elec;
    REAL theta_d_elec;
} st_enc;
typedef struct {
    REAL theta_excitation; // in rad
    REAL theta_d_elec_entered;
    Uint32 countEntered;
    int direction;
} st_psd;
typedef struct {
    REAL ia;
    REAL ib;
    REAL ic;
    REAL dist_ua;
    REAL dist_ub;
    REAL dist_uc;

    /* Adaptive Vsat */
    REAL gain_Vsat;

    /* DOB for stationary voltage */
    REAL iab_lpf[2];
    REAL iab_hpf[2];
    REAL filter_pole;
    REAL uab_DOB[2];

    /* Park.Sul-2012 */
    REAL gamma_theta_trapezoidal;
    REAL Vsat; // = 4*td*Udc/(pi*Ts)

    REAL thetaA;
    REAL cos_thetaA;
    REAL sin_thetaA;

    REAL u_comp[3]; // phase ABC quantities of compensation
    REAL ual_comp; // alpha-beta frame quantities of compensation
    REAL ube_comp;
    REAL uDcomp_atA; // Compensation Voltage's D-component at phase A frame
    REAL uQcomp_atA;

    REAL iD_atA; // D-component at phase A frame
    REAL iQ_atA;
    REAL I5_plus_I7; // Magnitude of 6th harmonic in syn. speed frame
    REAL I5_plus_I7_LPF; // lpf'd version of I5_plus_I7
    REAL I11_plus_I13;
    REAL I11_plus_I13_LPF;
    REAL I17_plus_I19;
    REAL I17_plus_I19_LPF;
    REAL theta_trapezoidal; // theta_t defined by Park.Sul-2012
} st_InverterNonlinearity;
typedef struct {
    // ADC Offset
        // Automatic Offset Removing
        REAL Offset_Counter;
        REAL Offset_W;
        REAL Offset_V;
        REAL Offset_U;
        REAL Offset_Udc;
        // Raw
        REAL offsetU,offsetV,offsetW,offsetUDC; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
    // DAC
    REAL dac_offset[8];
    REAL dac_time;
    REAL dac_watch[60];
    REAL dac_watch_stator_resistance;
    // REAL type is not very accurate 64 missing by counting 1e-4 sec to 10 sec.
    Uint32  test_integer;
    REAL    test_float;
    // Raw measurment for easy access
    REAL Current_U, Current_V, Current_W, Current_Not_Used, Voltage_DC_BUS;
    // Mode Changing During Experiment
    int FLAG_ENABLE_PWM_OUTPUT; // 电机模式标志位
    Uint16 Rotor_angle_selection; // delete?
    REAL Set_manual_current_iq,Set_manual_current_id,Set_manual_rpm;
    int DAC_MAX5307_FLAG; // for single core case
    int AD_offset_flag2;
    int bool_comm_status;
    // Mode Changing During Experiment Debug
    REAL OverwriteSpeedOutLimit; // = 2;
    REAL Overwrite_Voltage_DC_BUS; // = 180;
    int flag_overwite_voltage_dc_bus; // = FALSE;
    int flag_use_ecap_voltage; // = 0;
} st_global_variables;
struct ControllerForExperiment{

    /* Basic quantities */
    REAL timebase;

    /* Machine parameters */
    st_pmsm_parameters *motor;

    /* Peripheral configurations */
    st_enc *enc;
    st_psd *psd; // phase sequence detection

    /* Inverter */
    st_InverterNonlinearity *inv;
    SVGENDQ svgen1;

    /* Console */
    st_global_variables *g;

    /* Black Box Model | Controller quantities */
    st_controller_inputs  *I;
    st_controller_states  *S;
    st_controller_outputs *O;
};

extern struct ControllerForExperiment CTRL;
#define MOTOR   (*CTRL.motor)
#define ENC     (*CTRL.enc)
#define PSD     (*CTRL.psd)
#define IN      (*CTRL.I)
#define STATE   (*CTRL.S)
#define OUT     (*CTRL.O)
#define INV     (*CTRL.inv)
#define G       (*CTRL.g)
    // extern st_InverterNonlinearity t_inv;
    // #define INV     t_inv


void experiment_init();
void CTRL_init();
void commissioning();
void controller(REAL rpm_speed_command, REAL set_iq_cmd, REAL set_id_cmd);
void allocate_CTRL(struct ControllerForExperiment *CTRL);

// void control(REAL speed_cmd, REAL speed_cmd_dot);

void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);


/* 逆变器非线性 */
/* 查表法 */
void inverter_voltage_command();
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut);
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist);

/* ParkSul2012 梯形波 */
// #define GAIN_THETA_TRAPEZOIDAL (20) // 20
void inverterNonlinearity_Initialization();
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal);
REAL lpf1(REAL x, REAL y_tminus1);
REAL shift2pi(REAL thetaA);

void Modified_ParkSul_Compensation(void);

#endif
#endif
