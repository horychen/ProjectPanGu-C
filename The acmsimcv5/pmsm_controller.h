#ifndef PMSM_CONTROLLER_H
#define PMSM_CONTROLLER_H
#if MACHINE_TYPE == 2
typedef struct { 
    REAL  Ualpha; // Input: reference alpha-axis phase voltage
    REAL  Ubeta;  // Input: reference beta-axis phase voltage
    REAL  Ta;     // Output: reference phase-a switching function
    REAL  Tb;     // Output: reference phase-b switching function
    REAL  Tc;     // Output: reference phase-c switching function
    REAL CMPA[3]; // PWM compare register value original
    REAL CMPA_DBC[3]; // PWM compare register value dead-band compensated
} SVGENDQ;

typedef struct {
    // commands
    REAL cmd_position_rad;  // mechanical
    REAL cmd_speed_rpm;     // mechanical
    REAL cmd_omg_elec;      // electrical
    REAL cmd_rotor_flux_Wb;
    REAL cmd_psi; // this is not used for PMSM
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
} st_controller_inputs;
typedef struct {
    // controller strategy
    int ctrl_strategy;
    int go_sensorless;
    // field oriented control
    REAL cosT;
    REAL sinT;
    REAL cosT_compensated_1p5omegaTs;
    REAL sinT_compensated_1p5omegaTs;
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
    // State of Operation
    REAL Motor_or_Generator; // 1 for motoring, -1 for regenerating
} st_controller_states;
typedef struct {
    // voltage commands
    REAL uab_cmd[2];
    REAL uab_cmd_to_inverter[2]; // foc control output + inverter nonlinearity compensation
    REAL udq_cmd[2];
    REAL udq_cmd_to_inverter[2];
    // current commands
    REAL iab_cmd[2];
} st_controller_outputs;
typedef struct {
    // electrical 
    REAL R;
    REAL KE;
    REAL Ld;
    REAL Lq;
    REAL Lq_inv;
    REAL DeltaL; // Ld - Lq for IPMSM
    REAL KActive;
    REAL Rreq;
    // mechanical
    REAL npp;
    REAL npp_inv;
    REAL Js;
    REAL Js_inv;    
} st_pmsm_parameters;

#define MA_SEQUENCE_LENGTH         10 // 20 * CL_TS = window of moving average in seconds
#define MA_SEQUENCE_LENGTH_INVERSE 0.1 // 20 MA gives speed resolution of 3 rpm for 2500 ppr encoder
// #define MA_SEQUENCE_LENGTH         20 // 20 * CL_TS = window of moving average in seconds
// #define MA_SEQUENCE_LENGTH_INVERSE 0.05 // 20 MA gives speed resolution of 3 rpm for 2500 ppr encoder
// #define MA_SEQUENCE_LENGTH         80
// #define MA_SEQUENCE_LENGTH_INVERSE 0.0125
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
} st_enc; // encoder
typedef struct {
    REAL theta_excitation; // in rad
    REAL theta_d_elec_entered;
    Uint32 countEntered;
    int direction;
} st_psd; // phase sequence detection
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

    /* Chen 2021 */
    REAL I_plateau_Max;
    REAL I_plateau_Min;
    REAL I_plateau;
    REAL gamma_I_plateau;
    REAL V_plateau;
    REAL gamma_V_plateau;
    /**/
    REAL w6;
    REAL w12;
    REAL w18;
    REAL sig_a2;
    REAL sig_a3;
    REAL gamma_a2;
    REAL gamma_a3;
} st_InverterNonlinearity; // inverter
typedef struct {
    // ECAP support (i.e., the API)
    REAL terminal_DutyOnRatio[3];
    REAL terminal_voltage[3];
    REAL line_to_line_voltage[3]; // uv, vw, wu
    REAL pwm_time;
    REAL uab0[3];
    REAL dq[2];
    REAL dq_lpf[2];
    REAL dq_mismatch[2];
    // [Internal variables]
        // Run TI's example
        struct TestECapture {
            // REAL TSt1, TSt2, TSt3, TSt4;
            REAL Period1, Period2, Period3;
            REAL DutyOnTime1, DutyOffTime1, DutyOnTime2, DutyOffTime2;
        }ecapU, ecapV, ecapW;
        // On-demand ECAP decode with nonlinear filtering out disturbance. This mode gives raw ECAP values
        int flag_nonlinear_filtering;
        int flag_bad_U_capture;
        int flag_bad_V_capture;
        int flag_bad_W_capture;
        // On-interrupt ECAP decode. This mode gives ECAP values that are already free of disturbance.
        Uint32 good_capture_U[4];
        Uint32 good_capture_V[4];
        Uint32 good_capture_W[4];
        struct bad_values {
            Uint32 on1, off1, on2, off2;
        } u_ecap_bad1, v_ecap_bad1, w_ecap_bad1, u_ecap_bad2, v_ecap_bad2, w_ecap_bad2;
        // ISR counting
        Uint64 ECapIntCount[3];
        Uint64 ECapPassCount[3];
        // ISR nesting
        int password_isr_nesting; // set to 178 in EPWM1 ISR
        int decipher_password[3];
} st_capture; // eCapture
typedef struct {
    // ADC Offset
        // Automatic Offset Removing
        REAL Offset_Counter;
        REAL Offset_W;
        REAL Offset_V;
        REAL Offset_U;
        REAL Offset_Udc;
        REAL AD_scale_W;
        REAL AD_scale_V;
        REAL AD_scale_U;
        REAL AD_scale_VDC;
        // Raw
        REAL offsetU,offsetV,offsetW,offsetUDC; // ADC offset. U, V, W corresponds to ADCRESULT2, ADCRESULT3, ADCRESULT1.
    // DAC
        REAL dac_offset[8];
        REAL dac_time;
        REAL dac_watch[60];
        REAL dac_watch_stator_resistance;
    // To show that REAL type is not very accurate 64 missing by counting 1e-4 sec to 10 sec.
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
    // Mode Changing During Experiment Debug
        REAL OverwriteSpeedOutLimit; // = 2;
        REAL Overwrite_Voltage_DC_BUS; // = 180;
        int flag_overwite_voltage_dc_bus; // = FALSE;
        int flag_use_ecap_voltage; // = 0;
        int flag_experimental_initialized; 
        int FLAG_TUNING_CURRENT_SCALE_FACTOR; // for comm (special mode for calibrate current sensor channel gain)
        int flag_do_inverter_characteristics; // for comm
        int Seletc_exp_operation; // for exp
        int flag_auto_id_cmd; // for slow reversal
        int FLAG_INVERTER_NONLINEARITY_COMPENSATION;
    // Select Algorithm
        // int Select_algorithm;
        REAL omg_elec;
        REAL theta_d;
    //
    int16 sendCurrentCommandFlag;
    int16 sendSpeedCommandFlag;
    // 
    REAL ia;
    REAL dist_ua;
} st_global_variables; // globals

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

    /* Capture */
    st_capture *cap;

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
#define CAP     (*CTRL.cap)
#define G       (*CTRL.g)
    // extern st_InverterNonlinearity t_inv;
    // #define INV     t_inv


void init_experiment();
void init_CTRL();
void commissioning();
void controller(REAL rpm_speed_command, REAL set_iq_cmd, REAL set_id_cmd);
void allocate_CTRL(struct ControllerForExperiment *CTRL);

// void control(REAL speed_cmd, REAL speed_cmd_dot);

void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);


/* 逆变器非线性 */
/* 查表法 */
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist);
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut);
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist);

/* ParkSul2012 梯形波 */
// #define GAIN_THETA_TRAPEZOIDAL (20) // 20
void inverterNonlinearity_Initialization();
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal);
REAL lpf1(REAL x, REAL y_tminus1);
REAL shift2pi(REAL thetaA);

void Modified_ParkSul_Compensation(void);
void Online_PAA_Based_Compensation(void);

/* Main */
void main_inverter_voltage_command(int bool_use_iab_cmd);

#endif
#endif
