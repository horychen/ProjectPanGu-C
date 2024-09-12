#include "ACMSim.h"

// 定义顶级结构体（指针的集合）
int axisCnt = 0;
struct ControllerForExperiment CTRL_1;
struct ControllerForExperiment *CTRL;
struct DebugExperiment debug_1;
struct DebugExperiment *debug;
ST_D_SIM d_sim;
REAL one_over_six = 1.0/6.0;

// 定义内存空间（结构体）
st_motor_parameters     t_motor_1={0};
st_enc                  t_enc_1={0};
st_psd                  t_psd_1={0};
st_controller_inputs    t_I_1={0};
st_controller_states    t_S_1={0};
st_controller_outputs   t_O_1={0};
st_InverterNonlinearity t_inv_1={0}; // Because of the sv_count bug, I cannot declare t_inv in this .c file. // extern st_InverterNonlinearity t_inv; 
st_capture              t_cap_1={0};
st_global_variables     t_g_1={0};

st_pid_regulator _PID_iD_1       = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_iQ_1       = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_Speed_1    = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_Position_1 = st_pid_regulator_DEFAULTS;

#if PC_SIMULATION == FALSE
    //#pragma DATA_SECTION(CTRL     ,"MYGLOBALS"); // 陈嘉豪是傻逼
    #pragma DATA_SECTION(CTRL_1       ,"MYGLOBALS_1"); // FUCK! 叶明是天才！ 2024-03-12
    #pragma DATA_SECTION(t_motor_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_enc_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_psd_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_I_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_S_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_O_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_inv_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_cap_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_g_1        ,"MYGLOBALS_1");
    // #pragma DATA_SECTION(_PID_iX_1    ,"MYGLOBALS_1");
    // #pragma DATA_SECTION(_PID_iY_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_iD_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_iQ_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_Position_1   ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_Speed_1   ,"MYGLOBALS_1");

    #if NUMBER_OF_AXES == 2
        // 注意，一定要先extern再pragma？？？
        #pragma DATA_SECTION(CTRL_2     ,"MYGLOBALS_2"); // FUCK! 叶明是天才！ 2024-03-12
        #pragma DATA_SECTION(t_motor_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_enc_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_psd_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_I_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_S_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_O_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_inv_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_cap_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_g_2        ,"MYGLOBALS_2");
        // #pragma DATA_SECTION(_PID_iX_2    ,"MYGLOBALS_2");
        // #pragma DATA_SECTION(_PID_iY_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_iD_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_iQ_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_Position_2   ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_Speed_2   ,"MYGLOBALS_2");
        struct ControllerForExperiment CTRL_2;
        struct DebugExperiment debug_2;

        st_motor_parameters     t_motor_2={0};
        st_enc                  t_enc_2={0};
        st_psd                  t_psd_2={0};
        st_controller_inputs    t_I_2={0};
        st_controller_states    t_S_2={0};
        st_controller_outputs   t_O_2={0};
        st_InverterNonlinearity t_inv_2={0}; // Because of the sv_count bug, I cannot declare t_inv in this .c file. // extern st_InverterNonlinearity t_inv;
        st_capture              t_cap_2={0};
        st_global_variables     t_g_2={0};

        st_pid_regulator _PID_iD_2       = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_iQ_2       = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_Speed_2    = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_Position_2 = st_pid_regulator_DEFAULTS;

        // st_PIDController _PID_iX_2 = {
        //                     SUSPENSION_PID_KP, SUSPENSION_PID_KI, SUSPENSION_PID_KD,
        //                     SUSPENSION_PID_TAU,
        //                     SUSPENSION_PID_OUT_LIMIT,
        //                     SUSPENSION_PID_INT_LIMIT, CL_TS };
        // st_PIDController _PID_iY_2 = {
        //                     SUSPENSION_PID_KP, SUSPENSION_PID_KI, SUSPENSION_PID_KD,
        //                     SUSPENSION_PID_TAU,
        //                     SUSPENSION_PID_OUT_LIMIT,
        //                     SUSPENSION_PID_INT_LIMIT, CL_TS };
    #endif
#endif

// 初始化顶级结构体指针，指向定义好的内存空间
void allocate_CTRL(struct ControllerForExperiment *p){
    /* My attemp to use calloc with TI's compiler in CCS has failed. */
        // p->motor = calloc(1,sizeof(st_pmsm_parameters)); // 意思是，一个，st_pmsm_parameters那么大的空间
        // p->I = calloc(1,sizeof(st_controller_inputs));
        // p->S = calloc(1,sizeof(st_controller_states));
        // p->O = calloc(1,sizeof(st_controller_outputs));

    if(axisCnt==0){
        p->motor = &t_motor_1;
        p->enc   = &t_enc_1;
        p->psd   = &t_psd_1;
        p->i     = &t_I_1;
        p->s     = &t_S_1;
        p->o     = &t_O_1;
        p->inv   = &t_inv_1;
        p->cap   = &t_cap_1;
        p->g     = &t_g_1;

        p->s->iD  = &_PID_iD_1;
        p->s->iQ  = &_PID_iQ_1;
        p->s->Speed = &_PID_Speed_1;
        p->s->Position = &_PID_Position_1;


        // p->S->iX = &_PID_iX_1;
        // p->S->iY = &_PID_iY_1;
    }

    #if PC_SIMULATION == FALSE
        if(axisCnt==1){
            #if NUMBER_OF_AXES == 2
                p->motor = &t_motor_2;
                p->enc   = &t_enc_2;
                p->psd   = &t_psd_2;
                p->i     = &t_I_2;
                p->s     = &t_S_2;
                p->o     = &t_O_2;
                p->inv   = &t_inv_2;
                p->cap   = &t_cap_2;
                p->g     = &t_g_2;

                p->s->iD  = &_PID_iD_2;
                p->s->iQ  = &_PID_iQ_2;
                p->s->Speed = &_PID_Speed_2;
                p->s->Position = &_PID_Position_2;

                // p->S->iX = &_PID_iX_2;
                // p->S->iY = &_PID_iY_2;
            #endif
        }
    #endif
}





// Global watch variables
struct GlobalWatch watch;

/* Structs for Algorithm */

    struct ObserverForExperiment OBSV;
    struct SharedFluxEstimatorForExperiment FE;

    // 游离在顶级之外的算法结构体
    // struct RK4_DATA rk4;
    // struct Harnefors2006 harnefors={0};
    // struct CJH_EEMF_AO_Design OBSV.cjheemf={0};
    // struct FARZA09_EEMF_HGO OBSV.hgo4eemf={0};
    // struct Chen20_NSO_AF OBSV.nsoaf={0};

    // 游离在顶级之外的算法结构体
    // struct RK4_DATA rk4;
    struct Marino2005 marino={0};

    struct Variables_SimulatedVM                         simvm      ={0};
    // struct Variables_Ohtani1992                          ohtani     ={0};
    // struct Variables_HuWu1998                            huwu       ={0};
    // struct Variables_HoltzQuan2002                       holtz02    ={0};
    // struct Variables_Holtz2003                           htz        ={0};
    // struct Variables_Harnefors2003_SCVM                  harnefors  ={0};
    // struct Variables_LascuAndreescus2006                 lascu      ={0};
    // struct Variables_Stojic2015                          stojic     ={0};
    // struct Variables_fluxModulusEstimator                fme        ={0};
    // struct Variables_ExactCompensationMethod             exact      ={0};
    // struct Variables_ProposedxRhoFramePICorrectionMethod picorr     ={0};
    // struct Variables_ClosedLoopFluxEstimator             clest      ={0};


// struct eQEP_Variables qep={0};

void init_experiment(){

    init_CTRL_Part1();
    init_CTRL_Part2(); // 控制器结构体初始化
    init_FE();  // flux estimator
    rk4_init(); // 龙格库塔法结构体初始化
    // observer_init();
    init_pmsm_observers(); // 永磁电机观测器初始化
}
void init_CTRL_Part1(){
    // 我们在初始化 debug 全局结构体的时候，需要用到一部分 CTRL 中的电机参数，所以要先把这一部分提前初始化。
    allocate_CTRL(CTRL);

    /* Basic quantities */
    (*CTRL).timebase = 0.0;

    /* Machine parameters */
    // elec
    (*CTRL).motor->R  = d_sim.init.R;
    (*CTRL).motor->KE = d_sim.init.KE; // * (0.1/0.1342); // 【实验编号：】
    (*CTRL).motor->Ld = d_sim.init.Ld;
    (*CTRL).motor->Lq = d_sim.init.Lq;
    (*CTRL).motor->Lq_inv = 1.0 / (*CTRL).motor->Lq;
    (*CTRL).motor->DeltaL = (*CTRL).motor->Ld - (*CTRL).motor->Lq; // for IPMSM
    (*CTRL).motor->KActive = (*CTRL).motor->KE;                    // TODO:

    (*CTRL).motor->Rreq = d_sim.init.Rreq;

    // mech
    (*CTRL).motor->npp = d_sim.init.npp;
    (*CTRL).motor->npp_inv = 1.0 / (*CTRL).motor->npp;
    (*CTRL).motor->Js = d_sim.init.Js;
    (*CTRL).motor->Js_inv = 1.0 / (*CTRL).motor->Js;
}
void init_CTRL_Part2(){

    // /* Peripheral configurations */

    /* Inverter */
    (*CTRL).inv->filter_pole = 3000 * 2 * M_PI;
    inverterNonlinearity_Initialization();
    G.FLAG_INVERTER_NONLINEARITY_COMPENSATION = (*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT;
    // G.FLAG_TUNING_CURRENT_SCALE_FACTOR = TUNING_CURRENT_SCALE_FACTOR_INIT;


    /* Peripheral configurations */
    //(*CTRL).enc->OffsetCountBetweenIndexAndUPhaseAxis = 0;
    //(*CTRL).enc->theta_d_offset = (*CTRL).enc->OffsetCountBetweenIndexAndUPhaseAxis * CNT_2_ELEC_RAD;

    /* Console */
    // See init_experiment_overwrite() in CJHMainISR.c
    G.flag_do_inverter_characteristics = 0;
    G.overwrite_vdc = 20;

    /* Black Box Model | Controller quantities */

    // 控制器tuning
    if((*debug).who_is_user == USER_WB){
        _user_wubo_WC_Tuner();
        #if PC_SIMULATION == TRUE
            printf(">>> Wc_Tuner is Applied to the Speed Loop Control <<<\n");
        #endif
    }
    else{
        ACMSIMC_PIDTuner();
    }

    // commands
    (*CTRL).i->cmd_psi = d_sim.init.KE;

    (*CTRL).s->xRho = 0.0;
    (*CTRL).s->cosT = 1.0;
    (*CTRL).s->sinT = 0.0;
    (*CTRL).s->cosT_compensated_1p5omegaTs = 1.0;
    (*CTRL).s->sinT_compensated_1p5omegaTs = 0.0;
    (*CTRL).s->cosT2 = 1.0;
    (*CTRL).s->sinT2 = 0.0;
    (*CTRL).s->omega_syn = 0.0;
    (*CTRL).s->the_vc_count = 1;



    /* Capture */
    (*CTRL).cap->flag_nonlinear_filtering = FALSE;
    (*CTRL).cap->flag_bad_U_capture = FALSE;
    (*CTRL).cap->flag_bad_V_capture = FALSE;
    (*CTRL).cap->flag_bad_W_capture = FALSE;
    (*CTRL).cap->good_capture_U[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->ECapIntCount[0] = 0;
    (*CTRL).cap->ECapIntCount[1] = 0;
    (*CTRL).cap->ECapIntCount[2] = 0;
    (*CTRL).cap->ECapPassCount[0] = 0;
    (*CTRL).cap->ECapPassCount[1] = 0;
    (*CTRL).cap->ECapPassCount[2] = 0;

    init_im_controller();
}

