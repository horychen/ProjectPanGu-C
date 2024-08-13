#include "ACMSim.h"

// 定义顶级结构体（指针的集合）
int axisCnt = 0;
struct ControllerForExperiment CTRL_1;
struct ControllerForExperiment *CTRL;
struct DebugExperiment debug;
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
    // struct CJH_EEMF_AO_Design cjheemf={0};
    // struct FARZA09_EEMF_HGO hgo4eemf={0};
    // struct Chen20_NSO_AF nsoaf={0};

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



