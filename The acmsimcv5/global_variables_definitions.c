#include "ACMSim.h"

#if PC_SIMULATION == FALSE
#pragma DATA_SECTION(CTRL       ,"MYGLOBALS");
#pragma DATA_SECTION(t_motor    ,"MYGLOBALS");
#pragma DATA_SECTION(t_enc      ,"MYGLOBALS");
#pragma DATA_SECTION(t_psd      ,"MYGLOBALS");
#pragma DATA_SECTION(t_I        ,"MYGLOBALS");
#pragma DATA_SECTION(t_S        ,"MYGLOBALS");
#pragma DATA_SECTION(t_O        ,"MYGLOBALS");
#pragma DATA_SECTION(t_inv      ,"MYGLOBALS");
#pragma DATA_SECTION(t_cap      ,"MYGLOBALS");
#pragma DATA_SECTION(t_g        ,"MYGLOBALS");
#pragma DATA_SECTION(pid1_iM    ,"MYGLOBALS");
#pragma DATA_SECTION(pid1_iT    ,"MYGLOBALS");
#pragma DATA_SECTION(pid1_pos   ,"MYGLOBALS");
#pragma DATA_SECTION(pid1_spd   ,"MYGLOBALS");
#endif

// 定义顶级结构体（指针的集合）
struct ControllerForExperiment CTRL;
// 定义内存空间（结构体）
st_pmsm_parameters      t_motor={0};
st_enc                  t_enc={0};
st_psd                  t_psd={0};
st_controller_inputs    t_I={0};
st_controller_states    t_S={0};
st_controller_outputs   t_O={0};
st_InverterNonlinearity t_inv={0}; // Because of the sv_count bug, I cannot declare t_inv in this .c file. // extern st_InverterNonlinearity t_inv; 
st_capture              t_cap={0};
st_global_variables     t_g={0};

st_pid_regulator      pid1_iM  = st_pid_regulator_DEFAULTS;
st_pid_regulator      pid1_iT  = st_pid_regulator_DEFAULTS;
st_pid_regulator      pid1_pos = st_pid_regulator_DEFAULTS;
st_pid_regulator      pid1_spd = st_pid_regulator_DEFAULTS;




// 初始化顶级结构体指针，指向定义好的内存空间
void allocate_CTRL(struct ControllerForExperiment *p){
    /* My attemp to use calloc with TI's compiler in CCS has failed. */
        // p->motor = calloc(1,sizeof(st_pmsm_parameters)); // 意思是，一个，st_pmsm_parameters那么大的空间
        // p->I = calloc(1,sizeof(st_controller_inputs));
        // p->S = calloc(1,sizeof(st_controller_states));
        // p->O = calloc(1,sizeof(st_controller_outputs));
    p->motor = &t_motor;
    p->enc   = &t_enc;
    p->psd   = &t_psd;
    p->I     = &t_I;
    p->S     = &t_S;
    p->O     = &t_O;
    p->inv   = &t_inv;
    p->cap   = &t_cap;
    p->g     = &t_g;

    p->S->iM  = &pid1_iM;
    p->S->iT  = &pid1_iT;
    p->S->pos = &pid1_pos;
    p->S->spd = &pid1_spd;
}


// CCS Debug Window
int Set_current_loop=0;

// Global watch variables
struct GlobalWatch watch;

/* Structs for Algorithm */
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

    struct ObserverForExperiment OBSV;
    struct SharedFluxEstimatorForExperiment FE;

    // 游离在顶级之外的算法结构体
    // struct RK4_DATA rk4;
    // struct Harnefors2006 harnefors={0};
    // struct CJH_EEMF_AO_Design cjheemf={0};
    // struct FARZA09_EEMF_HGO hgo4eemf={0};
    // struct Chen20_NSO_AF nsoaf={0};

#else
    // 游离在顶级之外的算法结构体
    struct RK4_DATA rk4;
    struct Marino2005 marino={0};

    struct Variables_SimulatedVM                         simvm      ={0};
    struct Variables_Ohtani1992                          ohtani     ={0};
    struct Variables_HuWu1998                            huwu       ={0};
    struct Variables_HoltzQuan2002                       holtz02    ={0};
    struct Variables_Holtz2003                           htz        ={0};
    struct Variables_Harnefors2003_SCVM                  harnefors  ={0};
    struct Variables_LascuAndreescus2006                 lascu      ={0};
    struct Variables_Stojic2015                          stojic     ={0};
    struct Variables_fluxModulusEstimator                fme        ={0};
    struct Variables_ExactCompensationMethod             exact      ={0};
    struct Variables_ProposedxRhoFramePICorrectionMethod picorr     ={0};
    struct Variables_ClosedLoopFluxEstimator             clest      ={0};
#endif


// struct eQEP_Variables qep={0};



