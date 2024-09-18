#include "ACMSim.h"

#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)

// #if PC_SIMULATION==FALSE
// REAL CpuTimer_Delta = 0;
// Uint32 CpuTimer_Before = 0;
// Uint32 CpuTimer_After = 0;
// #endif


#define IM ACM // 权宜之计

// 实用函数
REAL sat_kappa(REAL x){
    if(x>marino.kappa){
        return marino.kappa;
    }else if(x<-marino.kappa){
        return -marino.kappa;
    }else{
        return x;
    }
}
REAL deriv_sat_kappa(REAL x){
    if(x>marino.kappa){
        return 0;
    }else if(x<-marino.kappa){
        return -0;
    }else{
        return 1;
    }
}

// 控制器
int32 BOOL_FAST_REVERSAL = FALSE;
int32 CONSTANT_SPEED_OPERATION = 0;
int32 Jerk = 0;
// REAL Jerk_value = 1;
REAL acc = 0.0;
// REAL delta_speed = 800;
REAL imife_accerleration_fast = 1600;        //  /60.0*2*M_PI; // 6400 rpm/s
REAL jerk = 6400 / 0.002;                    //  /60.0*2*M_PI; // rpm/s / s
REAL SINE_AMPL = 100.0 ;                     //  /60.0*2*M_PI;
REAL imife_ramp_slope = 100;                 //  /60.0*2*M_PI; // 400; // 400; // rpm/s
REAL imife_accerleration_slow = 20;          //  /60.0*2*M_PI; // rpm/s
REAL Jerk_time = 0.002;
REAL accerleration_time_inv=8;
REAL imife_reversal_end_time = 5.0; // s
REAL marino_speed_freq = 4;

void controller_marino2005_with_commands(){

    /// 0. 参数时变
    // if (fabs((*CTRL).timebase-2.0)<CL_TS){
    //     printf("[Runtime] Rotor resistance of the simulated IM has changed!\n");
    //     ACM.alpha = 0.5*INIT_RREQ / IM_MAGNETIZING_INDUCTANCE;
    //     ACM.rreq = ACM.alpha*ACM.Lmu;
    //     ACM.rr   = ACM.alpha*(ACM.Lm+ACM.Llr);
    // }


    /// 1. 生成转速指令
    static REAL rpm_speed_command=0.0, amp_current_command=0.0;
    {
        // commands(&rpm_speed_command, &amp_current_command);
        static REAL local_dc_rpm_cmd = 0.0;
        static REAL local_dc_rpm_cmd_deriv = 0.0;

        static REAL last_omg_cmd=0.0;
        REAL OMG1;
        #define DELAY 100
        #define OFF 1000
        if((*CTRL).timebase>6){/*Variable Speed: Sinusoidal Speed + Ramp Speed*/
            if      ((*CTRL).timebase>7.0){
                OMG1 = (2*M_PI*marino_speed_freq);
            }else if((*CTRL).timebase>6.875){
                OMG1 = (2*M_PI*32);
            }else if((*CTRL).timebase>6.75){
                OMG1 = (2*M_PI*16);
            }else if((*CTRL).timebase>6.5){
                OMG1 = (2*M_PI*8);
            }else{
                OMG1 = (2*M_PI*4);
            }
            //OMG1 = (2*M_PI*4);
            OMG1 = 0;

            // 反转(step speed)
            if(BOOL_FAST_REVERSAL){
                if((int)((*CTRL).timebase*accerleration_time_inv)%10==0){ //*8*400/750
                    local_dc_rpm_cmd_deriv = -imife_accerleration_fast;
                }else if((int)((*CTRL).timebase*accerleration_time_inv)%10==5){
                    local_dc_rpm_cmd_deriv = imife_accerleration_fast;
                }
                else 
                    local_dc_rpm_cmd_deriv = 0;

            }

            // 反转
            if(BOOL_FAST_REVERSAL==FALSE){
                if((*CTRL).timebase>10 && (*CTRL).timebase<15+imife_reversal_end_time){/*Reversal*/
                    // OMG1 = 0;
                    // TODO: test positive and negative high speed
                    local_dc_rpm_cmd_deriv = -imife_accerleration_slow;
                }else if((*CTRL).timebase>15+imife_reversal_end_time && (*CTRL).timebase<15.2+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.2+imife_reversal_end_time && (*CTRL).timebase<15.4+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv = - 1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.4+imife_reversal_end_time && (*CTRL).timebase<15.6+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.6+imife_reversal_end_time && (*CTRL).timebase<15.8+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv = - 1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.8+imife_reversal_end_time && (*CTRL).timebase<16.0+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else{
                    local_dc_rpm_cmd_deriv = 0.0;
                }
            }

            // local_dc_rpm_cmd_deriv = 0.0;

            //Jerk Hugh
            // if(Jerk==1){
            //     if((int)((*CTRL).timebase*8)%10==0){
            //         if( (*CTRL).timebase-(int)((*CTRL).timebase)>=0 && (*CTRL).timebase-(int)((*CTRL).timebase)<=Jerk_time){ //0-0.125
            //             acc = ((*CTRL).timebase-(int)((*CTRL).timebase)) * jerk;
            //             acc = - acc;
            //         }
            //         else if((*CTRL).timebase-(int)((*CTRL).timebase)>=(0.125-Jerk_time) && (*CTRL).timebase-(int)((*CTRL).timebase)<=0.125){
            //             acc = imife_accerleration_fast-(((*CTRL).timebase-(int)((*CTRL).timebase))-(0.125-Jerk_time)) * jerk;
            //             acc = - acc;
            //         }
            //         else
            //         acc = -imife_accerleration_fast;
            //     }else if((int)((*CTRL).timebase*8)%10==5){
            //         if( (*CTRL).timebase-(int)((*CTRL).timebase)>=0 && (*CTRL).timebase-(int)((*CTRL).timebase)<=Jerk_time){ //0-0.125
            //             acc = ((*CTRL).timebase-(int)((*CTRL).timebase)) * jerk;
            //             acc = acc;
            //         }
            //         else if((*CTRL).timebase-(int)((*CTRL).timebase)>=(0.125-Jerk_time) && (*CTRL).timebase-(int)((*CTRL).timebase)<=0.125){
            //             acc = imife_accerleration_fast-(((*CTRL).timebase-(int)((*CTRL).timebase))-(0.125-Jerk_time)) * jerk;
            //             acc = acc;
            //         }
            //         else
            //         acc = imife_accerleration_fast;
            //     }
            //     else 
            //         acc = 0;
            // }
            // local_dc_rpm_cmd_deriv = acc; // jerk

            local_dc_rpm_cmd            += CL_TS * local_dc_rpm_cmd_deriv;

            rpm_speed_command              = (SINE_AMPL          * sin(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd       );
            (*CTRL).i->cmd_varOmega        = (SINE_AMPL          * sin(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd       )*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = (SINE_AMPL*OMG1     * cos(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd_deriv )*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_dderiv_varOmega = (SINE_AMPL*OMG1*OMG1*-sin(OMG1*(*CTRL).timebase) + 0                      )*RPM_2_MECH_RAD_PER_SEC;
        }else if((*CTRL).timebase>5+OFF){/*Constant Speed*/
            rpm_speed_command           = local_dc_rpm_cmd;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
        }else if((*CTRL).timebase>3+OFF){/*Ramp Speed*/
            rpm_speed_command += CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>2+OFF){/*Ramp Speed (Downward)*/
            rpm_speed_command -= CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>1.5+OFF){/*Ramp Speed*/
            rpm_speed_command += CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>1.5+0.0){ /*Constant Speed*/
            rpm_speed_command           = local_dc_rpm_cmd;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
        }else if((*CTRL).timebase>0.5+0.0){ /*Ramp Speed*/
            rpm_speed_command           += CL_TS*imife_ramp_slope;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else{ /*Bulding Flux*/
            rpm_speed_command           = 0; // 0*20                   * sin(2*M_PI*(*CTRL).timebase);
            (*CTRL).i->cmd_varOmega        = 0; // 0*20                   * sin(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0; // 0*20*(2*M_PI)          * cos(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_dderiv_varOmega = 0; // 0*20*(2*M_PI)*(2*M_PI) *-sin(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
        }
    }

    // // Overwrite speed command
    // if(CONSTANT_SPEED_OPERATION!=0){
    //     rpm_speed_command           = 0;
    //     (*CTRL).i->cmd_varOmega        = CONSTANT_SPEED_OPERATION*RPM_2_MECH_RAD_PER_SEC;
    //     (*CTRL).i->cmd_deriv_varOmega  = 0;
    //     (*CTRL).i->cmd_dderiv_varOmega = 0;
    // }

    /// 2. 生成磁链指令
    #define TIME_COST 0.1
    if((*CTRL).timebase<TIME_COST){
        (*CTRL).i->cmd_psi_raw   += CL_TS*(*CTRL).i->m0/TIME_COST;
        (*CTRL).i->cmd_psi        = (*CTRL).i->cmd_psi_raw;
        (*CTRL).i->cmd_deriv_psi  = (*CTRL).i->m0/TIME_COST;
        (*CTRL).i->cmd_dderiv_psi = 0.0;
    }else{
        // (*CTRL).i->m1 = 0.0;
        (*CTRL).i->cmd_psi_raw    = (*CTRL).i->m0 + (*CTRL).i->m1 * sin((*CTRL).i->omega1*(*CTRL).timebase);
        (*CTRL).i->cmd_psi        = (*CTRL).i->cmd_psi_raw; // _lpf((*CTRL).i->cmd_psi_raw, (*CTRL).i->cmd_psi, 5);
        (*CTRL).i->cmd_deriv_psi  = (*CTRL).i->m1 * (*CTRL).i->omega1 * cos((*CTRL).i->omega1*(*CTRL).timebase);
        (*CTRL).i->cmd_dderiv_psi = (*CTRL).i->m1 * (*CTRL).i->omega1 * (*CTRL).i->omega1 * -sin((*CTRL).i->omega1*(*CTRL).timebase);
    }
    (*CTRL).i->cmd_psi_inv = 1.0 / (*CTRL).i->cmd_psi;
    (*CTRL).i->cmd_psi_active[0] = MT2A((*CTRL).i->cmd_psi, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT); // TODO 这里的cosT和sinT还没更新到当前步，有没有关系？
    (*CTRL).i->cmd_psi_active[1] = MT2B((*CTRL).i->cmd_psi, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);

    /// 调用具体的控制器
    // controller_IFOC();
    controller_marino2005();

    main_inverter_voltage_command(0);
}
extern REAL marino_sat_d_axis_flux_control;
extern REAL marino_sat_q_axis_flux_control;
void controller_marino2005(){

    // Cascaded from other system
    /* Flux Est. */
    flux_observer();

        // flux feedback
        if (FALSE){
            /*Simulation only flux*/
            // marino.psi_Dmu = simvm.psi_D2_ode1;
            // marino.psi_Qmu = simvm.psi_Q2_ode1;
            // // marino.psi_Dmu = simvm.psi_D2_ode1_v2;
            // // marino.psi_Qmu = simvm.psi_Q2_ode1_v2;

            /*A. Exact Compensation based on Waveform Top and Butt */
                // marino.psi_Dmu = exact.psi_DQ2[0];
                // marino.psi_Qmu = exact.psi_DQ2[1];
                // marino.psi_Dmu = AB2M(exact.psi_2[0], exact.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // marino.psi_Qmu = AB2T(exact.psi_2[0], exact.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // #define COMPENSATION 0
                // marino.psi_Dmu = AB2M(exact.psi_2[0]-COMPENSATION*exact.filtered_compensation[0], exact.psi_2[1]-COMPENSATION*exact.filtered_compensation[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // marino.psi_Qmu = AB2T(exact.psi_2[0]-COMPENSATION*exact.filtered_compensation[0], exact.psi_2[1]-COMPENSATION*exact.filtered_compensation[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Dmu = AB2M(FE.exact.psi_2_real_output[0], FE.exact.psi_2_real_output[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.exact.psi_2_real_output[0], FE.exact.psi_2_real_output[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*ohtani*/
            marino.psi_Dmu = AB2M(FE.ohtani.psi_2[0], FE.ohtani.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.ohtani.psi_2[0], FE.ohtani.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*holtz2002*/

            /*boldea*/
            // marino.psi_Dmu = AB2M(FE.boldea.psi_2[0], FE.boldea.psi_2[1], CTRL.S->cosT, CTRL.S->sinT);
            // marino.psi_Qmu = AB2T(FE.boldea.psi_2[0], FE.boldea.psi_2[1], CTRL.S->cosT, CTRL.S->sinT);

            /*picorr*/
            marino.psi_Dmu = AB2M(FE.picorr.psi_2[0], FE.picorr.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.picorr.psi_2[0], FE.picorr.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*harnefors*/
            marino.psi_Dmu = AB2M(FE.harnefors.psi_2[0], FE.harnefors.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.harnefors.psi_2[0], FE.harnefors.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*lascu*/
            marino.psi_Dmu = AB2M(FE.lascu.psi_2[0], FE.lascu.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.lascu.psi_2[0], FE.lascu.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*clest*/
            marino.psi_Dmu = AB2M(FE.clest.psi_2[0], FE.clest.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.clest.psi_2[0], FE.clest.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*holtz2003*/
            marino.psi_Dmu = AB2M(FE.htz.psi_2[0], FE.htz.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.htz.psi_2[0], FE.htz.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
        }

    // #if PC_SIMULATION==FALSE
    // EALLOW;
    // CpuTimer1.RegsAddr->TCR.bit.TRB = 1; // reset cpu timer to period value
    // CpuTimer1.RegsAddr->TCR.bit.TSS = 0; // start/restart
    // CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
    // EDIS;
    // #endif

    // TODO: 反馈磁链用谁？
    marino.psi_Dmu = AB2M(FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA, (*CTRL).s->cosT, (*CTRL).s->sinT);
    marino.psi_Qmu = AB2T(FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA, (*CTRL).s->cosT, (*CTRL).s->sinT);
    #if PC_SIMULATION
        // marino.psi_Dmu = AB2M(ACM.psi_Amu, ACM.psi_Bmu, (*CTRL).s->cosT, (*CTRL).s->sinT);
        // marino.psi_Qmu = AB2T(ACM.psi_Amu, ACM.psi_Bmu, (*CTRL).s->cosT, (*CTRL).s->sinT);
    #endif

    // /*Simulation only flux*/
    // marino.psi_Dmu = simvm.psi_D2_ode1;
    // marino.psi_Qmu = simvm.psi_Q2_ode1;
    // marino.psi_Dmu = ACM.psi_Dmu; // nonono, the dq flux should be obtained using (*CTRL).s->cosT/sinT.
    // marino.psi_Qmu = ACM.psi_Qmu; // nonono, the dq flux should be obtained using (*CTRL).s->cosT/sinT.

    // flux error quantities should be updated when feedback is updated | verified: this has nothing to do with the biased xTL at high speeds
    marino.e_psi_Dmu = marino.psi_Dmu - (*CTRL).i->cmd_psi;
    marino.e_psi_Qmu = marino.psi_Qmu - 0.0;

    // marino.e_psi_Dmu *= marino_sat_d_axis_flux_control; // no luck
    // marino.e_psi_Qmu *= marino_sat_q_axis_flux_control; // no luck


    // API to the fourth-order system of observer and identifiers
    observer_marino2005();
    (*CTRL).i->theta_d_elec = marino.xRho;
    REAL xOMG = marino.xOmg; // * MOTOR.npp_inv;
    (*CTRL).motor->alpha    = marino.xAlpha;
    (*CTRL).i->TLoad        = marino.xTL;

    // #define xOMG (CTRL->i->varOmega*MOTOR.npp)
    // REAL xOMG = (*CTRL).i->varOmega * MOTOR.npp;

    // 磁场可测 debug
    // marino.psi_Dmu = simvm.psi_D2;
    // marino.psi_Qmu = simvm.psi_Q2;
    // (*CTRL).i->theta_d_elec = ACM.theta_M;
    // (*CTRL).i->varOmega = (*CTRL).i->varOmega;
    // (*CTRL).motor->alpha = ACM.alpha;
    // (*CTRL).i->TLoad = ACM.TLoad;

    // flux error quantities (moved up)
    // marino.e_psi_Dmu = marino.psi_Dmu - (*CTRL).i->cmd_psi;
    // marino.e_psi_Qmu = marino.psi_Qmu - 0.0;

    (*CTRL).motor->alpha_inv = 1.0/(*CTRL).motor->alpha;
    // (*CTRL).motor->Lmu = (*CTRL).motor->Rreq * (*CTRL).motor->alpha_inv;
    // (*CTRL).motor->Lmu_inv = 1.0 / (*CTRL).motor->Lmu;

    // αβ to DQ
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT);

    if(TRUE){
        // 当磁链幅值给定平稳时，这项就是零。
        marino.deriv_iD_cmd = 1.0*(*CTRL).motor->Lmu_inv*(  (*CTRL).i->cmd_deriv_psi \
                                                + (*CTRL).i->cmd_dderiv_psi*(*CTRL).motor->alpha_inv \
                                                - (*CTRL).i->cmd_deriv_psi*(*CTRL).motor->alpha_inv*(*CTRL).motor->alpha_inv*marino.deriv_xAlpha);
        // 重新写！
        // REAL mu_temp     = (*CTRL).motor->npp_inv*(*CTRL).motor->Js * CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv;
        // REAL mu_temp_inv = (*CTRL).motor->npp*(*CTRL).motor->Js_inv * CLARKE_TRANS_TORQUE_GAIN*(*CTRL).motor->npp;
        // 第一项很有用，第二项无用。
        marino.deriv_iQ_cmd =   (*CTRL).motor->npp_inv*(*CTRL).motor->Js * CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv * (\
            1.0*(-marino.k_omega*deriv_sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) * (marino.deriv_xOmg - (*CTRL).i->cmd_deriv_varOmega ) + (*CTRL).motor->Js_inv*(*CTRL).motor->npp*marino.deriv_xTL + (*CTRL).i->cmd_dderiv_varOmega  ) * (*CTRL).i->cmd_psi_inv\
          - 1.0*(-marino.k_omega*      sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) + (*CTRL).motor->Js_inv*(*CTRL).motor->npp*(*CTRL).i->TLoad + (*CTRL).i->cmd_deriv_varOmega ) * ((*CTRL).i->cmd_deriv_psi * (*CTRL).i->cmd_psi_inv*(*CTRL).i->cmd_psi_inv)
            );

        // current error quantities
        (*CTRL).i->cmd_iDQ[0] = ( (*CTRL).i->cmd_psi + (*CTRL).i->cmd_deriv_psi*(*CTRL).motor->alpha_inv ) * (*CTRL).motor->Lmu_inv;
        (*CTRL).i->cmd_iDQ[1] = ( (*CTRL).motor->npp_inv*(*CTRL).motor->Js *( 1*(*CTRL).i->cmd_deriv_varOmega  - marino.k_omega*sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) ) + (*CTRL).i->TLoad ) * (CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv*(*CTRL).i->cmd_psi_inv);
        marino.e_iDs = (*CTRL).i->iDQ[0] - (*CTRL).i->cmd_iDQ[0];
        marino.e_iQs = (*CTRL).i->iDQ[1] - (*CTRL).i->cmd_iDQ[1];

        marino.torque_cmd = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->cmd_iDQ[1] * (*CTRL).i->cmd_psi   - (*CTRL).i->cmd_iDQ[0]*(0));
        marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->iDQ[1]     * marino.psi_Dmu - (*CTRL).i->iDQ[0] * marino.psi_Qmu);
        // marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->iDQ[1]     * marino.psi_Dmu);


        // linear combination of error
        marino.zD = marino.e_iDs + (*CTRL).motor->Lsigma_inv*marino.e_psi_Dmu;
        marino.zQ = marino.e_iQs + (*CTRL).motor->Lsigma_inv*marino.e_psi_Qmu;

        // known signals to feedforward (to cancel)
        marino.Gamma_D = (*CTRL).motor->Lsigma_inv * (-(*CTRL).motor->R*(*CTRL).i->iDQ[0] -(*CTRL).motor->alpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[0] +(*CTRL).motor->alpha  *(*CTRL).i->cmd_psi +(*CTRL).s->omega_syn*marino.e_psi_Qmu) +(*CTRL).s->omega_syn*(*CTRL).i->iDQ[1] - marino.deriv_iD_cmd;
        marino.Gamma_Q = (*CTRL).motor->Lsigma_inv * (-(*CTRL).motor->R*(*CTRL).i->iDQ[1] -(*CTRL).motor->alpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[1] -xOMG *(*CTRL).i->cmd_psi -(*CTRL).s->omega_syn*marino.e_psi_Dmu) -(*CTRL).s->omega_syn*(*CTRL).i->iDQ[0] - marino.deriv_iQ_cmd;

        // voltage commands
        (*CTRL).o->cmd_uDQ[0] = (*CTRL).motor->Lsigma * (-(marino.kz+0.25*(*CTRL).motor->Lsigma*(*CTRL).motor->Lmu*marino.xAlpha_Max)*marino.zD - marino.Gamma_D);
        (*CTRL).o->cmd_uDQ[1] = (*CTRL).motor->Lsigma * (-(marino.kz+0.25*(*CTRL).motor->Lsigma*(*CTRL).motor->Lmu*marino.xAlpha_Max)*marino.zQ - marino.Gamma_Q);

    }else{
        // PI control with marino observer works if 
        // damping factor = 6.5
        // VLBW = 5 Hz
        PID_iD->Fbk = (*CTRL).i->iDQ[0];
        PID_iQ->Fbk = (*CTRL).i->iDQ[1];

        /// 5. 转速环
        static int im_vc_count = 1;
        if(im_vc_count++ == SPEED_LOOP_CEILING){
            im_vc_count = 1;

            PID_Speed->Ref = (*CTRL).i->cmd_varOmega ; //rpm_speed_command*RPM_2_ELEC_RAD_PER_SEC;
            PID_Speed->Fbk = xOMG ;
            PID_Speed->calc(PID_Speed);
            PID_iQ->Ref = PID_Speed->Out;
            (*CTRL).i->cmd_iDQ[1] = PID_iQ->Ref;
        }
        (*CTRL).i->cmd_psi = IM_FLUX_COMMAND_DC_PART;
        (*CTRL).i->cmd_iDQ[0] = (*CTRL).i->cmd_psi * (*CTRL).motor->Lmu_inv;
        PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];

        /// 6. 电流环
        REAL decoupled_M_axis_voltage=0.0, decoupled_T_axis_voltage=0.0;
        PID_iD->calc(PID_iD);
        PID_iQ->calc(PID_iQ);

        decoupled_M_axis_voltage = PID_iD->Out;
        decoupled_T_axis_voltage = PID_iQ->Out;

        (*CTRL).o->cmd_uDQ[0] = decoupled_M_axis_voltage;
        (*CTRL).o->cmd_uDQ[1] = decoupled_T_axis_voltage;

        /// 7. 反帕克变换
        // (*CTRL).o->cmd_uAB[0] = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
        // (*CTRL).o->cmd_uAB[1] = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
    }

    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    // #if PC_SIMULATION==FALSE
    // CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
    // CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
    // #endif

    // use the second 3 phase inverter
    // (*CTRL).o->cmd_uAB[0+2] = (*CTRL).o->cmd_uAB[0];
    // (*CTRL).o->cmd_uAB[1+2] = (*CTRL).o->cmd_uAB[1];

    // for view in scope
    // (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    // (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
}



void controller_IFOC(){

    /// 3. 电气转子位置和电气转子转速反馈

    flux_observer(); // FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA
    // (*CTRL).i->varOmega = FE.htz.omg_est;=

                    // (*CTRL).i->varOmega = OBSV.esoaf.xOmg - (OBSV.esoaf.bool_ramp_load_torque<0) * (*CTRL).s->omega_sl;
                    // (*CTRL).i->varOmega = OBSV.esoaf.xOmg;

                        //（编码器反馈）
                        // (*CTRL).i->varOmega     = qep.varOmega;
                        // (*CTRL).i->theta_d_elec__fb = qep.theta_d;

                        //（实际反馈，实验中不可能）
                        // (*CTRL).i->varOmega     = ENC.varOmega ;
                        // (*CTRL).i->varOmega     = ACM.varOmega ;
                        // (*CTRL).i->varOmega     = ACM.x[4] ;

                        //（无感）
                        // harnefors_scvm();
                        // (*CTRL).i->varOmega     = omg_harnefors;
                        // (*CTRL).i->theta_d_elec__fb = theta_d_harnefors;

    // 间接磁场定向第一部分
    (*CTRL).s->xRho += CL_TS * (*CTRL).s->omega_syn;
    // (*CTRL).s->xRho = ACM.theta_d; // debug
    (*CTRL).i->theta_d_elec = (*CTRL).s->xRho;

    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    if((*CTRL).s->xRho > M_PI){
        (*CTRL).s->xRho -= 2*M_PI;
    }else if((*CTRL).s->xRho < -M_PI){
        (*CTRL).s->xRho += 2*M_PI; // 反转！
    }

    /// 4. 帕克变换
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iQ->Fbk = (*CTRL).i->iDQ[1];

    /// 5. 转速环
    static int im_vc_count = 1;
    if(im_vc_count++ == SPEED_LOOP_CEILING){
        im_vc_count = 1;

        PID_Speed->Ref = (*CTRL).i->cmd_varOmega; //rpm_speed_command*RPM_2_ELEC_RAD_PER_SEC;
        PID_Speed->Fbk = (*CTRL).i->varOmega;
        PID_Speed->calc(PID_Speed);
        PID_iQ->Ref = PID_Speed->Out;
        (*CTRL).i->cmd_iDQ[1] = PID_iQ->Ref;
    }
    // 磁链环
    // if(ob.taao_flux_cmd_on){
    //     (*CTRL).i->cmd_iDQ[0] = IM_FLUX_COMMAND_DC_PART * (*CTRL).motor->Lmu_inv   \
    //                    + (   M1*OMG1*cos(OMG1*ob.timebase)  )/ (*CTRL).motor->Rreq; ///////////////////////////////// 
    // }else{
        // (*CTRL).i->cmd_iDQ[0] =  * (*CTRL).motor->Lmu_inv + (deriv_fluxModCmd)/ (*CTRL).motor->Rreq; 
        (*CTRL).i->cmd_psi = IM_FLUX_COMMAND_DC_PART;
        (*CTRL).i->cmd_iDQ[0] = (*CTRL).i->cmd_psi * (*CTRL).motor->Lmu_inv;
    // }
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    // 计算转矩
    (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * (*CTRL).i->cmd_iDQ[1] * ((*CTRL).i->cmd_psi);
    // 间接磁场定向第二部分
    (*CTRL).s->omega_sl  = (*CTRL).motor->Rreq*(*CTRL).i->cmd_iDQ[1]/((*CTRL).i->cmd_psi);
    (*CTRL).s->omega_syn = (*CTRL).i->varOmega * CTRL->motor->npp + (*CTRL).s->omega_sl;

    /// 5.Extra 扫频将覆盖上面产生的励磁、转矩电流指令
    #if EXCITATION_TYPE == EXCITATION_SWEEP_FREQUENCY
        #if SWEEP_FREQ_C2V == TRUE
            PID_iQ->Ref = amp_current_command; 
        #endif
        #if SWEEP_FREQ_C2C == TRUE
            PID_iQ->Ref = 0.0;
            PID_iD->Ref = amp_current_command; // 故意反的
        #else
            PID_iD->Ref = 0.0;
        #endif
    #endif



    /// 6. 电流环
    REAL decoupled_M_axis_voltage=0.0, decoupled_T_axis_voltage=0.0;
    PID_iD->calc(PID_iD);
    PID_iQ->calc(PID_iQ);
    {   // Steady state dynamics based decoupling circuits for current regulation
        if (d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
            // decoupled_M_axis_voltage = vM + ((*CTRL).motor->R+(*CTRL).motor->Rreq)*(*CTRL).iMs + (*CTRL).motor->Lsigma*(-(*CTRL).s->omega_syn*(*CTRL).iTs) - (*CTRL).motor->alpha*(*CTRL).psimod_fb; // Jadot09
            // decoupled_T_axis_voltage = vT + ((*CTRL).motor->R+(*CTRL).motor->Rreq)*(*CTRL).iTs + (*CTRL).motor->Lsigma*( (*CTRL).s->omega_syn*(*CTRL).iMs) + (*CTRL).omg_fb*(*CTRL).psimod_fb;
            // decoupled_T_axis_voltage = vT + (*CTRL).s->omega_syn*((*CTRL).motor->Lsigma+(*CTRL).motor->Lmu)*(*CTRL).iMs; // 这个就不行，说明：(*CTRL).motor->Lmu*iMs != ob.taao_flux_cmd，而是会因iMs的波动在T轴控制上引入波动和不稳定

            decoupled_M_axis_voltage = PID_iD->Out + ((*CTRL).motor->Lsigma) * (-(*CTRL).s->omega_syn*(*CTRL).i->iDQ[1]); // Telford03/04
            decoupled_T_axis_voltage = PID_iQ->Out + (*CTRL).s->omega_syn*((*CTRL).i->cmd_psi + (*CTRL).motor->Lsigma*(*CTRL).i->iDQ[0]); // 这个行，但是无速度运行时，会导致M轴电流在转速暂态高频震荡。
            // decoupled_T_axis_voltage = PID_iQ->Out; // 无感用这个
        }else{
            decoupled_M_axis_voltage = PID_iD->Out;
            decoupled_T_axis_voltage = PID_iQ->Out;
        }
    }

    /// 7. 反帕克变换
    (*CTRL).o->cmd_uAB[0] = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);


    /// 8. 补偿逆变器非线性
    main_inverter_voltage_command(TRUE);

}

void init_im_controller(){

    #define LOCAL_SCALE 1.0
    marino.kz         = LOCAL_SCALE * 2*700.0; // zd, zq
    marino.k_omega    = LOCAL_SCALE * 0.5*88*60.0; // 6000  // e_omega // 增大这个可以消除稳态转速波形中的正弦扰动（源自q轴电流给定波形中的正弦扰动，注意实际的q轴电流里面是没有正弦扰动的）


    marino.kappa      = 1e4*24; // \in [0.1, 1e4*24] no difference // e_omega // 增大这个意义不大，转速控制误差基本上已经是零了，所以kappa取0.05和24没有啥区别。

    // lammbda_inv和gamma_inv是竞争的关系
    // marino.lambda_inv = 5* 0.1 * 1.5 * 6000.0;          // omega 磁链反馈为实际值时，这两个增益取再大都没有意义。
    // marino.gamma_inv  = 10 * 3e0 * 180/INIT_JS; // TL    磁链反馈为实际值时，这两个增益取再大都没有意义。
    // marino.delta_inv  = 0*75.0; // alpha 要求磁链幅值时变

    marino.lambda_inv = LAMBDA_INV_xOmg;  // 2022-11-22 实验中发现这个过大（2700）导致系统整个一个正弦波动，母线功率持续400W，减小为1000以后母线功率37W。
    marino.gamma_inv  = GAMMA_INV_xTL;
    marino.delta_inv  = DELTA_INV_alpha;

    marino.xTL_Max = 8.0;
    marino.xAlpha_Max = 8.0;
    marino.xAlpha_min = 1.0;

    marino.xRho = 0.0;
    marino.xTL = 0.0;
    marino.xAlpha = d_sim.init.Rreq / IM_MAGNETIZING_INDUCTANCE;
    marino.xOmg = 0.0;

    marino.deriv_xTL = 0.0;
    marino.deriv_xAlpha = 0.0;
    marino.deriv_xOmg = 0.0;

    marino.psi_Dmu = 0.0;
    marino.psi_Qmu = 0.0;

    marino.zD = 0.0;
    marino.zQ = 0.0;
    marino.e_iDs = 0.0;
    marino.e_iQs = 0.0;
    marino.e_psi_Dmu = 0.0;
    marino.e_psi_Qmu = 0.0;

    marino.deriv_iD_cmd = 0.0;
    marino.deriv_iQ_cmd = 0.0;

    marino.Gamma_D = 0.0;
    marino.Gamma_Q = 0.0;

    marino.torque_cmd = 0.0;
    marino.torque__fb = 0.0;

    // struct Holtz2003
    simvm.psi_D2 = 0.0;
    simvm.psi_Q2 = 0.0;
    simvm.psi_D1_ode1 = 0.0;
    simvm.psi_Q1_ode1 = 0.0;
    simvm.psi_D2_ode1 = 0.0;
    simvm.psi_Q2_ode1 = 0.0;
    simvm.psi_D1_ode4 = 0.0;
    simvm.psi_Q1_ode4 = 0.0;
    simvm.psi_D2_ode4 = 0.0;
    simvm.psi_Q2_ode4 = 0.0;





    int i=0,j=0;

    (*CTRL).timebase = 0.0;

    /* Parameter (including speed) Adaptation */ 
        (*CTRL).motor->R      = d_sim.init.R;
        (*CTRL).motor->Rreq   = d_sim.init.Rreq;

        (*CTRL).motor->npp    = d_sim.init.npp;
        (*CTRL).motor->Lsigma = d_sim.init.Lq;
        (*CTRL).motor->Lmu    = d_sim.init.Ld - d_sim.init.Lq;
        (*CTRL).motor->Js     = d_sim.init.Js;

        (*CTRL).motor->alpha  = (*CTRL).motor->Rreq/(*CTRL).motor->Lmu;
        (*CTRL).motor->alpha_inv = 1.0/(*CTRL).motor->alpha;

        (*CTRL).motor->npp_inv     = 1.0/(*CTRL).motor->npp;
        (*CTRL).motor->Lsigma_inv  = 1.0/(*CTRL).motor->Lsigma;
        (*CTRL).motor->Lmu_inv     = 1.0/(*CTRL).motor->Lmu;
        (*CTRL).motor->Js_inv      = 1.0/(*CTRL).motor->Js;

        // (*CTRL).i->TLoad  = 0.0;

    (*CTRL).s->cosT = 1.0;
    (*CTRL).s->sinT = 0.0;
    (*CTRL).s->cosT2 = 1.0;
    (*CTRL).s->sinT2 = 0.0;

    (*CTRL).i->m0 = IM_FLUX_COMMAND_DC_PART;
    (*CTRL).i->m1 = IM_FLUX_COMMAND_SINE_PART;
    (*CTRL).i->omega1 = 2*M_PI*IM_FLUX_COMMAND_SINE_HERZ;

    // (*debug).SENSORLESS_CONTROL = SENSORLESS_CONTROL;
    // (*CTRL).s->ctrl_strategy = CONTROL_STRATEGY;

    #define AKATSU00 FALSE
    #if AKATSU00 == TRUE
    int ind;
    for(ind=0;ind<2;++ind){
    // for(i=0;i<2;++i){
        hav.emf_stator[ind] = 0;

        hav.psi_1[ind] = 0;
        hav.psi_2[ind] = 0;
        hav.psi_2_prev[ind] = 0;

        hav.psi_1_nonSat[ind] = 0;
        hav.psi_2_nonSat[ind] = 0;

        hav.psi_1_min[ind] = 0;
        hav.psi_1_max[ind] = 0;
        hav.psi_2_min[ind] = 0;
        hav.psi_2_max[ind] = 0;

        hav.rs_est = 3.04;
        hav.rreq_est = 1.6;

        hav.Delta_t = 1;
        hav.u_off[ind] = 0;
        hav.u_off_integral_input[ind] = 0;
        hav.gain_off = 0.025;

        hav.flag_pos2negLevelA[ind] = 0;
        hav.flag_pos2negLevelB[ind] = 0;
        hav.time_pos2neg[ind] = 0;
        hav.time_pos2neg_prev[ind] = 0;

        hav.flag_neg2posLevelA[ind] = 0;
        hav.flag_neg2posLevelB[ind] = 0;
        hav.time_neg2pos[ind] = 0;
        hav.time_neg2pos_prev[ind] = 0;    

        hav.sat_min_time[ind] = 0.0;
        hav.sat_max_time[ind] = 0.0;
    }

    a92v.awaya_lambda = 31.4*1;
    a92v.q0 = 0.0;
    a92v.q1_dot = 0.0;
    a92v.q1 = 0.0;
    a92v.tau_est = 0.0;
    a92v.sum_A = 0.0;
    a92v.sum_B = 0.0;
    a92v.est_Js_variation = 0.0;
    a92v.est_Js = 0.0;
    #endif

    // /*Jadot2009*/
    // (*CTRL).is_ref[0] = 0.0;
    // (*CTRL).is_ref[1] = 0.0;
    // (*CTRL).psi_ref[0] = 0.0;
    // (*CTRL).psi_ref[1] = 0.0;

    // (*CTRL).pi_vsJadot_0.Kp = 7; 
    // (*CTRL).pi_vsJadot_0.Ti = 1.0/790.0; 
    // (*CTRL).pi_vsJadot_0.Kp = 15; 
    // (*CTRL).pi_vsJadot_0.Ti = 0.075; 
    // (*CTRL).pi_vsJadot_0.Ki = (*CTRL).pi_vsJadot_0.Kp / (*CTRL).pi_vsJadot_0.Ti * TS;
    // (*CTRL).pi_vsJadot_0.i_state = 0.0;
    // (*CTRL).pi_vsJadot_0.i_limit = 300.0; // unit: Volt

    // (*CTRL).pi_vsJadot_1.Kp = 7; 
    // (*CTRL).pi_vsJadot_1.Ti = 1.0/790.0; 
    // (*CTRL).pi_vsJadot_1.Kp = 15; 
    // (*CTRL).pi_vsJadot_1.Ti = 0.075; 
    // (*CTRL).pi_vsJadot_1.Ki = (*CTRL).pi_vsJadot_1.Kp / (*CTRL).pi_vsJadot_1.Ti * TS;
    // (*CTRL).pi_vsJadot_1.i_state = 0.0;
    // (*CTRL).pi_vsJadot_1.i_limit = 300.0; // unit: Volt

    // PID调谐
    // ACMSIMC_PIDTuner();
}



#endif
