/*
 * Deadtime_compensation.c
 *
 *  Created on: 2017Äê5ÔÂ12ÈÕ
 *      Author: Pis
 */

//#include "main_config.h"
#ifdef USE_YX_DEADTIME_COMPENSATION
DEADTIME deadtime1;
int Deadtime_compensation_Sector = 0 ;
float Deadtime_voltage;
float Deadtime_predictive_d_current=0;
float Deadtime_predictive_q_current=0;
int Set_deadtime_compensation_method=0;
void Deadtime_compensation(PARK* ptrV2,DEADTIME* ptrV)
{
    PARK v2;
    DEADTIME v;
    v=(*ptrV);
    v2=(*ptrV2);
    float System_Deadtime_pwm,Igbt_pwm_ton,Igbt_pwm_toff;
    float Igbt_Vce0,Igbt_Vd0;
    float Current_deadtime_angle;
    System_Deadtime_pwm=SYSTEM_PWM_DEADTIME*UNIT_US_DEADTIME;
    Igbt_pwm_ton=IGBT_PWM_TON*UNIT_US_DEADTIME;
    Igbt_pwm_toff=IGBT_PWM_TOFF*UNIT_US_DEADTIME;
    Igbt_Vce0=IGBT_VCE0_VOLTAGE;
    Igbt_Vd0=IGBT_DIODE0_VOLTAGE;
    // evaluate the deadtime voltage
    Deadtime_voltage=(ctrl.inverter->vdc*(System_Deadtime_pwm+Igbt_pwm_ton-Igbt_pwm_toff)*ctrl.switching_frq+(Igbt_Vce0+Igbt_Vd0)/2)*4/3;

    //evaluate current angle for SPMSM id=0;current_angle=r_anlge+90
    Current_deadtime_angle=v.Electrical_angle+M_PI_2;

    if(Current_deadtime_angle>M_PI_2&&Current_deadtime_angle<M_PI_2+M_PI_3)
    Deadtime_compensation_Sector=3;
    else if(Current_deadtime_angle>M_PI_2+M_PI_3&&Current_deadtime_angle<M_PI_2+2*M_PI_3)
    Deadtime_compensation_Sector=4;
    else if(Current_deadtime_angle>M_PI_2+2*M_PI_3&&Current_deadtime_angle<M_PI_2+M_PI)
    Deadtime_compensation_Sector=5;
    else if(Current_deadtime_angle>M_PI_2+M_PI&&Current_deadtime_angle<M_PI_2+4*M_PI_3)
    Deadtime_compensation_Sector=6;
    else if(Current_deadtime_angle>M_PI_2+4*M_PI_3&&Current_deadtime_angle<M_PI_2+5*M_PI_3)
    Deadtime_compensation_Sector=1;
    else if(Current_deadtime_angle>M_PI_2+5*M_PI_3&&Current_deadtime_angle<M_PI_2+2*M_PI)
    Deadtime_compensation_Sector=2;

    //Judgment sector and output
    switch(Deadtime_compensation_Sector)
    {

        case 1:   // Sector 1:-30 -30
            v.Ualpha_compensation=Deadtime_voltage;
            v.Ubeta_compensation=0;
        break;
        case 2:   // Sector 2: 30- 90
            v.Ualpha_compensation=Deadtime_voltage*0.5;
            v.Ubeta_compensation=Deadtime_voltage*SQRT_3;
        break;
        case 3:   // Sector 3:90- 150
            v.Ualpha_compensation=-Deadtime_voltage*0.5;
            v.Ubeta_compensation=Deadtime_voltage*SQRT_3;
        break;
        case 4:   // Sector 4:150-210
            v.Ualpha_compensation=-Deadtime_voltage;
            v.Ubeta_compensation=0;
        break;
        case 5:   // Sector 5:210- 270
            v.Ualpha_compensation=-Deadtime_voltage*0.5;
            v.Ubeta_compensation=-Deadtime_voltage*SQRT_3;
        break;
        case 6:   // Sector 6:270- 330
            v.Ualpha_compensation=Deadtime_voltage*0.5;
            v.Ubeta_compensation=-Deadtime_voltage*SQRT_3;
        break;
    }

    (*ptrV)=v;
    (*ptrV2)=v2;
}
void Deadtime_compensation_observer(DEADTIME* ptrV)
{
    DEADTIME v;
    IPARK v2;
    v=(*ptrV);
    float Deadtime_e_d_deadtime,Deadtime_e_q_deadtime;

    v.wn_deadtime=v.bandwidth_deadtime*2*M_PI;
    v.K_lo_1=2*v.wn_deadtime-ctrl.para->Rs/ctrl.para->Ld;
    v.K_lo_2=-v.wn_deadtime*ctrl.para->Ld*v.wn_deadtime;
    v.predictive_d_current_old=v.predictive_d_current;
    v.predictive_q_current_old=v.predictive_q_current;
    Deadtime_e_d_deadtime=v.id_Fbk-v.predictive_d_current;
    Deadtime_e_q_deadtime=v.iq_Fbk-v.predictive_q_current;
    // Design observer
    v.predictive_d_current=v.predictive_d_current+ctrl.sample_period/ctrl.para->Ld*(ctrl.inverter->vdq0[0]-v.predictive_d_current*ctrl.para->Rs+v.elec_speed*ctrl.para->Lq*v.iq_Fbk-v.Ud_compensation)+ctrl.sample_period*v.K_lo_1*Deadtime_e_d_deadtime;
    v.predictive_q_current=v.predictive_q_current+ctrl.sample_period/ctrl.para->Lq*(ctrl.inverter->vdq0[1]-v.predictive_q_current*ctrl.para->Rs-v.elec_speed*ctrl.para->Ld*v.id_Fbk-v.elec_speed*ctrl.para->Psi_r-v.Uq_compensation)+ctrl.sample_period*v.K_lo_1*Deadtime_e_q_deadtime;
    v.Ud_compensation=ctrl.sample_period*v.K_lo_2*Deadtime_e_d_deadtime+v.Ud_compensation;
    v.Uq_compensation=ctrl.sample_period*v.K_lo_2*Deadtime_e_q_deadtime+v.Uq_compensation;
    //dq to ab
    v2.Ds =v.Ud_compensation;
    v2.Qs =v.Uq_compensation;
    v2.Sine=v.Sine;
    v2.Cosine=v.Cosine;
    IPARK_Drive(&v2);
    v.Ualpha_compensation=v2.Alpha;
    v.Ubeta_compensation=v2.Beta;
  //v.Ualpha_compensation=v.Cosine*v.Ud_compensation-v.Sine*v.Uq_compensation;
  //v.Ubeta_compensation=v.Sine*v.Ud_compensation+v.Cosine*v.Uq_compensation;
    (*ptrV)=v;
}
#endif
