#include "ACMSim.h"

#if WHO_IS_USER == USER_WB

SpeedInnerLoop SIL_Controller;
wubo_Parameter_mismatch wubo_ParaMis;
wubo_Hit_Wall wubo_HW;


REAL global_id_ampl[NUMBER_OF_FREQUENCY_LEVEL] = {1, 1, 1};
REAL global_id_freq[NUMBER_OF_FREQUENCY_LEVEL] = {1, 0.1, 5e-06};
REAL tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL] = {0, 0, 0};

REAL global_uD_ampl  = 0.0;
REAL global_uD_square_time  = 1; // second
REAL gloval_uD_square_time_sum  = 0.0; //
void NB_MODE_codes(){
    /* Didn't work at moment */
    /* You can make sound */
    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-1] = global_id_freq[NUMBER_OF_FREQUENCY_LEVEL-1] * (*CTRL).timebase;
    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-1] -= (long)tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-1];
    global_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2] = global_id_ampl[NUMBER_OF_FREQUENCY_LEVEL-1] * sinf (2.0 * M_PI * tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-1]);

    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2] = global_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2] * (*CTRL).timebase;
    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2] -= (long)tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2];
    global_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3] = global_id_ampl[NUMBER_OF_FREQUENCY_LEVEL-2] * sinf (2.0 * M_PI * tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-2]);

    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3] = global_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3] * (*CTRL).timebase;
    tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3] -= (long)tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3];
    (*debug).set_id_command = global_id_ampl[NUMBER_OF_FREQUENCY_LEVEL-3] * sinf(2.0 * M_PI * tmp_id_freq[NUMBER_OF_FREQUENCY_LEVEL-3]);
    _onlyFOC(CTRL->i->theta_d_elec, CTRL->i->iAB);
}

void UDQ_GIVEN_TEST(){
    // 帕克变换
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    // Doing the plus to the golbal_uD_square_two_times is stupid cause u will always plus it running the code
    // Hence u got no chance run the else if content ! fuck it
    //* advice from LaoBan: timebase should be integer
    if (gloval_uD_square_time_sum + 4 * global_uD_square_time < (*CTRL).timebase) {
        gloval_uD_square_time_sum += 4 * global_uD_square_time;
    }
    REAL relative_timebase_to_one_cycle = (*CTRL).timebase - gloval_uD_square_time_sum;
    if (relative_timebase_to_one_cycle < global_uD_square_time) {
        (*CTRL).o->cmd_uDQ[0] = global_uD_ampl;
    } else if (relative_timebase_to_one_cycle < global_uD_square_time * 2) {
        (*CTRL).o->cmd_uDQ[0] = 0;
    } else if (relative_timebase_to_one_cycle < global_uD_square_time * 3) {
        (*CTRL).o->cmd_uDQ[0] = -global_uD_ampl;
    } else {
        (*CTRL).o->cmd_uDQ[0] = 0;
    }

    // printf("%f\n", gloval_uD_square_time_sum);
    // (*CTRL).o->cmd_uDQ[0] = global_uD_ampl * sin(2 * M_PI * global_uD_freq * (*CTRL).timebase);
    (*CTRL).o->cmd_uDQ[1] = 0;

    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];
}

/*
Author: Wubo
Algorithm: Six First-Order Curve for Compensation
*/
REAL inverter_current_point[NUMBER_OF_COMPENSATION_POINTS] = {0.85, 0.216, 0.03, 0, -0.019, -0.17, -0.86}; // Unit:A
REAL inverter_voltage_point[NUMBER_OF_COMPENSATION_POINTS] = {4.06, 3.67 , 1.83, 0, -2.36 , -3.58, -4.066};  // Unit:V

void wubo_inverter_Compensation(REAL iAB[2]){
    if ((*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT == 0){
        // No Compensation
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];
    }else {
        REAL uAB_dist[2];
        REAL dist_ua = 0.0;
        REAL dist_ub = 0.0;
        REAL dist_uc = 0.0;

        /* Compensation take iUVW to do the look-up for Distortion Voltage*/
        REAL iu, iv, iw;
        REAL cmd_uAB_dist[2];

        // 将此刻电机的iAB转为iUVW
        iu = AB2U_AI(iAB[0], iAB[1]);
        iv = AB2V_AI(iAB[0], iAB[1]);
        iw = AB2W_AI(iAB[0], iAB[1]);

        /* Calculation for uabc_dist */
        if ((*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT == 1){
            // Six First-Order Curve for Compensation
            dist_ua = wubo_inverter_Compensation_get_dist_voltage(iu);
            dist_ub = wubo_inverter_Compensation_get_dist_voltage(iv);
            dist_uc = wubo_inverter_Compensation_get_dist_voltage(iw);
        }else if ((*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT == 2){
            // 二分查表法 AKA.真查表法
            dist_ua = CJH_LUT_index_inverter_compensation_get_dist_voltage(iu);
            dist_ub = CJH_LUT_index_inverter_compensation_get_dist_voltage(iv);
            dist_uc = CJH_LUT_index_inverter_compensation_get_dist_voltage(iw);
        }
        /* CLARKE and Compensation */
        // we need ADD the distortion voltage to cmd_uAB cuz the distortion voltage plays negative in the phase voltage equation !!!
        // See EE275 notebook (346)
        CTRL->o->cmd_uAB_to_inverter[0] = CTRL->o->cmd_uAB[0] + UVW2A_AI(dist_ua, dist_ub, dist_uc);
        CTRL->o->cmd_uAB_to_inverter[1] = CTRL->o->cmd_uAB[1] + UVW2B_AI(dist_ua, dist_ub, dist_uc);
    }
    /*
    With more compensation methods, we can add more else if here.
    */
}

// REAL inverter_current_point[NUMBER_OF_COMPENSATION_POINTS] = {0.85, 0.216, 0.03, 0, -0.019, -0.17, -0.86};
// REAL inverter_voltage_point[NUMBER_OF_COMPENSATION_POINTS] = {4.06, 3.67, 1.83, 0, -2.36, -3.58, -4.066};

REAL wubo_inverter_Compensation_get_dist_voltage(REAL current){
    REAL slope[NUMBER_OF_COMPENSATION_POINTS - 1];
    REAL cut[NUMBER_OF_COMPENSATION_POINTS - 1];

    // 计算 slope 和 cut
    int i;int j;
    //* 这个两个变量也太浪费空间了吧？
    int flag_get_u_dist = 0;
    REAL voltage_dist = 0.0;

    for (i = 0; i < NUMBER_OF_COMPENSATION_POINTS - 1; i++){
        slope[i] = (inverter_voltage_point[i] - inverter_voltage_point[i + 1]) / (inverter_current_point[i] - inverter_current_point[i + 1]);
        cut[i] = inverter_voltage_point[i] - slope[i] * inverter_current_point[i];
    }

    if (current > inverter_current_point[0]) {
        voltage_dist = inverter_voltage_point[0]; // 找到此时 current 对应的 distortion voltage
    } else if (current < inverter_current_point[NUMBER_OF_COMPENSATION_POINTS - 1]) {
        voltage_dist = inverter_voltage_point[NUMBER_OF_COMPENSATION_POINTS - 1];
    } else { // 开始滑动查表
        for (j = 0; j < NUMBER_OF_COMPENSATION_POINTS - 1; j++) {
            if (inverter_current_point[j + 1] < current && current < inverter_current_point[j] && flag_get_u_dist == 0) {
                voltage_dist = slope[j] * current + cut[j]; // 找到此时 current 对应的 distortion voltage
                flag_get_u_dist = 1; // 成功找到 voltage_dist
            }
        }
    }
    return voltage_dist;
}


/*
Author: CJH
Algorithm: LUT index inverter compensation AKA.真查表法
with _2 stands for inverter compensation use
*/

#define LUT_N_LC_2  70
#define LUT_N_HC_2  29
REAL lut_lc_voltage_2[70] = {0, 1.8746, 2.144, 2.34136, 2.49419, 2.60973, 2.74337, 2.88097, 3.00094, 3.10985, 3.23057, 3.34807, 3.4332, 3.50093, 3.55079, 3.59368, 3.61982, 3.64135, 3.65953, 3.67652, 3.69216, 3.7051, 3.71887, 3.73073, 3.73901, 3.75071, 3.75723, 3.76842, 3.77575, 3.78264, 3.78858, 3.79489, 3.80023, 3.80754, 3.8125, 3.81981, 3.82407, 3.8291, 3.83278, 3.83879, 3.84318, 3.84597, 3.85316, 3.85658, 3.85856, 3.86383, 3.86556, 3.86962, 3.87269, 3.87618, 3.87955, 3.88147, 3.88496, 3.88746, 3.88879, 3.89325, 3.89452, 3.8973, 3.89922, 3.9023, 3.90339, 3.90545, 3.90747, 3.90848, 3.91075, 3.91321, 3.915, 3.917, 3.91789, 3.91824};
REAL lut_hc_voltage_2[29] = {3.92009, 3.96505, 4.00084, 4.02508, 4.04373, 4.05894, 4.07484, 4.08679, 4.09461, 4.10467, 4.11101, 4.11741, 4.12254, 4.12758, 4.1317, 4.13569, 4.14004, 4.14258, 4.1448, 4.14871, 4.14937, 4.15295, 4.15194, 4.15295, 4.15107, 4.15136, 4.15206, 4.1492, 4.14973};
#define LUT_STEPSIZE_BIG_2 0.3796966241724138
#define LUT_STEPSIZE_SMALL_2 0.012110000369565214
#define LUT_STEPSIZE_BIG_INVERSE_2 2.633681566644419
#define LUT_STEPSIZE_SMALL_INVERSE_2 82.5763806344048
#define LUT_I_TURNING_LC_2 0.847700025869565
#define LUT_I_TURNING_HC_2 11.858902126869566
#define V_PLATEAU_2 4.149734790202423

REAL CJH_LUT_index_inverter_compensation_get_dist_voltage(REAL current_value){
    REAL abs_current_value = fabs(current_value);
        if(abs_current_value < LUT_I_TURNING_LC_2){
            REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE_2;
            int index = (int)float_index;
            REAL slope;
            if(index+1 >= LUT_N_LC_2)
                slope = (lut_hc_voltage_2[0] - lut_lc_voltage_2[index]) * LUT_STEPSIZE_SMALL_INVERSE_2;
            else
                slope = (lut_lc_voltage_2[index+1] - lut_lc_voltage_2[index]) * LUT_STEPSIZE_SMALL_INVERSE_2;
            return sign(current_value) * (lut_lc_voltage_2[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL_2));
        }else{
            REAL float_index = (abs_current_value - LUT_I_TURNING_LC_2) * LUT_STEPSIZE_BIG_INVERSE_2;
            int index = (int)float_index; // THIS IS A RELATIVE INDEX!
            REAL slope;
            if(index+1 >= LUT_N_HC_2)
                return sign(current_value) * V_PLATEAU_2;
            else
                slope = (lut_hc_voltage_2[index+1] - lut_hc_voltage_2[index]) * LUT_STEPSIZE_BIG_INVERSE_2;
            return sign(current_value) * (lut_hc_voltage_2[index] + slope * (abs_current_value - LUT_I_TURNING_LC_2 - index*LUT_STEPSIZE_BIG_2));
        }
}



//TODO:这个变量需要删除
/* For wubo debuger */
REAL wubo_debug_tools[10] = {1,0,0,0,0,0,0,0,0,0};

/* User Tuner */
REAL _init_WC_Tuner_Part2(REAL zeta, REAL omega_n, REAL max_CLBW_PER_min_CLBW){
    REAL Ld  = d_sim.init.Ld * wubo_ParaMis.percent_Ld;
    REAL Lq  = d_sim.init.Lq * wubo_ParaMis.percent_Lq;
    REAL R   = d_sim.init.R  * wubo_ParaMis.percent_Rs;
    REAL Js  = d_sim.init.Js * wubo_ParaMis.percent_Js;
    REAL npp = d_sim.init.npp                         ;
    REAL KE  = d_sim.init.KE * wubo_ParaMis.percent_KE;
    
    // motor parameters
    REAL KT = 1.5 * npp * KE;  // torque constant
    REAL K0 = KT / Js;  // motor constant

    REAL max_CLBW = zeta * omega_n * 4; REAL min_CLBW = zeta * omega_n * 2;

    // 通过一个小于1的比例系数来选取电流环带宽
    REAL FOC_CLBW = max_CLBW_PER_min_CLBW * max_CLBW +  (1-max_CLBW_PER_min_CLBW) * min_CLBW;
    //FOC_VLBW = d_sim['user.omega_n'] * np.sqrt(1 - 2 * d_sim['user.zeta'] **2 + np.sqrt(4 * d_sim['user.zeta'] ** 4 - 4 * d_sim['user.zeta'] ** 2 + 2))
    REAL FOC_VLBW = omega_n * sqrt(1 - 2 * zeta*zeta + sqrt(4 * zeta*zeta*zeta*zeta - 4 * zeta*zeta + 2));

    REAL Series_D_KP = FOC_CLBW * Ld; REAL Series_D_KI = R / Ld;
    REAL Series_Q_KP = FOC_CLBW * Lq; REAL Series_Q_KI = R / Lq;
    REAL Series_Speed_KP = omega_n * omega_n / (FOC_CLBW * K0);
    REAL Series_Speed_KFB = (2*zeta*omega_n*FOC_CLBW - 4*zeta*zeta*omega_n*omega_n) / (FOC_CLBW*K0);
    REAL Series_Speed_KI = ( FOC_CLBW - (sqrt(FOC_CLBW * FOC_CLBW - 4*FOC_CLBW*K0*Series_Speed_KFB)) ) * 0.5;

    PID_iD->Kp = Series_D_KP;
    PID_iQ->Kp = Series_Q_KP;
    PID_Speed->Kp = Series_Speed_KP;

    #if CURRENT_LOOP_KI_TIMES_TEN
        PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS * 10;
        PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS * 10;
    #else
        PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS;
        PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS;
    #endif
    
    PID_Speed->Ki_CODE = Series_Speed_KI * Series_Speed_KP * VL_TS;
    SIL_Controller.KFB = Series_Speed_KFB;

    SIL_Controller.FOC_CLBW = FOC_CLBW;
    SIL_Controller.FOC_VLBW = FOC_VLBW;

    // #if PC_SIMULATION == TRUE
    //     printf("Dual Loop Theoritical Bandwidth is: \n");
    //     printf("FOC_VLBW = %f rad/s\n", FOC_VLBW);
    //     printf("FOC_VLBW = %f Hz\n", FOC_VLBW * ONE_OVER_2PI);
    //     printf("FOC_CLBW = %f rad/s\n", FOC_CLBW);
    //     printf("FOC_CLBW = %f Hz\n", FOC_CLBW * ONE_OVER_2PI);
    //     printf("K0       = %f\n",       K0);
    // #endif

    return  K0;
}
void _init_WC_Tuner(){
    /* Tuned by d_sim */
    //* Due to the fact that TI tuner is running at Python at momment hence our tuner should use paramter to run the tuning process
    REAL K0 = 0.0;
    K0 = _init_WC_Tuner_Part2(d_sim.user.zeta, d_sim.user.omega_n, d_sim.user.max_CLBW_PER_min_CLBW);
    #if PC_SIMULATION == TRUE
        printf("FOC_VLBW = %f rad/s\n", SIL_Controller.FOC_VLBW);
        printf("FOC_VLBW = %f Hz\n", SIL_Controller.FOC_VLBW * ONE_OVER_2PI);
        printf("FOC_CLBW = %f rad/s\n", SIL_Controller.FOC_CLBW);
        printf("FOC_CLBW = %f Hz\n", SIL_Controller.FOC_CLBW * ONE_OVER_2PI);
        printf("K0       = %f\n",       K0);
        if (SIL_Controller.FOC_CLBW - 4 * K0 * SIL_Controller.KFB < 0){
            printf("can not do zero-pole cancellation\n");
        }else{
            printf(">>> Zero-pole cancellation can be done <<<\n");
        }
    #endif
    PID_iD->OutLimit    = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_iQ->OutLimit    = 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc;
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;
    // >>实验<<限幅的部分我放在pangu-c的main.c中的measurement函数里面，也就是说，测量此时的Vdc，然后根据Vdc觉得限幅的大小

    // see when codes run here
    extern REAL wubo_debug_tools[10];
    wubo_debug_tools[2] = 99;
}
void _user_wubo_WC_Tuner_Online(){

    REAL FOC_CLBW, K0;
    FOC_CLBW, K0 = _init_WC_Tuner_Part2((*debug).zeta, (*debug).omega_n, (*debug).max_CLBW_PER_min_CLBW);
    // >>实验<<限幅的部分我放在pangu-c的main.c中的measurement函数里面，也就是说，测量此时的Vdc，然后根据Vdc觉得限幅的大小

    // see when codes run here
    extern REAL wubo_debug_tools[10];
    wubo_debug_tools[3] = 99;
}
void _user_wubo_TI_Tuner_Online(){
    /* Tuned by debug */
    REAL CLBW_HZ = (*debug).CLBW_HZ;
    REAL delta = (*debug).delta;

    // TODO: 下面的psi_A在IM上应该不能用，因为我拿的是电机参数中原始的KE
    REAL Ld = d_sim.init.Ld;
    REAL Lq = d_sim.init.Lq;
    REAL R = d_sim.init.R;
    REAL Js = d_sim.init.Js;
    REAL npp = d_sim.init.npp;
    REAL KE = d_sim.init.KE;

    // motor parameters
    REAL KT = 1.5 * npp * KE;  // torque constant
    REAL K0 = KT / Js;  // motor constant
    
    REAL Series_D_KP = CLBW_HZ * 2 * M_PI * Ld;
    REAL Series_D_KI = R / Ld;
    REAL Series_Q_KP = CLBW_HZ * 2 * M_PI * Lq;
    REAL Series_Q_KI = R / Lq;
    REAL Series_Speed_KI = 2* M_PI * CLBW_HZ / (delta * delta); //THIS IS INTEGRAL GAIN
    REAL Series_Speed_KP = delta * Series_Speed_KI / KT * Js;

    PID_iD->Kp = Series_D_KP;
    PID_iQ->Kp = Series_Q_KP;
    PID_Speed->Kp = Series_Speed_KP;

    #if CURRENT_LOOP_KI_TIMES_TEN
        PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS * 10;
        PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS * 10;
    #else
        PID_iD->Ki_CODE = Series_D_KI * Series_D_KP * CL_TS;
        PID_iQ->Ki_CODE = Series_Q_KI * Series_Q_KP * CL_TS;
    #endif
    
    PID_Speed->Ki_CODE = Series_Speed_KI * Series_Speed_KP * VL_TS;

    // see when codes run here
    extern REAL wubo_debug_tools[10];
    wubo_debug_tools[4] = 99;
}

/* User Controller */
void _user_wubo_SpeedInnerLoop_controller(st_pid_regulator *r, SpeedInnerLoop *r_IL){
        //* 存储控制器的各项输出
        r->P_Term += r->Kp * ( r->Err - r->ErrPrev );
        r->I_Term += r->Ki_CODE * r->Err;
        r_IL->KFB_Term = r_IL->KFB * r->Fbk;

        r->Err = r->Ref - r->Fbk;
        r->Out = r->OutPrev + \
                r->Kp * ( r->Err - r->ErrPrev ) \
                + r->Ki_CODE * r->Err; // - r_IL->KFB_Term;
                
        //* 这里对Out做限幅是为了后面给OutPrev的值时，make sure it is able to cover the speed steady error carried by KFB
        if(r->Out > r->OutLimit + r_IL->KFB_Term)
            r->Out = r->OutLimit + r_IL->KFB_Term;
        else if(r->Out < -r->OutLimit + r_IL->KFB_Term)
            r->Out = -r->OutLimit + r_IL->KFB_Term;

        r->ErrPrev = r->Err; 
        r->OutPrev = r->Out;

        r->Out = r->Out - r_IL->KFB_Term;
        if(r->Out > r->OutLimit)
            r->Out = r->OutLimit;
        else if(r->Out < -r->OutLimit)
            r->Out = -r->OutLimit;
}

/* DPCC */
void DPCC(REAL cmd_idq[2], REAL idq[2]){
    (*CTRL).o->cmd_uDQ[0] = 0.0;
    (*CTRL).o->cmd_uDQ[1] = 0.0;
}

/* Auto MISMATCH the Parameter with CTRL.timebase */
void _init_wubo_ParaMis() {
    //* Maximum and Minimum value of Parameter Mismatch
    REAL max = wubo_ParaMis.percent_max = d_sim.user.ParaMis_percent_max;
    REAL min = wubo_ParaMis.percent_min = d_sim.user.ParaMis_percent_min;

    // Initialization of the parameter mismatch
    int i;
    REAL period_percent = ( max - min ) * TOTAL_PARAMETER_MISMATCH_PERIOD_INV;
    for (i = 0; i <= TOTAL_PARAMETER_MISMATCH_PERIOD - 1; i++) {
        wubo_ParaMis.percent_Para[i] = min + period_percent * (REAL)i;
    }
    wubo_ParaMis.percent_Ld   = 1.0;
    wubo_ParaMis.percent_Lq   = 1.0;
    wubo_ParaMis.percent_Rs   = 1.0;
    wubo_ParaMis.percent_Js   = 1.0;
    wubo_ParaMis.percent_KE   = 1.0;

    // initialization of the timebase
    wubo_ParaMis.total_exp_time = 0.0;
}
void _wubo_ParaMis_asTime(){
    REAL total_sim_time_to_one_cycle = CL_TS * d_sim.sim.NUMBER_OF_STEPS;
    // REAL total_sim_time_to_one_cycle = 0.5;
    REAL period_time = total_sim_time_to_one_cycle * TOTAL_PARAMETER_MISMATCH_PERIOD_INV; // the number of the time period equals to Marco
    
    /* The cycle should be only seen at the Real EXP*/
    /* Parameter Sensitivity Speed Reference Given */
    if (wubo_ParaMis.total_exp_time + total_sim_time_to_one_cycle < (*CTRL).timebase) {
        wubo_ParaMis.total_exp_time += total_sim_time_to_one_cycle;
    }
    REAL relative_timebase_to_one_cycle = (*CTRL).timebase - wubo_ParaMis.total_exp_time;

    if ( relative_timebase_to_one_cycle < period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 0);
        // printf("Im here 0!\n");
        //* At the first half of one cycle give a positive speed ref then zero at the last half
        if (      relative_timebase_to_one_cycle < 0.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < period_time )       (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 2 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 1);
        // printf("Im here 1!\n");
        if (      relative_timebase_to_one_cycle < 1.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 2 * period_time   ) (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 3 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 2);
        // printf("Im here 2!\n");
        if (      relative_timebase_to_one_cycle < 2.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 3 * period_time   ) (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 4 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 3);
        // printf("Im here 3!\n");
        if (      relative_timebase_to_one_cycle < 3.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 4 * period_time   ) (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 5 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 4);
        // printf("Im here 4!\n");
        if (      relative_timebase_to_one_cycle < 4.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 5 * period_time   ) (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 6 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 5);
        // printf("Im here 5!\n");
        if (      relative_timebase_to_one_cycle < 5.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 6 * period_time   ) (*CTRL).i->cmd_varOmega = 0;

    } else if ( relative_timebase_to_one_cycle < 7 * period_time) {
        HANDLE_PARAMETER_MISMATCH(d_sim.user.ParaMis_Mode_Select, 6);
        // printf("Im here 6!\n");
        if (      relative_timebase_to_one_cycle < 6.5 * period_time ) (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
        else if ( relative_timebase_to_one_cycle < 7 * period_time   ) (*CTRL).i->cmd_varOmega = 0;
    } else {
        wubo_ParaMis.percent_Ld   = 1.0;
        wubo_ParaMis.percent_Lq   = 1.0;
        wubo_ParaMis.percent_Rs   = 1.0;
        wubo_ParaMis.percent_Js   = 1.0;
        wubo_ParaMis.percent_KE   = 1.0;
        (*CTRL).i->cmd_varOmega = 0;
        #if PC_SIMULATION == TRUE
            printf("Wtf why u here?\n");
        #endif
    }
}


/* SIGNAL GENERATOR */
wubo_SignalGenerator wubo_SG = {
    .idq_amp          = {1.0, 1.0},
    .idq_freq         = {500, 500.0}, 
    .speed_amp        = 100,
    .speed_freq       = 80,
    .squareWave_amp   = 50.0,
    .squareWave_quarter_cycle = 0.5,
    .squareWave_total_time = 0.0,
    .signal_out = 0.0
}; // define with initilization
REAL wubo_Signal_Generator(int signal_mode){
    REAL signal_series = 0;
    switch (signal_mode){
    case GENERATE_D_CURRENT_SINE:
        wubo_SG.signal_out = wubo_SG.idq_amp[0] * sin(2 * M_PI * wubo_SG.idq_freq[0] * (*CTRL).timebase);
        break;
    case GENERATE_Q_CURRENT_SINE:
        wubo_SG.signal_out = wubo_SG.idq_amp[1] * sin(2 * M_PI * wubo_SG.idq_freq[1] * (*CTRL).timebase);
        break;
    case GENERATE_SPEED_SINE:
        wubo_SG.signal_out = wubo_SG.speed_amp * sin(2 * M_PI * wubo_SG.speed_freq * (*CTRL).timebase);
        break;
    case GENERATE_SPEED_SAUARE_WAVE_WITH_INV:
        if (wubo_SG.squareWave_total_time + 4 * wubo_SG.squareWave_quarter_cycle < (*CTRL).timebase) {
                wubo_SG.squareWave_total_time += 4 * wubo_SG.squareWave_quarter_cycle;
            }
            REAL relative_timebase_to_one_cycle = (*CTRL).timebase - wubo_SG.squareWave_total_time;
            if (relative_timebase_to_one_cycle < wubo_SG.squareWave_quarter_cycle) {
                wubo_SG.signal_out = wubo_SG.squareWave_amp;
            } else if (relative_timebase_to_one_cycle < wubo_SG.squareWave_quarter_cycle * 2) {
                wubo_SG.signal_out = 0;
            } else if (relative_timebase_to_one_cycle < wubo_SG.squareWave_quarter_cycle * 3) {
                wubo_SG.signal_out = -wubo_SG.squareWave_amp;;
            } else {
                wubo_SG.signal_out = 0;
            }
            break;
    default:
        wubo_SG.signal_out = 0.0;
        break;
    }
    return wubo_SG.signal_out;
}

void _user_wubo_Sweeping_Command(){
    // #if PC_SIMULATION //* 先让扫频只在simulation中跑
        // ACM.TLoad = 0; // 强制将负载设置为0
        if ((*CTRL).timebase > (*debug).CMD_SPEED_SINE_END_TIME){
            // next frequency
            (*debug).CMD_SPEED_SINE_HZ += (*debug).CMD_SPEED_SINE_STEP_SIZE;
            // next end time
            (*debug).CMD_SPEED_SINE_LAST_END_TIME = (*debug).CMD_SPEED_SINE_END_TIME;
            (*debug).CMD_SPEED_SINE_END_TIME += 1.0/(*debug).CMD_SPEED_SINE_HZ; // 1.0 Duration for each frequency
            // WUBO：这里扫频的思路是每隔一个frequency的时间，扫频信号提升1个Hz，例如1Hz的信号走1s，1/2Hz的信号走0.5s，1/3Hz的信号走0.33s，
            // 依次类推，所以单单从图上看时间是不能直接看出 Bandwidth到底是多少，但是可以用级数求和直接表示？
        }
        if ((*debug).CMD_SPEED_SINE_HZ > (*debug).CMD_SPEED_SINE_HZ_CEILING){
            (*CTRL).i->cmd_varOmega = 0.0; // 到达扫频的频率上限，速度归零
            (*debug).set_id_command = 0.0;
        }else{
            if ((*debug).bool_sweeping_frequency_for_speed_loop == TRUE){
                (*CTRL).i->cmd_varOmega = RPM_2_MECH_RAD_PER_SEC * (*debug).CMD_SPEED_SINE_RPM * sin(2* M_PI *(*debug).CMD_SPEED_SINE_HZ*((*CTRL).timebase - (*debug).CMD_SPEED_SINE_LAST_END_TIME));
            }else{
                // 让电机转起来，然后在d轴上电流扫频？
                //* 所以让电机转起来的原因是？？？
                // (*debug).bool_Null_D_Control = FALSE; //确保Null iD不开启
                // if (FALSE)
                //     (*CTRL).i->cmd_varOmega = (*debug).CMD_SPEED_SINE_RPM * RPM_2_MECH_RAD_PER_SEC;
                // else
                //     (*CTRL).i->cmd_varOmega = 0;
                if (d_sim.user.bool_sweeping_frequency_for_current_loop_iD == TRUE){
                    (*CTRL).i->cmd_iDQ[0] = (*debug).CMD_CURRENT_SINE_AMPERE * sin(2* M_PI *(*debug).CMD_SPEED_SINE_HZ*((*CTRL).timebase - (*debug).CMD_SPEED_SINE_LAST_END_TIME));
                    (*CTRL).i->cmd_iDQ[1] = 0.0;
                } else {
                    (*CTRL).i->cmd_iDQ[0] = 0.0;
                    (*CTRL).i->cmd_iDQ[1] = (*debug).CMD_CURRENT_SINE_AMPERE * sin(2* M_PI *(*debug).CMD_SPEED_SINE_HZ*((*CTRL).timebase - (*debug).CMD_SPEED_SINE_LAST_END_TIME));
                }
            }
        }
    // #endif
}

/* HIT WALL  */
void _init_wubo_Hit_Wall(){
    wubo_HW.time_interval = 2.0;
    REAL max = 2.0;
    REAL min = 0.1;
    int i;

    for ( i = 0; i < NUMBER_OF_HIT_WALL_VAR_RATIO; i++ ){
        wubo_HW.Vdc_limit_ratio[i] = ( 2.0 - 0.1) * NUMBER_OF_HIT_WALL_VAR_RATIO_INV * i;
        wubo_HW.Motor_Current_limit_ratio[i] = ( 2.0 - 0.1) * NUMBER_OF_HIT_WALL_VAR_RATIO_INV * i;
    }
}






#endif