#include "ACMSim.h"
#include "Bezier.h"
BezierController Bcontroller; // initial a controller
/* 逆变器非线性 */

/* 拟合法 */
#ifdef _XCUBE1
// b-phase @80 V Mini6PhaseIPMInverter
REAL sig_a2 = 7.705938744542915;
REAL sig_a3 = 67.0107635483658;

// 180V (old)
// REAL sig_a2 = 16.0575341;  // yuefei tuning gives a2' = 9.2*2
// REAL sig_a3 = 17.59278688; // yuefei tuning gives a3' = a3*5
#else
// 80 V
// REAL sig_a2 = 6.67159129;
// REAL sig_a3 = 8.42010418;

// 100 V from SlessInv paper
REAL sig_a2 = 6.7;
REAL sig_a3 = 5.6;

// 150 V
// REAL sig_a2 = 13.25723639;
// REAL sig_a3 = 5.6420585;

// 180 V
// REAL sig_a2 = 15.43046115;
// REAL sig_a3 = 5.04010863;
#endif
REAL sigmoid(REAL x)
{
    // beta
    // REAL a1 = 0.0; // *1.28430164
    // REAL a2 = 8.99347107;
    // REAL a3 = 5.37783655;

    // -------------------------------

#ifdef _XCUBE1
    /* Si Mini OW IPM Inverter*/

    // b-phase @180 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //1.34971502
    // REAL a2 = 16.0575341;  // yuefei tuning gives a2' = 9.2*2
    // REAL a3 = 17.59278688; // yuefei tuning gives a3' = a3*5

    // b-phase @80 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //2.10871359 ;
    // REAL a2 = 7.36725304;
    // REAL a3 = 62.98712805;
    REAL a1 = 0.0; //(1.705,
    REAL a2 = sig_a2;
    REAL a3 = sig_a3;

#else
    /* SiC Cree 3 Phase Inverter*/

    // b-phase @20 V
    // REAL a1 = 0.0; //1.25991178;
    // REAL a2 = 2.30505904;
    // REAL a3 = 12.93721814;

    // b-phase @80 V
    // REAL a1 = 0.0; //1.28430164;
    // REAL a2 = 7.78857441;
    // REAL a3 = 6.20979077;
    REAL a1 = 0.0; // 1.6997103;
    REAL a2 = sig_a2;
    // 6.67159129;
    REAL a3 = sig_a3;
    // 8.42010418;

    // b-phase @180 V
    /*最早的拟合结果*/
    // REAL a1 = 0.0; //1.50469916;
    // REAL a2 = 13.48467604;
    // REAL a3 = 5.22150403;
    /*错误！多乘了一次beta-axis转phase系数！*/
    // REAL a1 = 0.0; //1.83573529
    // REAL a2 = sig_a2; 13.36317172;
    // REAL a3 = sig_a3; 5.81981571;
    /*纠正！100个点，平均分配*/
    // REAL a1 = 0.0; //1.8357354;
    // REAL a2 = sig_a2; 15.43046115;
    // REAL a3 = sig_a3; 5.04010863;

#endif

    return a1 * x + a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}
REAL sigmoid_online_v2(REAL x, REAL a2, REAL a3)
{
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}
REAL sigmoid_online(REAL x, REAL Vsat, REAL a3)
{

    REAL a2 = Vsat * 2;
    // REAL a3 = 5.22150403;
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}

/* 真查表法 // C代码参考ui_curve_v4.py */
#define LUT_N_LC 70
#define LUT_N_HC 29
REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
#define LUT_STEPSIZE_BIG 0.11641244037931034
#define LUT_STEPSIZE_SMALL 0.01237159786376811
#define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
#define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
#define LUT_I_TURNING_LC 0.8660118504637677
#define LUT_I_TURNING_HC 4.241972621463768
#define V_PLATEAU 7.43925517763064

REAL lookup_compensation_voltage_indexed(REAL current_value)
{
    REAL abs_current_value = fabsf(current_value);

    if (abs_current_value < LUT_I_TURNING_LC)
    {
        REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
        int index = (int)float_index;
        REAL slope;
        if (index + 1 >= LUT_N_LC)
            slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        else
            slope = (lut_lc_voltage[index + 1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        return signf(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index * LUT_STEPSIZE_SMALL));
    }
    else
    {
        REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
        int index = (int)float_index; // THIS IS A RELATIVE INDEX!
        REAL slope;
        if (index + 1 >= LUT_N_HC)
            return signf(current_value) * V_PLATEAU;
        else
            slope = (lut_hc_voltage[index + 1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
        return signf(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index * LUT_STEPSIZE_BIG));
    }
}
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist)
{

    /* 查表法 */
    if (TRUE)
    {
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);
        REAL dist_uc = lookup_compensation_voltage_indexed(ic);

        G.ia = ia;
        G.dist_ua = dist_ua;

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }
    else
    {
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}

/* 伪查表法（需要循环比大小，运算量不固定） */
REAL lookup_phase_current(REAL current, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    /* assume lut_voltage[0] is negative and lut_voltage[-1] is positive */
    int j;
    if (current < lut_current[0])
    {
        return lut_voltage[0];
    }
    else if (current > lut_current[length_of_lut - 1])
    {
        return lut_voltage[length_of_lut - 1];
    }
    else
    {
        for (j = 0; j < length_of_lut - 1; ++j)
        {
            if (current > lut_current[j] && current < lut_current[j + 1])
            {
                REAL slope = (lut_voltage[j + 1] - lut_voltage[j]) / (lut_current[j + 1] - lut_current[j]);
                return (current - lut_current[j]) * slope + lut_voltage[j];
                // return 0.5*(lut_voltage[j]+lut_voltage[j+1]); // this is just wrong! stupid!
            }
        }
    }
#if PC_SIMULATION
    printf("DEBUG inverter.c. %g, %d, %d\n", current, j, length_of_lut);
#endif
    return 0.0;
}
REAL sigmoid_a3_tune = 1.0;
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current and d-axis voltage.

    /* 拟合法 */
    REAL ia, ib, ic;
    ia = 1 * (ial);
    ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
    ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

    // offline sigmoid
    REAL dist_ua = sigmoid(ia);
    REAL dist_ub = sigmoid(ib);
    REAL dist_uc = sigmoid(ic);

    // online tunable sigmoid (experimental)
    // REAL dist_ua = sigmoid_online(ia, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_ub = sigmoid_online(ib, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_uc = sigmoid_online(ic, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);

    // Clarke transformation（三分之二，0.5倍根号三）
    ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
    ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534

    // for plot
    INV.ia = ia;
    INV.ib = ib;
    INV.ic = ic;
    INV.dist_ua = dist_ua;
    INV.dist_ub = dist_ub;
    INV.dist_uc = dist_uc;
}
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current and d-axis voltage.

    /* 查表法 */
    if (TRUE)
    {
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }
    else
    {
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
        // ic = AB2W_AI(ial, ibe); // ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        // REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        // ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        // ualbe_dist[1] = 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc); // 0.5773502695534

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}

/* Chen 2021 Linear Approximation of U-I Curve */
REAL trapezoidal_voltage_by_phase_current(REAL current, REAL V_plateau, REAL I_plateau, REAL oneOnver_I_plateau)
{
    REAL abs_current = fabsf(current);
    if (abs_current < I_plateau)
    {
        return current * V_plateau * oneOnver_I_plateau;
    }
    else
    {
        return signf(current) * V_plateau;
    }
}

/* ParkSul2012 梯形波 */
void inverterNonlinearity_Initialization()
{
    INV.gamma_theta_trapezoidal = GAIN_THETA_TRAPEZOIDAL;
#ifdef _XCUBE1
    INV.Vsat = 16.0575341 / 2; // 6.67054; // 180 V SiC
#else
    // INV.Vsat = sig_a2*0.5; //6.74233802;
    INV.Vsat = 7.84; // 150 V
#endif

    INV.gain_Vsat = 0 * 10;

    INV.thetaA = 0;
    INV.cos_thetaA = 1;
    INV.sin_thetaA = 0;

    // --
    INV.u_comp[0] = 0;
    INV.u_comp[1] = 0;
    INV.u_comp[2] = 0;
    INV.ual_comp = 0;
    INV.ube_comp = 0;
    INV.uDcomp_atA = 0;
    INV.uQcomp_atA = 0;

    INV.iD_atA = 0;
    INV.iQ_atA = 0;
    INV.I5_plus_I7 = 0;
    INV.I5_plus_I7_LPF = 0.0;
    INV.theta_trapezoidal = 11.0 * M_PI_OVER_180; // in rad

#ifdef _XCUBE1
    INV.I_plateau_Max = 2.0;
    INV.I_plateau_Min = 0.2;
#else
    INV.I_plateau_Max = 1.0;
    INV.I_plateau_Min = 0.1;
#endif
    INV.I_plateau = 0.7;
    INV.V_plateau = 1.5 * sig_a2 * 0.5;
    INV.gamma_I_plateau = 10.0;
    INV.gamma_V_plateau = 0.0; // this is updated by the estimated disturbance to the sinusoidal flux model.

#if PC_SIMULATION
    INV.sig_a2 = 1.0 * sig_a2;
    INV.sig_a3 = 1.5 * sig_a3; // = shape parameter
#else
    INV.sig_a2 = sig_a2; // = Plateau * 2
    INV.sig_a3 = sig_a3; // = shape parameter
#endif

    INV.w6 = 1;
    INV.w12 = 0;
    INV.w18 = 0;
    INV.gamma_a2 = 400;
    INV.gamma_a3 = 125;
}
#define trapezoidal_voltage_by_current_vector_angle u_comp_per_phase
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal)
{
    REAL compensation;

    // if(thetaA>0){
    //         compensation = Vsat;
    // }else{ // thetaA < 0
    //         compensation = -Vsat;
    // }

    if (thetaA > 0)
    {
        if (thetaA < theta_trapezoidal)
        {
            compensation = thetaA * Vsat * oneOver_theta_trapezoidal;
        }
        else if (thetaA > M_PI - theta_trapezoidal)
        {
            compensation = (M_PI - thetaA) * Vsat * oneOver_theta_trapezoidal;
        }
        else
        {
            compensation = Vsat;
        }
    }
    else
    { // thetaA < 0
        if (-thetaA < theta_trapezoidal)
        {
            compensation = -thetaA * -Vsat * oneOver_theta_trapezoidal;
        }
        else if (-thetaA > M_PI - theta_trapezoidal)
        {
            compensation = (M_PI + thetaA) * -Vsat * oneOver_theta_trapezoidal;
        }
        else
        {
            compensation = -Vsat;
        }
    }

    return compensation;
}
REAL lpf1_inverter(REAL x, REAL y_tminus1)
{
    // #define LPF1_RC 0.6 // 0.7, 0.8, 1.2, 3太大滤得过分了 /* 观察I5+I7_LPF的动态情况进行确定 */
    // #define ALPHA_RC (TS/(LPF1_RC + TS)) // TODO：优化
    // return y_tminus1 + ALPHA_RC * (x - y_tminus1); // 0.00020828993959591752
    return y_tminus1 + 0.00020828993959591752 * (x - y_tminus1);
}
REAL shift2pi(REAL thetaA)
{
    if (thetaA > M_PI)
    {
        return thetaA - 2 * M_PI;
    }
    else if (thetaA < -M_PI)
    {
        return thetaA + 2 * M_PI;
    }
    else
    {
        return thetaA;
    }
}
// REAL watch_theta_trapezoidal = 0.0;
void Modified_ParkSul_Compensation(void)
{

    /* Park12/14
     * */
    // Phase A current's fundamental component transformation
    INV.thetaA = -M_PI * 1.5 + (*CTRL).i->theta_d_elec + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    INV.thetaA = shift2pi(INV.thetaA); /* Q: how to handle it when INV.thetaA jumps between pi and -pi? */                            // 这句话绝对不能省去，否则C相的梯形波会出错。

    INV.cos_thetaA = cosf(INV.thetaA);
    INV.sin_thetaA = sinf(INV.thetaA);
    INV.iD_atA = AB2M(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    INV.iQ_atA = AB2T(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);

    INV.I5_plus_I7 = INV.iD_atA * sinf(6 * INV.thetaA);                     /* Q: Why sinf? Why not cosf? 和上面的-1.5*pi有关系吗？ */
    INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF); /* lpf1 for inverter */

    INV.I11_plus_I13 = INV.iD_atA * sinf(12 * INV.thetaA);
    INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

    INV.I17_plus_I19 = INV.iD_atA * sinf(18 * INV.thetaA);
    INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);

    // The adjusting of theta_t via 6th harmonic magnitude
    INV.theta_trapezoidal += CL_TS * INV.gamma_theta_trapezoidal // *fabsf((*CTRL).i->cmd_speed_rpm)
                             * (INV.I5_plus_I7_LPF + 0 * INV.I11_plus_I13_LPF + 0 * INV.I17_plus_I19_LPF);
    // 两种方法选其一：第一种可用于sensorless系统。
    if (FALSE)
    {
        /* 利用theta_t的饱和特性辨识Vsat（次优解） */
    }
    else
    {
        // 由于实际逆变器有一个调制比低引入5、7次谐波的问题，所以最佳theta_t不在11度附近了。
        if (INV.theta_trapezoidal >= 25 * M_PI_OVER_180)
        {                                               // 17
            INV.theta_trapezoidal = 25 * M_PI_OVER_180; // 17
        }
        if (INV.theta_trapezoidal <= 0.223 * M_PI_OVER_180)
        {
            INV.theta_trapezoidal = 0.223 * M_PI_OVER_180;
        }
    }

#if PC_SIMULATION == TRUE
    // 用偏小的Vsat，观察theta_t的收敛是否直达下界？
    // if((*CTRL).timebase>23){
    //     INV.Vsat = 9.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>20){
    //     INV.Vsat = 7.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>15){
    //     INV.Vsat = 6.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>10){
    //     INV.Vsat = 5.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>5){
    //     INV.Vsat = 4.0/6.0 * 16.0575341/2;
    // }

    // /* Adaptive Vsat based on position error */
    // INV.Vsat += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK) * signf(ENC.omg_elec);
    // if (INV.Vsat>15){
    //     INV.Vsat = 15;
    // }else if(INV.Vsat<0){
    //     INV.Vsat = 0;
    // }
#else
    /* Adaptive Vsat based on position error */
    /* TO use this, you muast have a large enough stator current */
    /* TO use this, you muast have a large enough stator current */
    /* TO use this, you muast have a large enough stator current */
    INV.Vsat += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - PMSM_ELECTRICAL_POSITION_FEEDBACK) * signf(ENC.varOmega);
    if (INV.Vsat > 15)
    {
        INV.Vsat = 15;
    }
    else if (INV.Vsat < 0)
    {
        INV.Vsat = 0;
    }
#endif

#if INVERTER_NONLINEARITY == 3 // [ModelLUT]
    // INV.Vsat = 6.67054;
#elif INVERTER_NONLINEARITY == 2 // [ModelSigmoid]
    // INV.Vsat = sigmoid(100)*1.0;
    // INV.Vsat = sigmoid(100)*0.95;
    // INV.Vsat = sigmoid(100)*1.05;
#elif INVERTER_NONLINEARITY == 1 // [ModelSul96]
    REAL TM = _Toff - _Ton - _Tdead + _Tcomp; // Sul1996
    REAL Udist = (_Udc * TM * CL_TS_INVERSE - _Vce0 - _Vd0) / 6.0;
    INV.Vsat = 3 * fabsf(Udist); // 4 = 2*signf(ia) - signf(ib) - signf(ic) when ia is positive and ib/ic is negative
                                 // but Vsat is phase voltage distortion maximum, so we need to add a zero sequence voltage of Udist*(signf(ia) + signf(ib) + signf(ic)),
                                 // so, it is 3 * |Udist| as Vsat.
    static int bool_printed = FALSE;
    if (bool_printed == FALSE)
    {
        printf("\tVsat = %g V; Udist = %g.\n", INV.Vsat, Udist);
        bool_printed = TRUE;
    }
#endif

    REAL oneOver_theta_trapezoidal = 1.0 / INV.theta_trapezoidal;
    // watch_theta_trapezoidal = INV.theta_trapezoidal / M_PI_OVER_180;
    INV.u_comp[0] = u_comp_per_phase(INV.Vsat, INV.thetaA, INV.theta_trapezoidal, oneOver_theta_trapezoidal);
    INV.u_comp[1] = u_comp_per_phase(INV.Vsat, shift2pi(INV.thetaA - TWO_PI_OVER_3), INV.theta_trapezoidal, oneOver_theta_trapezoidal);
    INV.u_comp[2] = u_comp_per_phase(INV.Vsat, shift2pi(INV.thetaA - 2 * TWO_PI_OVER_3), INV.theta_trapezoidal, oneOver_theta_trapezoidal);

    // REAL ia,ib,ic;
    // ia = 1 * (       IS_C(0)                                  );
    // ib = 1 * (-0.5 * IS_C(0) - SIN_DASH_2PI_SLASH_3 * IS_C(1) );
    // ic = 1 * (-0.5 * IS_C(0) - SIN_2PI_SLASH_3      * IS_C(1) );
    // #define MISMATCH_A3 1.1
    // INV.u_comp[0] = sigmoid_online(ia, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);
    // INV.u_comp[1] = sigmoid_online(ib, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);
    // INV.u_comp[2] = sigmoid_online(ic, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);

    /* 相补偿电压Clarke为静止正交坐标系电压。 */
    // 改成恒幅值变换
    INV.ual_comp = 0.66666666667 * (INV.u_comp[0] - 0.5 * INV.u_comp[1] - 0.5 * INV.u_comp[2]);
    INV.ube_comp = 0.66666666667 * 0.86602540378 * (INV.u_comp[1] - INV.u_comp[2]);
    // INV.ual_comp = SQRT_2_SLASH_3      * (INV.u_comp[0] - 0.5*INV.u_comp[1] - 0.5*INV.u_comp[2]); // sqrt(2/3.)
    // INV.ube_comp = 0.70710678118654746 * (                    INV.u_comp[1] -     INV.u_comp[2]); // sqrt(2/3.)*sinf(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    // 区分补偿前的电压和补偿后的电压：
    // (*CTRL).ual, (*CTRL).ube 是补偿前的电压！
    // (*CTRL).ual + INV.ual_comp, (*CTRL).ube + INV.ube_comp 是补偿后的电压！

    // for plotting
    INV.uDcomp_atA = AB2M(INV.ual_comp, INV.ube_comp, INV.cos_thetaA, INV.sin_thetaA);
    INV.uQcomp_atA = AB2T(INV.ual_comp, INV.ube_comp, INV.cos_thetaA, INV.sin_thetaA);
}

void Online_PAA_Based_Compensation(void)
{

    // /* 角度反馈: (*CTRL).i->theta_d_elec or ELECTRICAL_POSITION_FEEDBACK? */
    // INV.thetaA = ELECTRICAL_POSITION_FEEDBACK;
    // // I (current vector amplitude)
    // INV.iD_atA = sqrt(IS_C(0)*IS_C(0) + IS_C(1)*IS_C(1));

    /* 在Park2012中，a相电流被建模成了sin函数，一个sin函数的自变量角度放在正交坐标系下看就是在交轴上的，所以要-1.5*pi */

    // Phase A current's fundamental component transformation
    /* 这里使用哪个角度的关键不在于是有感的角度还是无感的角度，而是你FOC电流控制器（Park变换）用的角度是哪个？ */
    if (debug.SENSORLESS_CONTROL)
    {
        INV.thetaA = -M_PI * 1.5 + PMSM_ELECTRICAL_POSITION_FEEDBACK + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    }
    else
    {
        INV.thetaA = -M_PI * 1.5 + (*CTRL).i->theta_d_elec + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    }
    INV.thetaA = shift2pi(INV.thetaA); /* Q: how to handle it when INV.thetaA jumps between pi and -pi? */ // 这句话绝对不能省去，否则C相的梯形波会出错。

    INV.cos_thetaA = cosf(INV.thetaA);
    INV.sin_thetaA = sinf(INV.thetaA);
    INV.iD_atA = AB2M(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    INV.iQ_atA = AB2T(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);

    if (FALSE)
    {
        /* Use q-axis current in phase A angle (Tentative and Failed) */
        INV.I5_plus_I7 = INV.iQ_atA * cosf(6 * INV.thetaA);
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF);

        INV.I11_plus_I13 = INV.iQ_atA * cosf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iQ_atA * cosf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
    }
    else
    {
        /* Use d-axis current in phase A angle */
        INV.I5_plus_I7 = INV.iD_atA * sinf(6 * INV.thetaA);                     /* Q: Why sinf? Why not cosf? 和上面的-1.5*pi有关系吗？ */
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF); /* lpf1 for inverter */

        INV.I11_plus_I13 = INV.iD_atA * sinf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iD_atA * sinf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
    }

#if PC_SIMULATION
    // INV.gamma_a2 = 0.0;
    // INV.gamma_a3 = 0.0;
#endif

    /* Online Update Sigmoid a3 */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    INV.sig_a3 -= CL_TS * INV.gamma_a3 // *fabsf((*CTRL).i->cmd_speed_rpm)
                  * (INV.w6 * INV.I5_plus_I7_LPF + INV.w12 * INV.I11_plus_I13_LPF + INV.w18 * INV.I17_plus_I19_LPF);

    // (*CTRL).s->Motor_or_Generator = signf((*CTRL).i->omg_elec * (*CTRL).i->cmd_iDQ[1]);
    // (*CTRL).s->Motor_or_Generator = signf((*CTRL).i->cmd_omg_elec);

    /* Online Update Sigmoid a2 */
    if ((*CTRL).timebase > 2)
    {
        /* Sensorless: Adaptive a2 based on flux amplitude error */
        // use linear FE
        // INV.sig_a2 += CL_TS * -100 * AFEOE.output_error_dq[0];

        // use nonlinear (saturation) FE
        INV.sig_a2 += CL_TS * -INV.gamma_a2 * (*CTRL).s->Motor_or_Generator * (MOTOR.KActive - FE.htz.psi_2_ampl_lpf);

        /* Sensored: Adaptive a2 based on position error */
        /* To use this, you must have a large enough stator current */
        /* 这个好像只对梯形波（电流值的函数）有效 ……*/
        // INV.sig_a2 += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK) * (*CTRL).s->Motor_or_Generator;
    }
    if (INV.sig_a2 > 40)
    {
        INV.sig_a2 = 40;
    }
    else if (INV.sig_a2 < 2)
    {
        INV.sig_a2 = 2;
    }

    /* Chen2021: linear approximation of u-i curve */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    // INV.I_plateau += CL_TS * 0 * INV.gamma_I_plateau \
    //                         // *fabsf((*CTRL).i->cmd_speed_rpm)
    //                         *(    1*INV.I5_plus_I7_LPF
    //                             + 0*INV.I11_plus_I13_LPF
    //                             + 0*INV.I17_plus_I19_LPF
    //                          );
    // if(INV.I_plateau > INV.I_plateau_Max){
    //     INV.I_plateau = INV.I_plateau_Max;
    // }else if(INV.I_plateau < INV.I_plateau_Min){
    //     INV.I_plateau = INV.I_plateau_Min;
    // }

    /* Chen2021SlessInv 覆盖 */
    if (INV.gamma_I_plateau != 0)
    {
        REAL ia_cmd = ((*CTRL).o->cmd_iAB[0]);
        REAL ib_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_DASH_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL ic_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL oneOver_I_plateau = 1.0 / INV.I_plateau;
        INV.u_comp[0] = trapezoidal_voltage_by_phase_current(ia_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[1] = trapezoidal_voltage_by_phase_current(ib_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[2] = trapezoidal_voltage_by_phase_current(ic_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);

        /* Online Sigmoid a3 覆盖 覆盖 */
        INV.u_comp[0] = sigmoid_online_v2(ia_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[1] = sigmoid_online_v2(ib_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[2] = sigmoid_online_v2(ic_cmd, INV.sig_a2, INV.sig_a3);
    }

    /* 相补偿电压Clarke为静止正交坐标系电压。 */
    // 改成恒幅值变换
    INV.ual_comp = 0.66666666667 * (INV.u_comp[0] - 0.5 * INV.u_comp[1] - 0.5 * INV.u_comp[2]);
    INV.ube_comp = 0.66666666667 * 0.86602540378 * (INV.u_comp[1] - INV.u_comp[2]);
    // INV.ual_comp = SQRT_2_SLASH_3      * (INV.u_comp[0] - 0.5*INV.u_comp[1] - 0.5*INV.u_comp[2]); // sqrt(2/3.)
    // INV.ube_comp = 0.70710678118654746 * (                    INV.u_comp[1] -     INV.u_comp[2]); // sqrt(2/3.)*sinf(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    // 区分补偿前的电压和补偿后的电压：
    // (*CTRL).ual, (*CTRL).ube 是补偿前的电压！
    // (*CTRL).ual + INV.ual_comp, (*CTRL).ube + INV.ube_comp 是补偿后的电压！
}

// /* --------------------------下面的是从 pmsm_controller.c 挪过来的公用逆变器死区电压辨识&补偿代码 */
//     #define LUT_N_LC  70
//     #define LUT_N_HC  29
//     REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
//     REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
//     #define LUT_STEPSIZE_BIG 0.11641244037931034
//     #define LUT_STEPSIZE_SMALL 0.01237159786376811
//     #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
//     #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
//     #define LUT_I_TURNING_LC 0.8660118504637677
//     #define LUT_I_TURNING_HC 4.241972621463768
//     #define V_PLATEAU 7.43925517763064
// REAL lookup_compensation_voltage_indexed(REAL current_value){
//     REAL abs_current_value = fabsf(current_value);

//     if(abs_current_value < LUT_I_TURNING_LC){
//         REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
//         int index = (int)float_index;
//         REAL slope;
//         if(index+1 >= LUT_N_LC)
//             slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         else
//             slope = (lut_lc_voltage[index+1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         return signf(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL));
//     }else{
//         REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
//         int index = (int)float_index; // THIS IS A RELATIVE INDEX!
//         REAL slope;
//         if(index+1 >= LUT_N_HC)
//             return V_PLATEAU;
//         else
//             slope = (lut_hc_voltage[index+1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
//         return signf(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index*LUT_STEPSIZE_BIG));
//     }
// }
// int test_lookup_compensation_voltage_indexed(){
//     int i=0;
//     while(TRUE){
//         i+=1;
//         printf("%g, %g\n", lookup_compensation_voltage_indexed(0.02*i), 0.02*i);
//         if(i>100)
//             break;
//     }
//     return 0;

// }
