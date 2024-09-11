#include "ACMSim.h"
#include "inverter_Compensation.h"


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

