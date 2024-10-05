#ifndef SIMUSER_BEZIER_H
#define SIMUSER_BEZIER_H

#if WHO_IS_USER == USER_BEZIER

#include "typedef.h"
#include "main_switch.h"
#include <stdio.h>
#include <stdlib.h>
#include "super_config.h"



#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct
{
    int funcalls;
    int iterations;
    int error_num;
} scipy_zeros_info;

#define CONVERGED 0
#define SIGNERR -1
#define CONVERR -2
#define EVALUEERR -3
#define INPROGRESS 1

typedef REAL (*callback_type)(REAL, void *);
typedef REAL (*solver_type)(callback_type, REAL, REAL, REAL, REAL,
                            int, void *, scipy_zeros_info *);

REAL brentq(callback_type f, const REAL xa, const REAL xb, scipy_zeros_info *solver_stats, void *func_data_param);


typedef struct
{
    REAL x;
    REAL y;
} Point;

    #if FALSE // 动态分配 points 的内存
    typedef struct
    {
        Point *points;
        int num_points;
        int order;
    } BezierController;
    #else
    #define BEZIER_MEMORY_MAX 15
    typedef struct
    {
        Point points[BEZIER_MEMORY_MAX]; // 暂时先假设最大只有十个点
        int order;
    } BezierController;
    extern BezierController BzController;
    #endif

int Comb(const int n, const int m);


void bezier_controller_run_in_main();
// Point bezier(const REAL *t, const BezierController *pBezier);

//inline
REAL bezier_x(const REAL *t, const BezierController *pBezier);

//inline
REAL bezier_y(const REAL *t, const BezierController *pBezier);

//inline
REAL bezier_x_diff(REAL t, void *params);

REAL find_t_for_given_x(const REAL x, const BezierController *pBezier);

REAL find_y_for_given_x(const REAL x, const BezierController *pBezier);

void set_points(BezierController *pBezier);

void control_output(st_pid_regulator *r, BezierController *BzController);

// 这个被移到自动生成的 super_config.c 里面去了！不要在这里用！
// #if PC_SIMULATION == FALSE
//     #define SIM_2_EXP_DEFINE_BEZIER_POINTS_X REAL x_tmp[5] = {0, 199.60567246573584, 423.0965698441416, 264.3887865699018, 585.9472944997638};
//     #define SIM_2_EXP_DEFINE_BEZIER_POINTS_Y REAL y_tmp[5] = {0, 3.565958912210109, 8.017249903070724, 1.328682531561319, 13.66303157657558};
// #endif

/* 这一行以下是自动覆盖的范围，不要在这里之后加任何新的代码！ */
/* 这一行以下是自动覆盖的范围，不要在这里之后加任何新的代码！ */
/* 这一行以下是自动覆盖的范围，不要在这里之后加任何新的代码！ */

#if PC_SIMULATION==FALSE
// This is for motor "SD80AEA07530-SC3"
#define SIM_2_EXP_DEFINE_BEZIER_POINTS_X REAL x_tmp[5] = {0.0, 7.03127759327284, 0.000125592388861232, 5.284772314746277e-07, 500.0};
#define SIM_2_EXP_DEFINE_BEZIER_POINTS_Y REAL y_tmp[5] = {0.0, 2.9999900504450316, 2.9999984157216186, 2.999999833819057, 3.0};
#endif
#endif
#endif
