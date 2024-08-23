#ifndef BEZIER_C
#define BEZIER_C

#include "typedef.h"
#include "brentq.h"
#include "pid_regulator.h"
#include <stdio.h>
#include <stdlib.h>
#include "super_config.h"


typedef struct
{
    REAL x;
    REAL y;
} Point;


#if FALSE // 动态分配 points 的内存
    typedef struct{
        Point *points;
        int num_points;
        int order;
    } BezierController;
#else
    #define BEZIER_MEMORY_MAX 15
    typedef struct{
        Point points[BEZIER_MEMORY_MAX]; // 暂时先假设最大只有十个点
        int order;
    } BezierController;
    extern BezierController BzController;
#endif

int Comb(const int n, const int m);

Point bezier(const REAL *t, const BezierController *BziController);

REAL bezier_x(const REAL *t, const BezierController *BziController);

REAL bezier_y(const REAL *t, const BezierController *BziController);

REAL bezier_x_diff(REAL t, void *params);

REAL find_t_for_given_x(const REAL x, const BezierController *BziController);

REAL find_y_for_given_x(const REAL x, const BezierController *BziController);

void set_points(BezierController *BziController);

void control_output(st_pid_regulator *r, BezierController *BzController);

#if PC_SIMULATION == FALSE
    #define SIM_2_EXP_DEFINE_BEZIER_POINTS_X REAL x_tmp[6] = {0, 13.56675106497797, 17.756252135875275, 0.656028337190837, 71.84660306428572, 391.3541988019523};
    #define SIM_2_EXP_DEFINE_BEZIER_POINTS_Y REAL y_tmp[6] = {0, 0.3927496750420234, 7.4062535504606775, 11.734194267544186, 2.3169646206845327, 16.786731372207594};
#endif

#endif
