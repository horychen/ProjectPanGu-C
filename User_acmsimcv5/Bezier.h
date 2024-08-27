#ifndef BEZIER_C
#define BEZIER_C

//#include "typedef.h"
#include "brentq.h"
#include "pid_regulator.h"
//#include <stdio.h>
//#include <stdlib.h>
#include "super_config.h"

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


Point bezier(const REAL *t, const BezierController *BziController);

REAL bezier_x(const REAL *t, const BezierController *BziController);

REAL bezier_y(const REAL *t, const BezierController *BziController);

REAL bezier_x_diff(REAL t, void *params);

REAL find_t_for_given_x(const REAL x, const BezierController *BziController);

REAL find_y_for_given_x(const REAL x, const BezierController *BziController);

void set_points(BezierController *BziController);

void control_output(st_pid_regulator *r, BezierController *BzController);

#if PC_SIMULATION == FALSE
#define SIM_2_EXP_DEFINE_BEZIER_POINTS_X REAL x_tmp[5] = {0, 199.60567246573584, 423.0965698441416, 264.3887865699018, 585.9472944997638};
#define SIM_2_EXP_DEFINE_BEZIER_POINTS_Y REAL y_tmp[5] = {0, 3.565958912210109, 8.017249903070724, 1.328682531561319, 13.66303157657558};
#endif

#endif
