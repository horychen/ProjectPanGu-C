#ifndef BEZIER_C
#define BEZIER_C

#include "typedef.h"
#include "brentq.h"
#include "pid_regulator.h"
#include <stdio.h>
#include <stdlib.h>

#define ARGS_PATH "../args.txt"

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

#endif
