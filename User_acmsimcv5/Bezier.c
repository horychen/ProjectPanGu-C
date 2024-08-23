#include "ACMSim.h"
#include "Bezier.h"


/* 管仲焘在用的东西 */
BezierController BzController;


#define BEZIER_NUMBER_OF_POINTS (d_sim.user.bezier_order+1)

/**
 * @struct FindTParams
 * @brief Structure representing the parameters for finding the value of t in a Bezier curve.
 *
 * This structure holds a pointer to a BezierController object and the target x-coordinate value.
 * It is used to pass the necessary parameters for finding the value of t that corresponds to a given x-coordinate
 * on a Bezier curve.
 */
// typedef struct
// {
//     const BezierController *BzierController; ///< Pointer to a BezierController object.
//     REAL target_x;                           ///< Target x-coordinate value.
// } FindTParams;

/**
 * @brief Calculates the binomial coefficient
 * @param n The total number of elements
 * @param m The number of elements to choose
 * @return The binomial coefficient
 */
int Comb(const int n, const int m)
{
    if (m == 0 || m == n)
        return 1;
    return Comb(n - 1, m) + Comb(n - 1, m - 1);
}

/**
 * @brief Calculates the point on the Bezier curve at the given parameter t
 * @param t The parameter value
 * @param BezierController The Bezier controller
 * @return The point on the Bezier curve
 */
Point bezier(const REAL *t, const BezierController *BzierController)
{
    Point re = {0.0, 0.0};
    int i;
    for (i = 0; i < BEZIER_NUMBER_OF_POINTS; i++)
    {
        re.x += BzierController->points[i].x * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
        re.y += BzierController->points[i].y * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
    }
    return re;
}

/**
 * @brief Calculates the x-coordinate on the Bezier curve at the given parameter t
 * @param t The parameter value
 * @param BezierController The Bezier controller
 * @return The x-coordinate on the Bezier curve
 */
REAL bezier_x(const REAL *t, const BezierController *BzierController)
{
    return bezier(t, BzierController).x;
}

/**
 * @brief Calculates the y-coordinate on the Bezier curve at the given parameter t
 * @param t The parameter value
 * @param BezierController The Bezier controller
 * @return The y-coordinate on the Bezier curve
 */
REAL bezier_y(const REAL *t, const BezierController *BzierController)
{
    return bezier(t, BzierController).y;
}

/**
 * Calculates the difference between the x-coordinate of a point on a Bezier curve
 * and a target x-coordinate.
 *
 * @param t The parameter value of the Bezier curve.
 * @param params A pointer to the FindTParams struct containing the BezierController and target_x.
 * @return The difference between the x-coordinate of the Bezier curve point and the target x-coordinate.
 */
// inline REAL bezier_x_diff(REAL t, void *params)
// {
//     FindTParams *p = (FindTParams *)params;
//     REAL bezier_x_val = bezier_x(&t, p->BzierController);
//     return bezier_x_val - p->target_x;
// }

/**
 * @brief Finds the parameter t for the given x-coordinate on the Bezier curve
 * @param x The x-coordinate
 * @param BezierController The Bezier controller
 * @return The parameter t
 */
REAL find_t_for_given_x(const REAL x, const BezierController *BzierController)
{
    FindTParams params;
    params.BzierController = BzierController;
    params.target_x = x;
    scipy_zeros_info solver_stats;
    REAL root = brentq(bezier_x_diff, 0.0f, 1.0f, &solver_stats, &params);
    if (solver_stats.error_num != CONVERGED)
    {
        #if PC_SIMULATION
            printf("Error: %d\n", solver_stats.error_num);
        #endif
    }
    return root;
}

/**
 * @brief Finds the y-coordinate on the Bezier curve for the given x-coordinate
 * @param x The x-coordinate
 * @param BezierController The Bezier controller
 * @return The y-coordinate on the Bezier curve
 */
REAL find_y_for_given_x(const REAL x, const BezierController *BzierController)
{
    REAL t = find_t_for_given_x(x, BzierController);
    return bezier_y(&t, BzierController);
}

/**
 * @brief Initial the control points for the Bezier curve
 * @param BezierController The Bezier controller
 */
#include <errno.h>
#ifndef ARGS_PATH
#define ARGS_PATH "../args.txt"
#endif
void set_points(BezierController *BzierController){

    int i;

    // int terminal = 4;
    // REAL x_tmp[10]= {0.00000, 167.19914134825365, 684.6302061302766, 79.7836670190831,0,0,0,0,0,0};
    // REAL y_tmp[10]= {0.00000, 44.6915324901932, 79.7836670190831, 143.94129962856175,0,0,0,0,0,0};
    // REAL x_tmp[10] = {0, 0.0015036187833379713, 0.02421162125777866, 0.011573628570659907, 0.2351940203872609, 0, 0, 0, 0, 0};
    // REAL y_tmp[10] = {0, 0.06831889454002364, 0.7855265180918924, 0.3982552801401638, 9.004200761259638, 0, 0, 0, 0, 0};
    // REAL x_tmp[] = {0, 0.001483522141517538, 0.3925131729155844, 4.055584989134949, 0.7597112568584444, 5.152056419843189};
    // REAL y_tmp[] = {0, 4.894113784453421, 4.969405552328565, 0.3448187910584064, 8.039303402140806, 9.006185362086445};

    #if FALSE // 动态分配 points 的内存
        BEZIER_NUMBER_OF_POINTS = sizeof(x_tmp) / sizeof(x_tmp[0]);
        BzierController->order = BEZIER_NUMBER_OF_POINTS - 1;
        #if PC_SIMULATION
            printf("num_points: %d\n", BEZIER_NUMBER_OF_POINTS);
        #endif
        BzierController->points = realloc(BzierController->points, sizeof(Point) * BEZIER_NUMBER_OF_POINTS);
    #else
        BzierController->order = d_sim.user.bezier_order;
        #if PC_SIMULATION
            REAL x_tmp[BEZIER_NUMBER_OF_POINTS];
            REAL y_tmp[BEZIER_NUMBER_OF_POINTS];
            FILE *fw; fw = fopen(ARGS_PATH, "r");
            if (fw == NULL)
            {
                #if PC_SIMULATION
                    printf("Error opening file!\n");
                #endif
                exit(1);
            }
            printf("Bezier control points:\n");
            for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i) {
                if (fscanf(fw, "%f,%f\n", &x_tmp[i], &y_tmp[i]) != 2) {
                    #if PC_SIMULATION
                        printf("Error reading line %d\n", i + 1);
                    #endif
                    exit(1);
                }
                #if PC_SIMULATION
                    printf("\t%d, %lf %lf\n", i, x_tmp[i], y_tmp[i]);
                #endif
            }
            fclose(fw);  // 关闭文件
        #else
            SIM_2_EXP_DEFINE_BEZIER_POINTS_X
            SIM_2_EXP_DEFINE_BEZIER_POINTS_Y
        #endif
        for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
            BzierController->points[i].x = x_tmp[i];
            BzierController->points[i].y = y_tmp[i];
        }
    #endif
    if (BzierController->points == NULL)
    {
        errno = ENOMEM;
        return;
    }

}

/**
 * @brief Calculates the control output using Bezier curve interpolation
 * @param r The PID regulator
 * @param BziController The Bezier controller
 */

void control_output(st_pid_regulator *r, BezierController *BziController)
{
    r->Err = r->Ref - r->Fbk;
    REAL error = r->Err;
    // #if PC_SIMULATION printf("error: %lf\n", error); #endif
    if (fabs(error) > BziController->points[BziController->order].x)
    {
        error = copysignf(BziController->points[BziController->order].x, error);
    }
    // #if PC_SIMULATION printf("error after Bezier: %lf\n", error); #endif
    REAL out = find_y_for_given_x(fabs(error), BziController);
    r->OutPrev = r->Out;
    // r->Out = out*( error / (error + 1e-7) );
    r->Out = copysignf(out, error);
    // #if PC_SIMULATION printf("%.1f, %.1f\t", BziController.points[BziController.num_points].x, sign(error)); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\t", r->Out, r->OutLimit, out); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\n", r->Err, r->Ref, r->Fbk); #endif
    r->ErrPrev = r->Err;
    if (r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if (r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;
}
