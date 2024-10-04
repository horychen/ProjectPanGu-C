#include "ACMSim.h"
#include "simuser_bezier.h"
#if WHO_IS_USER == USER_BEZIER

/* -----------------Brentq-------------*/
REAL brentq(callback_type f, const REAL xa, const REAL xb, scipy_zeros_info *solver_stats, void *func_data_param){
    const REAL xtol = 1e-7;
    const REAL rtol = 1e-7;
    const int32 iter = 100;
    REAL xpre = xa, xcur = xb;
    REAL xblk = 0., fpre, fcur, fblk = 0., spre = 0., scur = 0., sbis;
    /* the tolerance is 2*delta */
    REAL delta;
    REAL stry, dpre, dblk;
    int16 i;

    fpre = (*f)(xpre, func_data_param);
    fcur = (*f)(xcur, func_data_param);

    solver_stats->error_num = INPROGRESS;

    solver_stats->funcalls = 2;
    if (fpre == 0)
    {
        solver_stats->error_num = CONVERGED;
        return xpre;
    }

    if (fcur == 0)
    {
        solver_stats->error_num = CONVERGED;
        return xcur;
    }
    if (signbit(fpre) == signbit(fcur))
    {

        // printf("fpre: %f,fcur: %f\n", fpre, fcur);
        solver_stats->error_num = SIGNERR;
        return 0.;
    }
    solver_stats->iterations = 0;
    for (i = 0; i < iter; i++)
    {
        solver_stats->iterations++;
        if (fpre != 0 && fcur != 0 &&
            (signbit(fpre) != signbit(fcur)))
        {
            xblk = xpre;
            fblk = fpre;
            spre = scur = xcur - xpre;
        }
        if (fabsf(fblk) < fabsf(fcur))
        {
            xpre = xcur;
            xcur = xblk;
            xblk = xpre;

            fpre = fcur;
            fcur = fblk;
            fblk = fpre;
        }

        delta = (xtol + rtol * fabsf(xcur)) / 2;
        sbis = (xblk - xcur) / 2;
        if (fcur == 0 || fabsf(sbis) < delta)
        {
            solver_stats->error_num = CONVERGED;
            return xcur;
        }

        if (fabsf(spre) > delta && fabsf(fcur) < fabsf(fpre))
        {
            if (xpre == xblk)
            {
                /* interpolate */
                stry = -fcur * (xcur - xpre) / (fcur - fpre);
            }
            else
            {
                /* extrapolate */
                dpre = (fpre - fcur) / (xpre - xcur);
                dblk = (fblk - fcur) / (xblk - xcur);
                stry = -fcur * (fblk * dblk - fpre * dpre) / (dblk * dpre * (fblk - fpre));
            }
            if (2 * fabsf(stry) < MIN(fabsf(spre), 3 * fabsf(sbis) - delta))
            {
                /* good short step */
                spre = scur;
                scur = stry;
            }
            else
            {
                /* bisect */
                spre = sbis;
                scur = sbis;
            }
        }
        else
        {
            /* bisect */
            spre = sbis;
            scur = sbis;
        }

        xpre = xcur;
        fpre = fcur;
        if (fabsf(scur) > delta)
        {
            xcur += scur;
        }
        else
        {
            xcur += (sbis > 0 ? delta : -delta);
        }

        fcur = (*f)(xcur, func_data_param);
        solver_stats->funcalls++;
    }
    solver_stats->error_num = CONVERR;
    return xcur;
}

/*------------------Bezier--------------*/

BezierController BzController;

#define BEZIER_NUMBER_OF_POINTS (d_sim.user.bezier_order + 1)

/**
 * @struct FindTParams
 * @brief Structure representing the parameters for finding the value of t in a Bezier curve.
 *
 * This structure holds a pointer to a BezierController object and the target x-coordinate value.
 * It is used to pass the necessary parameters for finding the value of t that corresponds to a given x-coordinate
 * on a Bezier curve.
 */
typedef struct
{
    const BezierController *BzierController; ///< Pointer to a BezierController object.
    REAL target_x;                           ///< Target x-coordinate value.
} FindTParams;

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
// Point bezier(const REAL *t, const BezierController *BzierController)
// {
//     Point re = {0.0, 0.0};
//     int i;
//     for (i = 0; i < BEZIER_NUMBER_OF_POINTS; i++)
//     {
//         re.x += BzierController->points[i].x * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
//         re.y += BzierController->points[i].y * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
//     }
//     return re;
// }

/**
 * @brief Calculates the x-coordinate on the Bezier curve at the given parameter t
 * @param t The parameter value
 * @param BezierController The Bezier controller
 * @return The x-coordinate on the Bezier curve
 */
//inline
REAL bezier_x(const REAL *t, const BezierController *BzierController)
{
    int i;
    REAL re = 0.0;
    for (i = 0; i < BEZIER_NUMBER_OF_POINTS; i++)
    {
        re += BzierController->points[i].x * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
    }
    return re;
}

/**
 * @brief Calculates the y-coordinate on the Bezier curve at the given parameter t
 * @param t The parameter value
 * @param BezierController The Bezier controller
 * @return The y-coordinate on the Bezier curve
 */
//inline
REAL bezier_y(const REAL *t, const BezierController *BzierController)
{
    int i;
    REAL re = 0.0;
    for (i = 0; i < BEZIER_NUMBER_OF_POINTS; i++)
    {
        re += BzierController->points[i].y * (REAL)Comb(BEZIER_NUMBER_OF_POINTS - 1, i) * powf(1 - *t, BEZIER_NUMBER_OF_POINTS - 1 - i) * powf(*t, i);
    }
    return re;
}

/**
 * Calculates the difference between the x-coordinate of a point on a Bezier curve
 * and a target x-coordinate.
 *
 * @param t The parameter value of the Bezier curve.
 * @param params A pointer to the FindTParams struct containing the BezierController and target_x.
 * @return The difference between the x-coordinate of the Bezier curve point and the target x-coordinate.
 */
//inline
REAL bezier_x_diff(REAL t, void *params)
{
    FindTParams *p = (FindTParams *)params;
    REAL bezier_x_val = bezier_x(&t, p->BzierController);
    return bezier_x_val - p->target_x;
}

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
#define ARGS_PATH "../plugin_moo_args.txt"
#endif
void set_points(BezierController *pBezier){

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
        pBezier->order = BEZIER_NUMBER_OF_POINTS - 1;
        #if PC_SIMULATION
            printf("num_points: %d\n", BEZIER_NUMBER_OF_POINTS);
        #endif
        pBezier->points = realloc(pBezier->points, sizeof(Point) * BEZIER_NUMBER_OF_POINTS);
    #else
        pBezier->order = d_sim.user.bezier_order;
        #if PC_SIMULATION
            if(BEZIER_NUMBER_OF_POINTS>10){
                if(d_sim.user.verbose)printf("TOO MANY BEZIER POINTS!!!");
            }
            REAL x_tmp[10];
            REAL y_tmp[10];
            FILE *fw;
            fw = fopen(ARGS_PATH, "r");
            if (fw == NULL){
                if(d_sim.user.verbose)printf("Error opening file!\n");
                exit(1);
            }
            if(d_sim.user.verbose)printf("Bezier control points:\n");
            for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
                if (fscanf(fw, "%f,%f\n", &x_tmp[i], &y_tmp[i]) != 2){
                    if(d_sim.user.verbose)if(d_sim.user.verbose)printf("Error reading line %d\n", i + 1);
                    exit(1);
                }
                if(d_sim.user.verbose)printf("\t%d, %lf %lf\n", i, x_tmp[i], y_tmp[i]);
            }
            fclose(fw); // 关闭文件
            if(d_sim.user.verbose){
                printf("BzController.order is %d\n", pBezier->order);
                printf("BzController.points[BzController.order].x = %f\n", pBezier->points[pBezier->order].x);
            }
        #else
            SIM_2_EXP_DEFINE_BEZIER_POINTS_X
            SIM_2_EXP_DEFINE_BEZIER_POINTS_Y
        #endif
        for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
            pBezier->points[i].x = x_tmp[i];
            pBezier->points[i].y = y_tmp[i];
        }
    #endif
    if (pBezier->points == NULL)
    {
        errno = ENOMEM;
        return;
    }
}

/**
 * @brief Calculates the control output using Bezier curve interpolation
 * @param r The PID regulator
 * @param pBezier The Bezier controller
 */

void control_output(st_pid_regulator *r, BezierController *pBezier){
    r->Err = r->Ref - r->Fbk;
    REAL error = r->Err * MECH_RAD_PER_SEC_2_RPM;
    
    // #if PC_SIMULATION printf("error: %lf\n", error); #endif
    
    if (fabsf(error) > pBezier->points[pBezier->order].x)
    {
        // printf("fabsf error: %f , bezier upper: %f \n", fabsf(error), pBezier->points[pBezier->order].x);
        error = copysignf(pBezier->points[pBezier->order].x, error);

    }
    // printf("error: %f\n", error);
    // #if PC_SIMULATION printf("error after Bezier: %lf\n", error); #endif
    REAL out = find_y_for_given_x(fabsf(error), pBezier);
    // printf("out: %f\n", out);
    r->OutPrev = r->Out;
    // r->Out = out*( error / (error + 1e-7) );
    r->Out = copysignf(out, error);
    // #if PC_SIMULATION printf("%.1f, %.1f\t", pBezier.points[pBezier.num_points].x, sign(error)); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\t", r->Out, r->OutLimit, out); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\n", r->Err, r->Ref, r->Fbk); #endif
    r->ErrPrev = r->Err;
    if (r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if (r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;
}

void bezier_controller_run_in_main(){
    #if PC_SIMULATION
        if ((*CTRL).timebase > CL_TS){
            (*CTRL).i->cmd_varOmega = d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_step_command){
            (*CTRL).i->cmd_varOmega = -d_sim.user.bezier_rpm_maximum_effective_speed_error * RPM_2_MECH_RAD_PER_SEC;
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance){
            ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
            //CTRL_2.i->cmd_iDQ[1] = 0.3;
        }
        if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance+0.1){
            ACM.TLoad = 0.0;
        }
    #else
        CTRL = &CTRL_1;
    #endif

    PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
    PID_Speed->Fbk = (*CTRL).i->varOmega;
    
    control_output(PID_Speed, &BzController);
    (*debug).set_iq_command = PID_Speed->Out;
    (*debug).set_id_command = 0;
    // printf("Ref: %f, Fbk: %f, Out: %f\n", PID_Speed->Ref, PID_Speed->Fbk, PID_Speed->Out);
    return;
}


#endif
