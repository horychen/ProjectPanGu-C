#include "ACMSim.h"
#include "simuser_bezier.h"
#include "super_config.h"
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

BezierController BezierVL, BezierVL_AdaptVersion; // Velocity
BezierController BezierCL, BezierCL_AdaptVersion; // Current

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
REAL find_t_for_given_x(const REAL x, const BezierController *BzierController){
    FindTParams params;
    params.BzierController = BzierController;
    params.target_x = x;
    scipy_zeros_info solver_stats;
    REAL root = brentq(bezier_x_diff, 0.0f, 1.0f, &solver_stats, &params);
    if (solver_stats.error_num != CONVERGED){
        #if PC_SIMULATION
            printf("[Bezier] Error: %d, %g\n", solver_stats.error_num, root);
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
REAL find_y_for_given_x(const REAL x, const BezierController *BzierController){
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

    // 自适应增益
    pBezier->adapt_gain = 0*2*250*1e-4; // 注意bezier控制器在实验中的执行频率是多少！less than 1kHz?
    pBezier->nonlinear_fake_disturbance_estimate = 0.0;
    pBezier->error = 0.0;
    pBezier->error_previous = 0.0;
    pBezier->flag_integral_saturated = FALSE;

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
                    if(d_sim.user.verbose)printf("Error reading line %d\n", i + 1);
                    exit(1);
                }
                if(d_sim.user.verbose)printf("\t%d, %lf %lf\n", i, x_tmp[i], y_tmp[i]);
            }
            fclose(fw); // 关闭文件
            if(d_sim.user.verbose){
                printf("BezierVL.order is %d\n", pBezier->order);
                printf("BezierVL.points[BezierVL.order].x = %f\n", pBezier->points[pBezier->order].x);
            }
        #else
            SIM_2_EXP_DEFINE_BEZIER_POINTS_X
            SIM_2_EXP_DEFINE_BEZIER_POINTS_Y
            // TODO
        #endif
        for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
            pBezier->points[i].x = x_tmp[i];
            pBezier->points[i].y = y_tmp[i];
        }
    #endif
    if (pBezier->points == NULL){
        errno = ENOMEM;
        return;
    }
}
#if 0
void set_points_cl(BezierController *pBezier){
    #define ARGS_PATH_BEZIER_CL "../acmsimc_bezier_points/bezier_cl.txt"

    // 自适应增益
    pBezier->adapt_gain = 0.5*250*1e-4; // 注意bezier控制器在实验中的执行频率是多少！less than 1kHz?
    pBezier->nonlinear_fake_disturbance_estimate = 0.0;
    pBezier->error = 0.0;
    pBezier->error_previous = 0.0;
    pBezier->flag_integral_saturated = FALSE;

    int i;

    pBezier->order = d_sim.user.bezier_order;
    #if PC_SIMULATION
        if(BEZIER_NUMBER_OF_POINTS>10){
            if(d_sim.user.verbose)printf("TOO MANY BEZIER POINTS!!!");
        }
        REAL x_tmp[10];
        REAL y_tmp[10];
        FILE *fw;
        fw = fopen(ARGS_PATH_BEZIER_CL, "r");
        if (fw == NULL){
            if(d_sim.user.verbose)printf("Error opening file!\n");
            exit(1);
        }
        if(d_sim.user.verbose)printf("Bezier control points:\n");
        for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
            if (fscanf(fw, "%f,%f\n", &x_tmp[i], &y_tmp[i]) != 2){
                if(d_sim.user.verbose)printf("Error reading line %d\n", i + 1);
                exit(1);
            }
            if(d_sim.user.verbose)printf("\t%d, %lf %lf\n", i, x_tmp[i], y_tmp[i]);
        }
        fclose(fw); // 关闭文件
        if(d_sim.user.verbose){
            printf("BezierVL.order is %d\n", pBezier->order);
            printf("BezierVL.points[BezierVL.order].x = %f\n", pBezier->points[pBezier->order].x);
        }
    #else
        // TODO
    #endif
    for (i = 0; i < BEZIER_NUMBER_OF_POINTS; ++i){
        pBezier->points[i].x = x_tmp[i];
        pBezier->points[i].y = y_tmp[i];
    }

    if (pBezier->points == NULL){
        errno = ENOMEM;
        return;
    }
}
#endif

/**
 * @brief Calculates the control output using Bezier curve interpolation
 * @param r The PID regulator
 * @param pBezier The Bezier controller
 */
void control_output(st_pid_regulator *r, BezierController *pBezier){
    // 重新定义一个误差，单位是 r/min
    r->ErrPrev = r->Err;
    r->Err = r->Ref - r->Fbk;
    pBezier->error = r->Err * MECH_RAD_PER_SEC_2_RPM;

    // error 的绝对值最大只能是 pBezier->points[pBezier->order].x
    if (fabsf(pBezier->error) > pBezier->points[pBezier->order].x){
        // printf("fabsf error: %f , bezier upper: %f \n", fabsf(error), pBezier->points[pBezier->order].x);
        pBezier->error = copysignf(pBezier->points[pBezier->order].x, pBezier->error);
    }

    // bezier curve mapping
    REAL out = find_y_for_given_x(fabsf(pBezier->error), pBezier);

    r->OutPrev = r->Out;
    r->Out = copysignf(out, pBezier->error);
    // printf("error: %f\n", error);
    // #if PC_SIMULATION printf("error after Bezier: %lf\n", error); #endif
    // printf("out: %f\n", out);
    // r->Out = out*( error / (error + 1e-7) );
    // #if PC_SIMULATION printf("%.1f, %.1f\t", pBezier.points[pBezier.num_points].x, sign(error)); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\t", r->Out, r->OutLimit, out); #endif
    // #if PC_SIMULATION printf("%.1f, %.1f, %.1f\n", r->Err, r->Ref, r->Fbk); #endif

    // if (r->Out > r->OutLimit)
    //     r->Out = r->OutLimit;
    // else if (r->Out < -r->OutLimit)
    //     r->Out = -r->OutLimit;

    // printf("%g: %g, %g\n", CTRL->timebase, r->Out, r->OutLimit);
}
REAL control_output_adaptVersion(st_pid_regulator *r, BezierController *pBAV){
    // 消灭稳态误差
    /* 这个补充函数必须在 control_output 之后调用，要求 r->Out 是被赋值更新过的 */
    /* control_output 的输出 r->Out 将成为本函数的非线性自适应增益 */
    /* 可调增益：pBAV->adapt_gain， where pBAV stands for pointer-Bezier-Adaptive-Version */

    // bezier curve mapping
    pBAV->error_previous = pBAV->error;
    pBAV->error = r->Err * MECH_RAD_PER_SEC_2_RPM;
    /* 积分重置，如果当前步误差异号 并且 积分器饱和 */
    if( pBAV->error_previous * pBAV->error < 0 && pBAV->flag_integral_saturated ){
        // pBAV->nonlinear_fake_disturbance_estimate = 0.0;
    }


    // 限制范围
    #define INDEX_HIGH_TORQUE_POINT 1
    if( fabsf(pBAV->nonlinear_fake_disturbance_estimate) >= pBAV->points[INDEX_HIGH_TORQUE_POINT].y){
        pBAV->flag_integral_saturated = TRUE;
        pBAV->points[0].y = pBAV->points[INDEX_HIGH_TORQUE_POINT].y;
        // pBAV->points[1].y = pBAV->points[INDEX_HIGH_TORQUE_POINT].y;
        // pBAV->points[2].y = pBAV->points[INDEX_HIGH_TORQUE_POINT].y;
    }else{
        pBAV->flag_integral_saturated = FALSE;
        pBAV->points[0].y = fabsf(pBAV->nonlinear_fake_disturbance_estimate);
        // pBAV->points[1].y = fabsf(pBAV->nonlinear_fake_disturbance_estimate);
        // pBAV->points[2].y = fabsf(pBAV->nonlinear_fake_disturbance_estimate);
    }


    // 【去掉符號】 error 的绝对值最大只能是 pBezier->points[pBezier->order].x
    if (fabsf(pBAV->error) > pBAV->points[pBAV->order].x){
        pBAV->error = copysignf(pBAV->points[pBAV->order].x, pBAV->error);
    }
    pBAV->output = find_y_for_given_x( fabsf(pBAV->error), pBAV );
    pBAV->output = copysignf(pBAV->output, pBAV->error);

    // 如果异号，需要手动补偿offset
    if(pBAV->output * pBAV->nonlinear_fake_disturbance_estimate < 0.0){
        /* 不要动这个数字，就是2.0，不要动！ */
        pBAV->output += 2.0 * pBAV->nonlinear_fake_disturbance_estimate;
    }

    // 自适应律的增益相对于速度误差也是非线性的，直接复用 bezier curve mapping
    /* Anti-windup (this is useful for aggressive Bezier curve) */
    if(fabsf(r->Out) < 0.5 * pBAV->points[pBAV->order].y)
    {
        // pBAV->nonlinear_fake_disturbance_estimate += pBAV->adapt_gain * r->Err; // 线性自适应律
        pBAV->nonlinear_fake_disturbance_estimate += pBAV->adapt_gain * r->Out; // Bezier自适应律
        // pBAV->nonlinear_fake_disturbance_estimate += 0.01 * (pBAV->output - r->Out);
        // pBAV->nonlinear_fake_disturbance_estimate += -pBAV->adapt_gain * r->Err;
    }else{
        /* 这个扰动估计的归零操作会引入额外的非线性 */
        // pBAV->nonlinear_fake_disturbance_estimate = 0.0;

        /* 或者，允许反向迅速减小扰动估计如何？ */
        if(pBAV->nonlinear_fake_disturbance_estimate * r->Out < 0.0){
            pBAV->nonlinear_fake_disturbance_estimate += 10 * pBAV->adapt_gain * r->Out;
        }
    }

    /* 仿真打印 */
    #if PC_SIMULATION
        static long counter = 0;
        if(++counter>=100){
            counter = 0;
            printf(
                "%g, %g, | %g, %g, | %g, %g, %g\n", 
                CTRL->timebase, 
                find_y_for_given_x( 0.0, pBAV ), 
                pBAV->error, 
                pBAV->output,
                (*debug).set_iq_command,
                CTRL->i->cmd_iDQ[1],
                pBAV->nonlinear_fake_disturbance_estimate
            );
        }
    #endif

    return pBAV->output;
}

void bezier_controller_run_in_main(){

    //TODO: These cmd should be placed at some function!!!
    // #if PC_SIMULATION
    //     // ACM.TLoad = 1.0 + 100 * VISCOUS_COEFF * (*CTRL).i->varOmega;
    //     if ((*CTRL).timebase > CL_TS){
    //         // (*CTRL).i->cmd_varOmega =  500 * RPM_2_MECH_RAD_PER_SEC;
    //         (*CTRL).i->cmd_varOmega =  400 * RPM_2_MECH_RAD_PER_SEC;

    //     }
    //     if ((*CTRL).timebase > 0.02){
    //         // (*CTRL).i->cmd_varOmega = 2100 * RPM_2_MECH_RAD_PER_SEC;
    //         (*CTRL).i->cmd_varOmega =  0 * RPM_2_MECH_RAD_PER_SEC;
    //     }

    //     // if ((*CTRL).timebase > d_sim.user.bezier_seconds_step_command){
    //     //     (*CTRL).i->cmd_varOmega = - 500 * RPM_2_MECH_RAD_PER_SEC;
    //     // }
    //     // if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance){
    //     if ((*CTRL).timebase > 0.03){
    //         // ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
    //         // ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 * 0.95) * sin(50*2*M_PI*CTRL->timebase);
    //         //CTRL_2.i->cmd_iDQ[1] = 0.3;
    //     }
    //     // if ((*CTRL).timebase > d_sim.user.bezier_seconds_load_disturbance+0.1){
    //     if ((*CTRL).timebase > 0.05){
    //         ACM.TLoad = 0.0;
    //     }
    // #else
    //     CTRL = &CTRL_1;
    // #endif

    if (!d_sim.user.bezier_Give_Sweeping_Ref_in_Interrupt){
        overwrite_sweeping_frequency();
        /* Mark -3db points */
        _user_Check_ThreeDB_Point( (*CTRL).i->varOmega*MECH_RAD_PER_SEC_2_RPM, d_sim.user.CMD_SPEED_SINE_RPM );
    }

    PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
    PID_Speed->Fbk = (*CTRL).i->varOmega;
    PID_Speed->OutLimit = BezierVL.points[BezierVL.order].y;
    control_output(PID_Speed, &BezierVL);
    if (d_sim.user.BOOL_BEZIER_ADAPTIVE_GAIN == FALSE){
        (*debug).set_iq_command = PID_Speed->Out;
    }else{
        (*debug).set_iq_command = control_output_adaptVersion(PID_Speed, &BezierVL_AdaptVersion);
    }
    (*debug).set_id_command = 0;
    
    // printf("Ref: %f, Fbk: %f, Out: %f\n", PID_Speed->Ref, PID_Speed->Fbk, PID_Speed->Out);
    return;
}


#endif
