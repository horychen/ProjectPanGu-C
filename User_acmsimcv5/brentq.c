/* Written by Charles Harris charles.harris@sdl.usu.edu */

#include "brentq.h"
#include "Bezier.h"
/*
  At the top of the loop the situation is the following:

    1. the root is bracketed between xa and xb
    2. xa is the most recent estimate
    3. xp is the previous estimate
    4. |fp| < |fb|

  The order of xa and xp doesn't matter, but assume xp < xb. Then xa lies to
  the right of xp and the assumption is that xa is increasing towards the root.
  In this situation we will attempt quadratic extrapolation as long as the
  condition

  *  |fa| < |fp| < |fb|

  is satisfied. That is, the function value is decreasing as we go along.
  Note the 4 above implies that the right inequlity already holds.

  The first check is that xa is still to the left of the root. If not, xb is
  replaced by xp and the interval reverses, with xb < xa. In this situation
  we will try linear interpolation. That this has happened is signaled by the
  equality xb == xp;

  The second check is that |fa| < |fb|. If this is not the case, we swap
  xa and xb and resort to bisection.

*/
const REAL xtol = 1e-4;
const REAL rtol = 1e-4;
const int32 iter = 70;

REAL xblk = 0., fpre, fcur, fblk = 0., spre = 0., scur = 0., sbis;
/* the tolerance is 2*delta */
REAL delta;
REAL stry, dpre, dblk;
int i;

REAL brentq(callback_type f, const REAL xa, const REAL xb, scipy_zeros_info *solver_stats, void *func_data_param)
{
    REAL xpre = xa, xcur = xb;

    fpre = bezier_x_diff(xpre, func_data_param);
    fcur = bezier_x_diff(xcur, func_data_param);

    solver_stats->error_num = INPROGRESS;

    solver_stats->funcalls = 2;
#if PC_SIMULATION == TRUE
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
#endif
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

        fcur = bezier_x_diff(xcur, func_data_param);
        solver_stats->funcalls++;
    }
    solver_stats->error_num = CONVERR;
    return xcur;
}
