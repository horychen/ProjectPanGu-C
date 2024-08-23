/* Written by Charles Harris charles.harris@sdl.usu.edu */

/* Modified to not depend on Python everywhere by Travis Oliphant.
 */

#ifndef BRENTQ_H
#define BRENTQ_H

#include "typedef.h"
#include <math.h>
#include <stdio.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

const REAL xtol = 1e-4;
const REAL rtol = 1e-4;
const int32 iter = 70;

REAL xblk = 0., fpre, fcur, fblk = 0., spre = 0., scur = 0., sbis;
/* the tolerance is 2*delta */
REAL delta;
REAL stry, dpre, dblk;
int i;

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

extern REAL brentq(callback_type f, const REAL xa, const REAL xb, scipy_zeros_info *solver_stats, void *func_data_param);

#endif
