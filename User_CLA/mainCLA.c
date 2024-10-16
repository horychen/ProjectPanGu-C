#include <All_Definition.h>

// function prototypes
//interrupt void control_Isr(void);

// global  variables
long IdleLoopCount = 0;
long IsrCount = 0;
float Duty;

// shared variables
#pragma DATA_SECTION(rk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(yk, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(uk, "Cla1ToCpuMsgRAM")
float rk = 0.25f;
float yk;
float uk;

#pragma DATA_SECTION(pi1, "Cla1DataRam1")
DCL_PI_CLA pi1 = PI_CLA_DEFAULTS;

