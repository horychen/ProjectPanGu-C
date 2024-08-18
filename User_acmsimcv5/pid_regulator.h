#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H


typedef struct {
   float32 Ref;
   float32 Fbk;
   float32 Err;
   float32 ErrPrev;
   float32 P_Term; 
   float32 I_Term; 
   float32 D_Term;
   float32 KFB_Term; 
   float32 OutNonSat;
   float32 OutLimit;
   float32 Out;
   float32 OutPrev; // for incremental pid
   float32 Kp;
   float32 Ki_CODE;
   float32 Kd;
   float32 KFB;
   float32 SatDiff;
   float32 FbkPrev;
   float32 KFB_Term_Prev;
   float32 OutWithInnerLoopFeedback;
   void (*calc)();
} st_pid_regulator;
typedef st_pid_regulator *st_pid_regulator_handle;
void PID_calc(st_pid_regulator_handle);
#define st_pid_regulator_DEFAULTS { \
  /*Reference*/ 0.0, \
  /*Feedback*/ 0.0, \
  /*Control Error*/ 0.0, \
  /*Control Error from Previous Step*/ 0.0, \
  /*Proportional Term*/  0.0, \
  /*Integral Term*/  0.0, \
  /*Derivative Term*/  0.0, \
  /*Inner Feedback Term*/  0.0, \
  /*Non-Saturated Output*/  0.0, \
  /*Output Limit*/  0.95, \
  /*Output*/  0.0, \
  /*Output from Previous Step*/  0.0, \
  /*Kp*/  1.0, \
  /*Ki*/  0.001, \
  /*Kd*/  0.0, \
  /*KFB*/ 0.0, \
  /*Difference between Non-Saturated Output and Saturated Output*/  0.0, \
  /*Previous Feedback*/  0.0, \
  /*Previous Inner Feedback Term*/  0.0, \
  /* OutWithInnerLoopFeedback */ 0.0, \
  (void (*)(Uint32)) PID_calc \
}
extern st_pid_regulator PID_iD;
extern st_pid_regulator PID_iQ;
extern st_pid_regulator PID_Speed;
extern st_pid_regulator PID_Position;
// extern st_pid_regulator pid1_ia;
// extern st_pid_regulator pid1_ib;
// extern st_pid_regulator pid1_ic;
// extern st_pid_regulator pid2_ix;
// extern st_pid_regulator pid2_iy;

void ACMSIMC_PIDTuner();

typedef struct{
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float outLimit;

    /* Integrator limits */
    float intLimit;

    /* Sample time (in seconds) */
    float Ts;

    /* Controller "memory" */
    float integrator;
    float prevError;      /* Required for integrator */
    float differentiator;
    float prevMeasurement;    /* Required for differentiator */

    /* Controller output */
    float out;

    /* Controller input */
    float setpoint;
    float measurement;
} st_PIDController;
extern st_PIDController pid1_dispX;
extern st_PIDController pid1_dispY;
void  PIDController_Init(st_PIDController *pid);
float PIDController_Update(st_PIDController *pid);


#endif
