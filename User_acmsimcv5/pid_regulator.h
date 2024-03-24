#ifndef st_pid_regulatorULATOR_H
#define st_pid_regulatorULATOR_H

typedef struct {
   float32 Ref;
   float32 Fbk;
   float32 Err;
   float32 ErrPrev;
   float32 P_Term; 
   float32 I_Term; 
   float32 D_Term; 
   float32 OutNonSat;
   float32 OutLimit;
   float32 Out;
   float32 OutPrev; // for incremental pid
   float32 Kp;
   float32 Ki;
   float32 Kd;
   float32 SatDiff;
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
  /*Non-Saturated Output*/  0.0, \
  /*Output Limit*/  0.95, \
  /*Output*/  0.0, \
  /*Output from Previous Step*/  0.0, \
  /*Kp*/  1.0, \
  /*Ki*/  0.001, \
  /*Kd*/  0.0, \
  /*Difference between Non-Saturated Output and Saturated Output*/  0.0, \
  (void (*)(Uint32)) PID_calc \
}

void ACMSIMC_PIDTuner();

void commands(REAL *p_set_rpm_speed_command, REAL *p_set_iq_cmd, REAL *p_set_id_cmd);



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

void  PIDController_Init(st_PIDController *pid);
float PIDController_Update(st_PIDController *pid);


extern st_PIDController _pid_iX_1;
extern st_PIDController _pid_iY_1;
extern st_pid_regulator _pid_iD_1;
extern st_pid_regulator _pid_iQ_1;
extern st_pid_regulator _pid_spd_1;
extern st_pid_regulator _pid_pos_1;

extern st_PIDController _pid_iX_2;
extern st_PIDController _pid_iY_2;
extern st_pid_regulator _pid_iD_2;
extern st_pid_regulator _pid_iQ_2;
extern st_pid_regulator _pid_spd_2;
extern st_pid_regulator _pid_pos_2;


//extern st_pid_regulator _pid_ia_1;
//extern st_pid_regulator _pid_ib_1;
//extern st_pid_regulator _pid_ic_1;

#define PID_iD  (CTRL->s->iD)
#define PID_iQ  (CTRL->s->iQ)
#define PID_iX  (CTRL->s->iX)
#define PID_iY  (CTRL->s->iY)
#define PID_spd  (CTRL->s->spd)
#define PID_pos  (CTRL->s->pos)


#endif
