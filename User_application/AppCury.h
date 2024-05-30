#ifndef APPCURY_H
#define APPCURY_H
#include <All_Definition.h>

#define HIP_TYPE 0
#define SHANK_TYPE 1

extern REAL HIP_MIN;
extern REAL HIP_MAX; // 娑撹桨绮堟稊鍫ｇ箹娑擃亝妲哥拹鐔烘畱閿涚喕锟藉奔绗朚AX濮ｆ摤IN鐏忓骏绱�
extern REAL CAN01_MIN; //43000//96000//89000       //51943 25000&40000 12000 34000
extern REAL CAN01_MAX; //33200//10300//101000      //61584

extern REAL SHANK_MIN;
extern REAL SHANK_MAX;
extern REAL CAN03_MIN; //29180//25000
extern REAL CAN03_MAX; //12500//40000

extern REAL SHANK_POS_CONTROL_IQ;
extern REAL HIP_POS_CONTROL_POS;

#define CAN_QMAX 131072


// 閸嬪洩顔曢弫鐗堝祦閻愯鏆熼柌蹇撳嚒閻拷
#define HIP_SHANK_N 106
// 閻€劋绨�涙ê鍋嶉弫鐗堝祦閻ㄥ嫮绮ㄩ弸鍕秼
typedef struct {
    REAL hip[HIP_SHANK_N];
    REAL shank[HIP_SHANK_N];
} HIP_SHANK_ANGLE_TABLE;

extern HIP_SHANK_ANGLE_TABLE hip_shank_angle_table;

extern REAL HIP_SHANK_FREQUENCY;
extern REAL TEST_HIP_KP;
extern REAL TEST_SHANK_KP;
extern REAL TEST_HIP_SPD_KP;
extern REAL TEST_HIP_SPD_KI;
extern REAL TEST_SHANK_SPD_KP;
extern REAL TEST_SHANK_SPD_KI;

extern REAL TEST_HIP_POS_OUTLIMIT;
extern REAL TEST_SHANK_POS_OUTLIMIT;

extern REAL deg_four_bar_map_motor_encoder_angle;
extern REAL rad_four_bar_map_motor_encoder_angle;
extern int32 cnt_four_bar_map_motor_encoder_angle;


REAL linearInterpolate(REAL x, REAL x1, REAL x2, REAL y1, REAL y2);
REAL linearInterpolate_Uint_to_real(Uint32 x, Uint32 x1, Uint32 x2, REAL y1, REAL y2);
REAL look_up_hip_shank_angle(REAL t, int type);
REAL hip_shank_angle_to_can(REAL angle, int type);

#define PI 3.14159265
#define BEZIER_ORDER 4
#define BEZIER_TRACE_SIZE 100
#define INIT_THETA1 0.0
#define INIT_THETA2 0.0
extern REAL IECON_HEIGHT;

typedef struct
{
    REAL L1;
    REAL L2;
    REAL theta1, theta2;  // unit [rad]   theta1: hip ; theta2: knee
    REAL dot_theta1, dot_theta2;
    REAL T;               // unit [s]
    REAL height_limit[2]; // unit [m]
    REAL C[BEZIER_ORDER][2];
    int order;
    REAL bezier_trace[BEZIER_TRACE_SIZE + 1][2];
} CURYCONTROLLER;

extern CURYCONTROLLER curycontroller;

void calc_theta_from_height(REAL height);
void reset_position();
void linear_controller(REAL t);
void sinusoidal_controller(REAL t);
void get_bezier_points();
REAL bezier_linear_interpoolation(REAL t);
void bezier_controller(REAL t);
void analoge_controller(REAL height);
void run_iecon_main(Uint64 t);

typedef struct
{
    REAL current[6];
    REAL weight[6]
} CURRENT_WEIGHT_TABLE;

REAL get_current_from_weight(REAL weight);

// impedance control
extern REAL HIP_IMPEDANCE_DESIRE;
extern REAL SHANK_IMPEDANCE_DESIRE;
extern REAL HIP_PREV_ANGLE;
extern REAL SHANK_PREV_ANGLE;
extern REAL HIP_IMPENDENCE_B;
extern REAL HIP_IMPENDENCE_D;
extern REAL SHANK_IMPENDENCE_B;
extern REAL SHANK_IMPENDENCE_D;

extern REAL IMPENDENCE_HIP_D_ANGULAR;
extern REAL IMPENDENCE_SHANK_D_ANGULAR;
extern REAL IQOUT_SHANK;
extern REAL IQOUT_HIP;


REAL position_count_to_angle(int axisType, Uint32 hip_count_fbk, Uint32 shank_count_fbk);
REAL calc_theta_angular_velocity(int axisType, REAL hip_theta_fbk, REAL shank_theta_fbk);
REAL Impendence_Control_cal(int axisType, Uint32 hip_count_fbk, Uint32 shank_count_fbk);

#endif

