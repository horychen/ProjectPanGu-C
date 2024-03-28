#ifndef APPCURY_H
#define APPCURY_H
#include <All_Definition.h>

#define HIP_TYPE 0
#define SHANK_TYPE 1

extern REAL HIP_MIN;
extern REAL HIP_MAX; // 涓轰粈涔堣繖涓槸璐熺殑锛熻�屼笖MAX姣擬IN灏忥紵
extern REAL CAN01_MIN; //43000//96000//89000       //51943 25000&40000 12000 34000
extern REAL CAN01_MAX; //33200//10300//101000      //61584

extern REAL SHANK_MIN;
extern REAL SHANK_MAX;
extern REAL CAN03_MIN; //29180//25000
extern REAL CAN03_MAX; //12500//40000

#define CAN_QMAX 131072


// 鍋囪鏁版嵁鐐规暟閲忓凡鐭�
#define HIP_SHANK_N 106
// 鐢ㄤ簬瀛樺偍鏁版嵁鐨勭粨鏋勪綋
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

extern REAL TEST_HIP_POS_OUTLIMIT;
extern REAL TEST_SHANK_POS_OUTLIMIT;

extern REAL deg_four_bar_map_motor_encoder_angle;
extern REAL rad_four_bar_map_motor_encoder_angle;
extern int32 cnt_four_bar_map_motor_encoder_angle;


REAL linearInterpolate(REAL x, REAL x1, REAL x2, REAL y1, REAL y2);
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

#endif

