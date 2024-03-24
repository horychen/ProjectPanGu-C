#ifndef APPCURY_H
#define APPCURY_H
#include <All_Definition.h>

#define HIP_TYPE 0
#define SHANK_TYPE 1

#define HIP_MIN 0.04
#define HIP_MAX -0.42 // 为什么这个是负的？而且MAX比MIN小？
#define CAN01_MIN 34000//96000//89000       //51943 25000&40000 12000 34000
#define CAN01_MAX 40000//10300//101000      //61584

#define SHANK_MIN -0.8855
#define SHANK_MAX 0.25
#define CAN03_MIN 12000//25000
#define CAN03_MAX 25000//40000

#define CAN_QMAX 131072


// 假设数据点数量已知
#define HIP_SHANK_N 106
// 用于存储数据的结构体
typedef struct {
    double hip[HIP_SHANK_N];
    double shank[HIP_SHANK_N];
} HIP_SHANK_ANGLE_TABLE;

extern HIP_SHANK_ANGLE_TABLE hip_shank_angle_table;

extern REAL HIP_SHANK_FREQUENCY;
extern REAL TEST_HIP_KP;
extern REAL TEST_SHANK_KP;
extern REAL TEST_HIP_SPD_KP;
extern REAL TEST_HIP_SPD_KI;

extern REAL deg_four_bar_map_motor_encoder_angle;
extern REAL rad_four_bar_map_motor_encoder_angle;
extern int32 cnt_four_bar_map_motor_encoder_angle;


double linearInterpolate(double x, double x1, double x2, double y1, double y2);
double look_up_hip_shank_angle(double t, int type);
double hip_shank_angle_to_can(double angle, int type);
#endif
