/* */
#ifndef INVERTER_COMPENSATION_H
#define INVERTER_COMPENSATION_H

/* Compensation Method */
void wubo_inverter_Compensation(REAL iAB[2]);
REAL wubo_inverter_Compensation_get_dist_voltage(REAL current);
REAL CJH_LUT_index_inverter_compensation_get_dist_voltage(REAL current_value);

void sul_inverter_Compensation();


/* 声明 inverter_Compensation 专用变量 */
#define NUMBER_OF_COMPENSATION_POINTS 7
extern REAL inverter_current_point[NUMBER_OF_COMPENSATION_POINTS];
extern REAL inverter_voltage_point[NUMBER_OF_COMPENSATION_POINTS];


#endif


