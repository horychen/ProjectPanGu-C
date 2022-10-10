#include <All_Definition.h>

// typedef struct {
// } st_hall; // hall sensor as quadrature encoder

// ����������һ�������ص����⣬�Ǿ��Ǿ�ֹ��ʱ���ʧȥ��׼��
#define UPWARD_TREND 1
#define DOWNWARD_TREND -1
#define PEAKING 1
#define VALLEY -1
#define RISING_CROSSING 1
#define FALLING_CROSSING -1

#define _T0 0
#define _T1 1
#define _T2 2
#define _T3 3
#define _T4 4
#define _T5 5
#define _T6 6
#define _T7 7

#define CW_DIRECTION 1.0
#define COUNTER_CW_DIRECTION -1.0

#define ST_MAGNETS 32
#define ONE_OVER_ST_MAGNETS 0.03125 // 1.0/32 : one over sensing target magnets

#define HALL_X 10
#define HALL_Y 11

#define NORTH_POLE  1
#define SOUTH_POLE -1
// REAL ADC_HALL_OFFSET_ADC5 = 1659.5;// (1350 - -732) / 2
// REAL ADC_HALL_OFFSET_ADC4 = 1671.0;// (1355 - -847) / 2
// REAL ADC_HALL_OFFSET_ADC10 = 1850; // (700+3000)/2
REAL hall_sensor_read_last[3];
REAL hall_sensor_read_delta[3];
REAL hall_sensor_read_delta_last[3];
REAL hall_sensor_read_max_global[3] = {100, 100, 100};
REAL hall_sensor_read_min_global[3] = {-100, -100, -100};
REAL hall_sensor_read_max_local[3] = {100, 100, 100};
REAL hall_sensor_read_min_local[3] = {-100, -100, -100};
int current_pole[3] = {NORTH_POLE, NORTH_POLE, NORTH_POLE};
int last_pole[3]    = {NORTH_POLE, NORTH_POLE, NORTH_POLE};
int current_trend[3] = {0,0,0};
int last_trend[3] = {0,0,0};
int event_extreme[3] = {0,0,0};
int event_zero_crossing[3] = {0,0,0};
REAL hall_rotating_direction = 1.0;
int count_magnet[3] = {0, 0, 0};
int count_extreme[3] = {0, 0, 0};
// #define CL_TS ctrl.Ts
// #define CTRL ctrl
REAL delta_time[3] = {0,0,0};
REAL delta_voltage[3] = {0,0,0};

REAL hall_timing[3][8]; // ��ʼ����
REAL hall_readin[3][8];

REAL hall_timing_eventpoint_most_recent[3];
REAL hall_readin_extremepoint_most_recent[3];
REAL hall_timing_midpoint_most_recent[3];
REAL hall_readin_midpoint_most_recent[3];

REAL hall_omega_r_elec[3];
REAL hall_theta_r_elec[3];
REAL hall_theta_r_elec_0 = 0.0;
REAL hall_theta_r_elec_0_last = 0.0;
REAL hall_theta_r_elec_incremental[3];
REAL hall_theta_r_elec_local_absolute[3];

long int pole_change_count = 0;
long int trend_change_count = 0;

int event_lock[3] = {FALSE,FALSE,FALSE};
REAL normalizer[3] = {1e-3, 1e-3, 1e-3};

// ���¼�����������ٶȼ��ɣ���Ϊ�¼�������ĽǶ���׼ȷ�ġ������������ټ���ȥ���½Ƕȼ��ɡ�

#define CODE_CALCULATE_SPEED(X) hall_omega_r_elec[0] = 2*M_PI*ONE_OVER_ST_MAGNETS / (CTRL.timebase - hall_timing_eventpoint_most_recent[X])

REAL hall_sensor_read_max_NORTH[3] = {100,100,100};
REAL hall_sensor_read_min_SOUTH[3] = {100,100,100};
REAL hall_sensor_read_max_NORTH_last[3] = {100,100,100};
REAL hall_sensor_read_min_SOUTH_last[3] = {100,100,100};

// REAL magnet_max[2][32];
// REAL magnet_max[2][32];
// int magnet_index = 0;

int error_hall_conversion = FALSE;

REAL event_rising_threhold[3] = {FALSE, FALSE, FALSE};
REAL event_falling_threhold[3] = {FALSE, FALSE, FALSE};
int event_rising_crossing[3] = {FALSE, FALSE, FALSE};
int event_falling_crossing[3] = {FALSE, FALSE, FALSE};

REAL hall_qep_count_raw[3] = {0, 0, 0};
REAL hall_qep_count = 0.0;
REAL hall_qep_angle = 0.0;

int recent_zero_crossing_hall_is=HALL_X;

void start_hall_conversion(REAL hall_sensor_read[]){

    // // ��ADCת���õ���ģ���ѹ�������м��ԵĻ�������������
    // hall_sensor_read[0] = (REAL)AdcaResultRegs.ADCRESULT5 - ADC_HALL_OFFSET_ADC5; // 32 poles on Aluminum target
    // hall_sensor_read[1] = (REAL)AdcaResultRegs.ADCRESULT4 - ADC_HALL_OFFSET_ADC4; // 32 poles on Aluminum target
    // hall_sensor_read[2] = (REAL)AdcbResultRegs.ADCRESULT10 - ADC_HALL_OFFSET_ADC10; // Bogen 40 poles on the top

    // �ֲ������������֮��ģ���ֵ
    if(hall_sensor_read[0] > hall_sensor_read_max_NORTH[0]) hall_sensor_read_max_NORTH[0] = hall_sensor_read[0];
    if(hall_sensor_read[0] < hall_sensor_read_min_SOUTH[0]) hall_sensor_read_min_SOUTH[0] = hall_sensor_read[0];
    if(hall_sensor_read[1] > hall_sensor_read_max_NORTH[1]) hall_sensor_read_max_NORTH[1] = hall_sensor_read[1];
    if(hall_sensor_read[1] < hall_sensor_read_min_SOUTH[1]) hall_sensor_read_min_SOUTH[1] = hall_sensor_read[1];

    // �����ı����Ǳ������еķ���
    // lock_zero_crossing[0]
    // lock_threshold[0]

    // ���⣺��ת��һ���ж�����

    // �жϼ�ֵ�߼�������ֵ�������䵽��ֵ�������е����ֵ���Ǽ�ֵ��No���޷�������
    // if(hall_sensor_read[0]>=THRESHOLD_RATIO*hall_sensor_read_max_NORTH_last[0]){
    //     lock_zero_crossing[0]
    // }

    // �жϻ��������
    #define THRESHOLD_RATIO 0.2
    /* HALL X */
    if(hall_sensor_read[0]>0 && hall_sensor_read_last[0]>0){ // ���κ��ϴξ�������
        pole_change_count += 1;
        current_pole[0] = NORTH_POLE;
    }else if(hall_sensor_read[0]<0 && hall_sensor_read_last[0]<0){ // ���κ��ϴξ�С����
        pole_change_count -= 1;
        current_pole[0] = SOUTH_POLE;
    }else{
        pole_change_count = 0;
    }
    // �жϻ�������ֵ��
    if(hall_sensor_read[0]>=THRESHOLD_RATIO*hall_sensor_read_max_NORTH_last[0]){
        if(event_zero_crossing[0]==RISING_CROSSING){
            // �����¼��������һ��Խ����ֵ�㣬������Ϊ�ϴι��������Ĺ����ˣ���һ����ǣ����ڿ�ʼ����¼���伫ֵ��
            event_rising_threhold[0] = TRUE;
        }else{
            // ����������ֵ�߸���������Ҳ�����ǴӼ�ֵ����䵽��ֵ
        }
        event_zero_crossing[0] = FALSE;
    }else if(hall_sensor_read[0]<=THRESHOLD_RATIO*hall_sensor_read_min_SOUTH_last[0]){
        if(event_zero_crossing[0]==FALLING_CROSSING){
            // �����¼��������һ��Խ����ֵ�㣬������Ϊ�ϴι��������Ĺ����ˣ���һ����ǣ����ڿ�ʼ����¼���伫ֵ��
            event_falling_threhold[0] = TRUE;
        }else{
            // ����������ֵ�߸���������Ҳ�����ǴӼ�ֵ����䵽��ֵ
        }
        event_zero_crossing[0] = FALSE;
    }

    // ���Ըı��ˣ�
    if(current_pole[0]*last_pole[0]==-1){
        // ���Ըı��Ҳ�Ϊ��
        // last_pole[0] = current_pole[0]; // last_pole�ĸ�����ζ����ʽ�����µı仯�����ˡ�

        // lock_zero_crossing[0] = TRUE;

        if(current_pole[0]==NORTH_POLE){

            if(event_zero_crossing[0]!=RISING_CROSSING){

                // ���Լ���ת��
                // CODE_CALCULATE_SPEED(0);

                // �����߹������������
                count_magnet[0] += 1;
                hall_theta_r_elec_incremental[0] = count_magnet[0] * M_PI*ONE_OVER_ST_MAGNETS;
                if(count_magnet[0]>=ST_MAGNETS){
                    count_magnet[0] = 0;

                    // ���»���������Ӧ��ѹ�����л���ƫ��ֵ
                    //            ADC_HALL_OFFSET_ADC5 = 0.5 * (hall_sensor_read_max_global[0] - hall_sensor_read_min_global[0]);
                    //            ADC_HALL_OFFSET_ADC4 = 0.5 * (hall_sensor_read_max_global[1] - hall_sensor_read_min_global[1]);
                    //            ADC_HALL_OFFSET_ADC10 = 0.5 * (hall_sensor_read_max_global[2] - hall_sensor_read_min_global[2]);
                    //            hall_sensor_read_max_global[0] =  100; // reset max and min
                    //            hall_sensor_read_max_global[1] =  100; // reset max and min
                    //            hall_sensor_read_max_global[2] =  100; // reset max and min
                    //            hall_sensor_read_min_global[0] = -100; // reset max and min
                    //            hall_sensor_read_min_global[1] = -100; // reset max and min
                    //            hall_sensor_read_min_global[2] = -100; // reset max and min
                }

                // ��¼���һ����������ֵ��������normalizer
                hall_sensor_read_max_NORTH_last[0] = hall_sensor_read_max_NORTH[0];
                normalizer[0] = 1.0 / hall_sensor_read_max_NORTH_last[0];
                hall_sensor_read_max_NORTH[0] =  100;

                // ��¼һЩ��Ϣ����һ���������ģ�
                hall_timing[0][_T0] = CTRL.timebase;       hall_timing_eventpoint_most_recent[0] = CTRL.timebase;
                hall_readin[0][_T0] = hall_sensor_read[0]; // hall_readin_eventpoint_most_recent[0] = hall_sensor_read[0];

                // ���������¼�
                event_zero_crossing[0] = RISING_CROSSING;
            }

        }else if(current_pole[0]==SOUTH_POLE){

            if(event_zero_crossing[0]!=FALLING_CROSSING){

                // �����߹������������
                count_magnet[0] += 1;
                if(count_magnet[0]>=ST_MAGNETS){
                    count_magnet[0] = 0;
                }
            }


            // ��¼���һ���������Сֵ��������normalizer
            hall_sensor_read_min_SOUTH_last[0] = hall_sensor_read_min_SOUTH[0];
            normalizer[0] = 1.0 / -hall_sensor_read_min_SOUTH_last[0];
            hall_sensor_read_min_SOUTH[0] = -100;

            // if(event_rising_threhold[0]==TRUE){
            //     // ���˿���ȷ�������ļ�ֵ�Ѿ��ﵽ��rising crossing threshold -> falling crossing zero��
            //     event_rising_threhold[0] = FALSE;
            //     hall_sensor_read_max_NORTH_last[0] = hall_sensor_read_max_NORTH[0];
            //     hall_sensor_read_max_NORTH[0] = 100;
            //     normalizer[0] = 1.0 / hall_sensor_read_max_NORTH_last[0];
            // }

            // hall_sensor_read_min_SOUTH_last[0] = hall_sensor_read_min_SOUTH[0];

            // hall_timing[0][_T4] = CTRL.timebase;       hall_timing_eventpoint_most_recent[0] = CTRL.timebase;
            // hall_readin[0][_T4] = hall_sensor_read[0]; // hall_readin_eventpoint_most_recent[0] = hall_sensor_read[0];
            // event_zero_crossing[0] = FALLING_CROSSING;
            // normalizer[0] = 1.0 / fabs(hall_sensor_read_min_SOUTH_last[0]);
        }

    }




    // �жϻ��������
    /* HALL Y */
    if(hall_sensor_read[1]>0 && hall_sensor_read_last[1]>0){ // ���κ��ϴξ�������
        pole_change_count += 1;
        current_pole[1] = NORTH_POLE;
    }else if(hall_sensor_read[1]<0 && hall_sensor_read_last[1]<0){ // ���κ��ϴξ�С����
        pole_change_count -= 1;
        current_pole[1] = SOUTH_POLE;
    }else{
        pole_change_count = 0;
    }
    // �жϻ�������ֵ��
    if(hall_sensor_read[1]>=THRESHOLD_RATIO*hall_sensor_read_max_NORTH_last[1]){
        if(event_zero_crossing[1]==RISING_CROSSING){
            // �����¼��������һ��Խ����ֵ�㣬������Ϊ�ϴι��������Ĺ����ˣ���һ����ǣ����ڿ�ʼ����¼���伫ֵ��
            event_rising_threhold[1] = TRUE;
        }else{
            // ����������ֵ�߸���������Ҳ�����ǴӼ�ֵ����䵽��ֵ
        }
        event_zero_crossing[1] = FALSE;
    }else if(hall_sensor_read[1]<=THRESHOLD_RATIO*hall_sensor_read_min_SOUTH_last[1]){
        if(event_zero_crossing[1]==FALLING_CROSSING){
            // �����¼��������һ��Խ����ֵ�㣬������Ϊ�ϴι��������Ĺ����ˣ���һ����ǣ����ڿ�ʼ����¼���伫ֵ��
            event_falling_threhold[1] = TRUE;
        }else{
            // ����������ֵ�߸���������Ҳ�����ǴӼ�ֵ����䵽��ֵ
        }
        event_zero_crossing[1] = FALSE;
    }

    // ���Ըı��ˣ�
    if(current_pole[1]*last_pole[1]==-1){
        // ���Ըı��Ҳ�Ϊ��
        // last_pole[1] = current_pole[1]; // last_pole�ĸ�����ζ����ʽ�����µı仯�����ˡ�

        // lock_zero_crossing[1] = TRUE;

        if(current_pole[1]==NORTH_POLE){

            if(event_zero_crossing[1]!=RISING_CROSSING){

                // ���Լ���ת��
                // CODE_CALCULATE_SPEED(1);

                // �����߹������������
                count_magnet[1] += 1;
                if(count_magnet[1]>=ST_MAGNETS){
                    count_magnet[1] = 0;

                    // ���»���������Ӧ��ѹ�����л���ƫ��ֵ
                    //            ADC_HALL_OFFSET_ADC5 = 0.5 * (hall_sensor_read_max_global[1] - hall_sensor_read_min_global[1]);
                    //            ADC_HALL_OFFSET_ADC4 = 0.5 * (hall_sensor_read_max_global[1] - hall_sensor_read_min_global[1]);
                    //            ADC_HALL_OFFSET_ADC10 = 0.5 * (hall_sensor_read_max_global[2] - hall_sensor_read_min_global[2]);
                    //            hall_sensor_read_max_global[1] =  100; // reset max and min
                    //            hall_sensor_read_max_global[1] =  100; // reset max and min
                    //            hall_sensor_read_max_global[2] =  100; // reset max and min
                    //            hall_sensor_read_min_global[1] = -100; // reset max and min
                    //            hall_sensor_read_min_global[1] = -100; // reset max and min
                    //            hall_sensor_read_min_global[2] = -100; // reset max and min
                }

                // hall_qep_count += 1;
                hall_theta_r_elec_incremental[1] = count_magnet[1] * M_PI*ONE_OVER_ST_MAGNETS;

                // if(current_pole[0]-last_pole[0]>0){
                //     event_rising_crossing[0] = TRUE;
                //     hall_qep_count += 1;
                // }else if(current_pole[0]-last_pole[0]<0){
                //     event_falling_crossing[0] = TRUE;
                //     hall_qep_count -= 1;
                // }

                // ��¼���һ����������ֵ��������normalizer
                hall_sensor_read_max_NORTH_last[1] = hall_sensor_read_max_NORTH[1];
                normalizer[1] = 1.0 / hall_sensor_read_max_NORTH_last[1];
                hall_sensor_read_max_NORTH[1] =  100;

                // ��¼һЩ��Ϣ����һ���������ģ�
                hall_timing[1][_T0] = CTRL.timebase;       hall_timing_eventpoint_most_recent[1] = CTRL.timebase;
                hall_readin[1][_T0] = hall_sensor_read[1]; // hall_readin_eventpoint_most_recent[1] = hall_sensor_read[1];

                // ���������¼�
                event_zero_crossing[1] = RISING_CROSSING;
            }

        }else if(current_pole[1]==SOUTH_POLE){

            if(event_zero_crossing[1]!=FALLING_CROSSING){

                // �����߹������������
                count_magnet[1] += 1;
                if(count_magnet[1]>=ST_MAGNETS){
                    count_magnet[1] = 0;
                }

                // if(current_pole[0]-last_pole[0]>0){
                //     event_rising_crossing[0] = TRUE;
                //     hall_qep_count += 1;
                // }else if(current_pole[0]-last_pole[0]<0){
                //     event_falling_crossing[0] = TRUE;
                //     hall_qep_count -= 1;
                // }

            }

            // ��¼���һ���������Сֵ��������normalizer
            hall_sensor_read_min_SOUTH_last[1] = hall_sensor_read_min_SOUTH[1];
            normalizer[1] = 1.0 / -hall_sensor_read_min_SOUTH_last[1];
            hall_sensor_read_min_SOUTH[1] = -100;

            // if(event_rising_threhold[1]==TRUE){
            //     // ���˿���ȷ�������ļ�ֵ�Ѿ��ﵽ��rising crossing threshold -> falling crossing zero��
            //     event_rising_threhold[1] = FALSE;
            //     hall_sensor_read_max_NORTH_last[1] = hall_sensor_read_max_NORTH[1];
            //     hall_sensor_read_max_NORTH[1] = 100;
            //     normalizer[1] = 1.0 / hall_sensor_read_max_NORTH_last[1];
            // }

            // hall_sensor_read_min_SOUTH_last[1] = hall_sensor_read_min_SOUTH[1];

            // hall_timing[1][_T4] = CTRL.timebase;       hall_timing_eventpoint_most_recent[1] = CTRL.timebase;
            // hall_readin[1][_T4] = hall_sensor_read[1]; // hall_readin_eventpoint_most_recent[1] = hall_sensor_read[1];
            // event_zero_crossing[1] = FALLING_CROSSING;
            // normalizer[1] = 1.0 / fabs(hall_sensor_read_min_SOUTH_last[1]);
        }

    }



    // �������루ָ���û�����ѹ��������Ϣ���;��ȣ�
    /* HALL X��ʾ����ͼ��Ļ�ɫͨ����Ҳ���ǲ��γ�ǰ��ʮ�ȵ��Ǹ������� */
    if(current_pole[0]*last_pole[0]==-1){
        if(current_pole[0]==NORTH_POLE){

            if(recent_zero_crossing_hall_is==HALL_X){
                // �������߷�ת�ˣ�
                // Do nothing
            }else if(recent_zero_crossing_hall_is==HALL_Y){
                if(event_rising_crossing[1]==TRUE){
                    event_rising_crossing[1] = FALSE; // �����¼���־λ
                    hall_qep_count -= 1; // ����Ϊ������������ת
                    hall_qep_count_raw[0] -= 1;
                    hall_rotating_direction = -1;
                }
                if(event_falling_crossing[1]==TRUE){
                    event_falling_crossing[1] = FALSE;
                    hall_qep_count += 1; // ����Ϊ������������ת���ӻ��������ѹ�����Ͽ��������Ҵ�����
                    hall_qep_count_raw[0] += 1;
                    hall_rotating_direction = +1;
                }
            }
            // �Ա������������¼����м�¼
            event_rising_crossing[0] = TRUE;
            recent_zero_crossing_hall_is = HALL_X;

        }else if(current_pole[0]==SOUTH_POLE){

            if(recent_zero_crossing_hall_is==HALL_X){
                // �������߷�ת�ˣ�
                // Do nothing
            }else if(recent_zero_crossing_hall_is==HALL_Y){
                if(event_rising_crossing[1]==TRUE){
                    event_rising_crossing[1] = FALSE; // �����¼���־λ
                    hall_qep_count += 1; // ����Ϊ������������ת���ӻ��������ѹ�����Ͽ��������Ҵ�����
                    hall_qep_count_raw[0] += 1;
                    hall_rotating_direction = +1;
                }
                if(event_falling_crossing[1]==TRUE){
                    event_falling_crossing[1] = FALSE; // �����¼���־λ
                    hall_qep_count -= 1; // ����Ϊ������������ת
                    hall_qep_count_raw[0] -= 1;
                    hall_rotating_direction = -1;
                }
            }
            // �Ա����½������¼����м�¼
            event_falling_crossing[0] = TRUE;
            recent_zero_crossing_hall_is = HALL_X;
        }
        last_pole[0] = current_pole[0]; // �����¼���־λ
    }
    /* HALL Y */
    if(current_pole[1]*last_pole[1]==-1){
        if(current_pole[1]==NORTH_POLE){

            if(recent_zero_crossing_hall_is==HALL_Y){
                // �������߷�ת�ˣ�
                // Do nothing
            }else if(recent_zero_crossing_hall_is==HALL_X){
                if(event_rising_crossing[0]==TRUE){
                    event_rising_crossing[0] = FALSE; // �����¼���־λ
                    hall_qep_count += 1; // �� HALL X һ��
                    hall_qep_count_raw[1] += 1;
                    hall_rotating_direction = +1;
                }
                if(event_falling_crossing[0]==TRUE){
                    event_falling_crossing[0] = FALSE;
                    hall_qep_count -= 1; // �� HALL X һ��
                    hall_qep_count_raw[1] -= 1;
                    hall_rotating_direction = -1;
                }
            }
            // �Ա������������¼����м�¼
            event_rising_crossing[1] = TRUE;
            recent_zero_crossing_hall_is = HALL_Y;

        }else if(current_pole[1]==SOUTH_POLE){

            if(recent_zero_crossing_hall_is==HALL_Y){
                // �������߷�ת�ˣ�
                // Do nothing
            }else if(recent_zero_crossing_hall_is==HALL_X){
                if(event_rising_crossing[0]==TRUE){
                    event_rising_crossing[0] = FALSE; // �����¼���־λ
                    hall_qep_count -= 1; // �� HALL X һ��
                    hall_qep_count_raw[1] -= 1;
                    hall_rotating_direction = -1;
                }
                if(event_falling_crossing[0]==TRUE){
                    event_falling_crossing[0] = FALSE; // �����¼���־λ
                    hall_qep_count += 1; // �� HALL X һ��
                    hall_qep_count_raw[1] += 1;
                    hall_rotating_direction = +1;
                }
            }
            // �Ա����½������¼����м�¼
            event_falling_crossing[1] = TRUE;
            recent_zero_crossing_hall_is = HALL_Y;
        }
        last_pole[1] = current_pole[1]; // �����¼���־λ
    }


    // ��¼�����С��������
    if(hall_sensor_read[0] > hall_sensor_read_max_global[0]) hall_sensor_read_max_global[0] = hall_sensor_read[0];
    if(hall_sensor_read[0] < hall_sensor_read_min_global[0]) hall_sensor_read_min_global[0] = hall_sensor_read[0];
    if(hall_sensor_read[1] > hall_sensor_read_max_global[1]) hall_sensor_read_max_global[1] = hall_sensor_read[1];
    if(hall_sensor_read[1] < hall_sensor_read_min_global[1]) hall_sensor_read_min_global[1] = hall_sensor_read[1];
    if(hall_sensor_read[2] > hall_sensor_read_max_global[2]) hall_sensor_read_max_global[2] = hall_sensor_read[2];
    if(hall_sensor_read[2] < hall_sensor_read_min_global[2]) hall_sensor_read_min_global[2] = hall_sensor_read[2];

    // register current read as last
    hall_sensor_read_last[0] = hall_sensor_read[0];
    hall_sensor_read_last[1] = hall_sensor_read[1];
    hall_sensor_read_last[2] = hall_sensor_read[1];
    hall_sensor_read_delta_last[0] = hall_sensor_read_delta[0];
    hall_sensor_read_delta_last[1] = hall_sensor_read_delta[1];
    hall_sensor_read_delta_last[2] = hall_sensor_read_delta[2];

    // �������������ѹΪת�ӽǶ�
    // �ٶȻ��ַ���
    // hall_theta_r_elec_incremental[0] += CL_TS*hall_omega_r_elec[0] * hall_rotating_direction;
    // hall_theta_r_elec_incremental[1] += CL_TS*hall_omega_r_elec[1] * hall_rotating_direction;
    // �ֲ�һ�Լ�ӳ�䷽��
    hall_theta_r_elec_incremental[0] = hall_qep_count * (0.015625 * 2*M_PI * MOTOR_NUMBER_OF_POLE_PAIRS);

    if(recent_zero_crossing_hall_is == HALL_X){
        hall_theta_r_elec_local_absolute[0] = hall_rotating_direction * fabs(hall_sensor_read[0]) * normalizer[0] * 0.5*M_PI * 1.375; // 1.375 = 22 / 16 (npp / st magnet pole pair number)
    }else if(recent_zero_crossing_hall_is == HALL_Y){
        hall_theta_r_elec_local_absolute[0] = hall_rotating_direction * fabs(hall_sensor_read[1]) * normalizer[1] * 0.5*M_PI * 1.375; // 1.375 = 22 / 16 (npp / st magnet pole pair number)        
    }
    hall_theta_r_elec_0 = hall_theta_r_elec_incremental[0] + hall_theta_r_elec_local_absolute[0];
    hall_omega_r_elec[0] = (hall_theta_r_elec_0 - hall_theta_r_elec_0_last) * CL_TS_INVERSE;
    hall_theta_r_elec_0_last = hall_theta_r_elec_0;

    hall_theta_r_elec[0] = fmod(hall_theta_r_elec_0, 2*M_PI);

    hall_qep_angle = hall_qep_count * (ONE_OVER_ST_MAGNETS * 0.5) * M_PI;
}
