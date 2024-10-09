#include "All_Definition.h"


#if WHO_IS_USER == USER_ZJL

void control_two_motor_position()
{
    // debug->flag_overwrite_theta_d = FALSE;
    Axis->Set_current_loop = FALSE;

    if (axisCnt == 0)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
        PID_Position->Ref = hip_shank_angle_to_can(
            curycontroller.theta1 + curycontroller.theta2,
            SHANK_TYPE);
#endif
    }
    if (axisCnt == 1)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
        PID_Position->Ref = hip_shank_angle_to_can(
            curycontroller.theta1,
            HIP_TYPE);
#endif
    }
    // 位置环
    // 长弧和短弧，要选短的。
    PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
    if (PID_Position->Err > (CAN_QMAX * 0.5))
    {
        PID_Position->Err -= CAN_QMAX;
    }
    if (PID_Position->Err < -(CAN_QMAX * 0.5))
    {
        PID_Position->Err += CAN_QMAX;
    }
    if (axisCnt == 0)
    {
    #if NUMBER_OF_AXES == 2
        PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * (curycontroller.dot_theta1+curycontroller.dot_theta2) * 60 * 0.15915494309189535;
    #endif
    }
    if (axisCnt == 1)
    {
    #if NUMBER_OF_AXES == 2
        PID_Position->Out = PID_Position->Err * PID_Position->Kp + 32 * curycontroller.dot_theta1* 60 * 0.15915494309189535;
    #endif
    }
    if (PID_Position->Out > PID_Position->OutLimit)
    {
        PID_Position->Out = PID_Position->OutLimit;
    }
    if (PID_Position->Out < -PID_Position->OutLimit)
    {
        PID_Position->Out = -PID_Position->OutLimit;
    }
    if (axisCnt == 1)
        Axis->Set_manual_rpm = -PID_Position->Out;
    else
        Axis->Set_manual_rpm = PID_Position->Out;
}

void control_single_motor_position()
{
    // Axis->flag_overwrite_theta_d = FALSE;
    Axis->Set_current_loop = FALSE;
    // 位置环
    // 长弧和短弧，要选短的。
    if (axisCnt == 0)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x03_fromCPU2;
        PID_Position->Ref = target_position_cnt_shank;
#endif
    }
    if (axisCnt == 1)
    {
#if NUMBER_OF_AXES == 2
        PID_Position->Fbk = position_count_CAN_ID0x01_fromCPU2;
        PID_Position->Ref = target_position_cnt_hip;
#endif
    }
    PID_Position->Err = PID_Position->Ref - PID_Position->Fbk;
    if (PID_Position->Err > (CAN_QMAX * 0.5))
    {
        PID_Position->Err -= CAN_QMAX;
    }
    if (PID_Position->Err < -(CAN_QMAX * 0.5))
    {
        PID_Position->Err += CAN_QMAX;
    }
    PID_Position->Out = PID_Position->Err * PID_Position->Kp;
    if (PID_Position->Out > PID_Position->OutLimit)
    {
        PID_Position->Out = PID_Position->OutLimit;
    }
    if (PID_Position->Out < -PID_Position->OutLimit)
    {
        PID_Position->Out = -PID_Position->OutLimit;
    }
    if (axisCnt == 1)
        Axis->Set_manual_rpm = -PID_Position->Out;
    else
        Axis->Set_manual_rpm = PID_Position->Out;
}

void run_shank_loop()
{ // shank motor only
    if (axisCnt == 0)
    {
        // Axis->flag_overwrite_theta_d = FALSE;

        //            Axis->Set_current_loop = TRUE;
        //            if (position_count_CAN_ID0x03_fromCPU2 > 62000)
        //            {
        //                Axis->Set_manual_current_iq = -1;
        //            }
        //            else if (position_count_CAN_ID0x03_fromCPU2 < 33000)
        //            {
        //                Axis->Set_manual_current_iq = 1;
        //            }

        Axis->Set_current_loop = FALSE;
        if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MIN)
        {
            Axis->Set_manual_rpm = -legBouncingSpeed;
        }
        else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MAX)
        {
            Axis->Set_manual_rpm = legBouncingSpeed;
        }
    }
    if (axisCnt == 1)
    {
        // Axis->flag_overwrite_theta_d = FALSE;

        //            Axis->Set_current_loop = TRUE;
        //            if (position_count_CAN_ID0x03_fromCPU2 > 62000)
        //            {
        //                Axis->Set_manual_current_iq = -1;
        //            }
        //            else if (position_count_CAN_ID0x03_fromCPU2 < 33000)
        //            {
        //                Axis->Set_manual_current_iq = 1;
        //            }

        Axis->Set_current_loop = FALSE;
        if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
        {
            Axis->Set_manual_rpm = legBouncingSpeed;
        }
        else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
        {
            Axis->Set_manual_rpm = -legBouncingSpeed;
        }
    }

}

void run_hip_loop()
{ // hip motor only
#if NUMBER_OF_AXES == 2
    if (axisCnt == 1)
    {

        // Axis->flag_overwrite_theta_d = FALSE;

        if (bool_TEMP == TRUE)
        {
            // Axis->flag_overwrite_theta_d = FALSE;
            Axis->Set_current_loop = TRUE;
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
            {
                Axis->Set_manual_current_iq = hipBouncingIq;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
            {
                Axis->Set_manual_current_iq = -hipBouncingIq;
            }
        }
        else
        {
            // Axis->flag_overwrite_theta_d = FALSE;
            Axis->Set_current_loop = FALSE;
            if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MIN)
            {
                Axis->Set_manual_rpm = -legBouncingSpeed;
            }
            else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MAX)
            {
                Axis->Set_manual_rpm = legBouncingSpeed;
            }
        }

        //
        //               Axis_2.flag_overwrite_theta_d = TRUE;
        //                Axis_2.Set_current_loop = TRUE;
        //                Axis_2.Set_manual_current_iq = 5;
        //                if (position_count_CAN_ID0x01_fromCPU2 > 58000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = 10;
        //                }
        //                else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
        //                {
        //                    Axis_2.Overwrite_Current_Frequency = -10;
        //                }
    }
#else
    // Axis->flag_overwrite_theta_d = TRUE;
    Axis->Set_current_loop = TRUE;
    Axis->Set_manual_current_iq = 5;
    if (position_count_CAN_ID0x01_fromCPU2 > 58000)
    {
        Axis->Overwrite_Current_Frequency = hipBouncingFreq;
    }
    else if (position_count_CAN_ID0x01_fromCPU2 < 48000)
    {
        Axis->Overwrite_Current_Frequency = -hipBouncingFreq;
    }
#endif
}

void run_both_loop()
{
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0)
    {
        // CAN03越大，小腿越伸出，legBouncingIq, rpm为正数，小腿伸出
        // CAN03越小，小腿越收起，legBouncingIq, rpm为负数，小腿收起
        // Axis_1.flag_overwrite_theta_d = FALSE;
        Axis_1.Set_current_loop = TRUE;
        if (position_count_CAN_ID0x03_fromCPU2 > CAN03_MAX)
        {
            Axis_1.Set_manual_current_iq = legBouncingIq; // CAN03太大，小腿伸出过头，需要iq为负数，收起小腿
        }
        else if (position_count_CAN_ID0x03_fromCPU2 < CAN03_MIN)
        {
            Axis_1.Set_manual_current_iq = -legBouncingIq;
        }
    }
    // 大腿电机
    if (axisCnt == 1)
    {
        // CAN01越小,大腿越抬起,hipBouncingIq为正数，大腿抬起
        // CAN01越大,大腿越放下,hipBouncingIq为负数，大腿放下
        // Axis_2.flag_overwrite_theta_d = FALSE;
        Axis_2.Set_current_loop = TRUE;
        if (position_count_CAN_ID0x01_fromCPU2 > CAN01_MAX)
        {
            Axis_2.Set_manual_current_iq = hipBouncingIq; // CAN01太大,大腿放下过多,需要iq为正数,抬起大腿
        }
        else if (position_count_CAN_ID0x01_fromCPU2 < CAN01_MIN)
        {
            Axis_2.Set_manual_current_iq = -hipBouncingIq;
        }
    }
#endif
}

void run_impedance_control()
{
#if NUMBER_OF_AXES == 2
    if (axisCnt == 0)
    {
        // Axis_1.flag_overwrite_theta_d = FALSE;
        Axis_1.Set_current_loop = TRUE;
        Axis_1.Set_manual_current_iq = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
    if (axisCnt == 1)
    {
        // Axis_2.flag_overwrite_theta_d = FALSE;
        Axis_2.Set_current_loop = TRUE;
        Axis_2.Set_manual_current_iq = Impendence_Control_cal(
                axisCnt,
                position_count_CAN_ID0x01_fromCPU2,
                position_count_CAN_ID0x03_fromCPU2
                );
    }
#endif
}

// this offset is moved to ACMconfig.h
// #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19

#define NO_POSITION_CONTROL 0
#define TWOMOTOR_POSITION_CONTROL 1
#define SINGLE_POSITION_CONTROL 2
#define SHANK_LOOP_RUN 3
#define HIP_LOOP_RUN 4
#define BOTH_LOOP_RUN 5
#define IMPEDANCE_CONTROL 6

// moved to global variable [debug]
//int positionLoopType = 0;           // TWOMOTOR_POSITION_CONTROL; //SHANK_LOOP_RUN; // SHANK_LOOP_RUN; //BOTH_LOOP_RUN;
//int use_first_set_three_phase = 1; //-1 for both motors

REAL call_position_loop_controller(int positionLoopType)
{
    CTRL_1.s->Speed->Kp = TEST_SHANK_SPD_KP;
    CTRL_1.s->Speed->Ki_CODE = TEST_SHANK_SPD_KI;

    CTRL_2.s->Speed->Kp = TEST_HIP_SPD_KP;
    CTRL_2.s->Speed->Ki_CODE = TEST_HIP_SPD_KI;


    CTRL_1.s->Position->Kp = TEST_SHANK_KP;
    CTRL_2.s->Position->Kp = TEST_HIP_KP;

    CTRL_1.s->Position->OutLimit = TEST_SHANK_POS_OUTLIMIT;
    CTRL_2.s->Position->OutLimit = TEST_HIP_POS_OUTLIMIT;

    run_iecon_main((*CTRL).timebase_counter);

    if (positionLoopType == TWOMOTOR_POSITION_CONTROL) control_two_motor_position();

    else if (positionLoopType == SINGLE_POSITION_CONTROL) control_single_motor_position();

    else if (positionLoopType == SHANK_LOOP_RUN) run_shank_loop();

    else if (positionLoopType == HIP_LOOP_RUN) run_hip_loop();

    else if (positionLoopType == BOTH_LOOP_RUN) run_both_loop();

    else if (positionLoopType == IMPEDANCE_CONTROL) run_impedance_control();


    return Axis->Set_manual_rpm;
}


#endif
