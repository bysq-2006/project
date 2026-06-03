/*********************************************************************************************************************
* IMU660RB 角度传感器基础框架
*********************************************************************************************************************/

#include "heading_control.h"
#include "car_control.h"
#include "../gyro_z_angle/gyro_z_angle.h"

// P：角度误差比例项，越大修正越快，过大容易抖动或过冲。
#define HEADING_CONTROL_P                   (2.30f)
// I：累计误差积分项，用来消除长期偏差，过大容易越积越猛。
#define HEADING_CONTROL_I                   (0.0280f)
// D：误差变化微分项，用来抑制过冲，过大容易对噪声敏感。
#define HEADING_CONTROL_D                   (14.0f)
// I_LIMIT：积分项累计上限，限制 I 项过度累积。
#define HEADING_CONTROL_I_LIMIT             (178.6f)
// W_LIMIT：最终旋转输出最大值，限制给底盘的最大转向力度。
#define HEADING_CONTROL_W_LIMIT             (100.0f)
// W_MIN：最终旋转输出最小值，用来克服电机小占空比不动的问题，0 表示关闭。
#define HEADING_CONTROL_W_MIN               (0.0f)
// OUTPUT_GAIN：PID 总输出倍率，整体放大或缩小修正力度。
#define HEADING_CONTROL_OUTPUT_GAIN         (1.0f)
// OUTPUT_DIR：PID 输出方向
#define HEADING_CONTROL_OUTPUT_DIR          (1.0f)

// 注意单位不是弧度，也不是角度
static uint8 heading_sensor_ready = 0;
static int16 heading_initial_x_raw = 0;
static float heading_target_angle = 0.0f;
static float heading_x_raw_error = 0.0f;
static float heading_last_x_raw_error = 0.0f;
static float heading_x_raw_error_sum = 0.0f;

// 读取 IMU 原始数据
static void heading_sensor_read_raw(void)
{
    imu660rb_get_acc();
    imu660rb_get_gyro();
}

// 限制旋转输出范围
static int8 heading_limit_w(float w)
{
    if((w > 0.0f) && (w < HEADING_CONTROL_W_MIN))
    {
        return (int8)HEADING_CONTROL_W_MIN;
    }

    if((w < 0.0f) && (w > -HEADING_CONTROL_W_MIN))
    {
        return (int8)-HEADING_CONTROL_W_MIN;
    }

    if(w > HEADING_CONTROL_W_LIMIT)
    {
        return (int8)HEADING_CONTROL_W_LIMIT;
    }

    if(w < -HEADING_CONTROL_W_LIMIT)
    {
        return (int8)-HEADING_CONTROL_W_LIMIT;
    }

    return (int8)w;
}

// 累加并限制积分项
static void heading_update_integral(void)
{
    heading_x_raw_error_sum += heading_x_raw_error;

    if(heading_x_raw_error_sum > HEADING_CONTROL_I_LIMIT)
    {
        heading_x_raw_error_sum = HEADING_CONTROL_I_LIMIT;
    }
    else if(heading_x_raw_error_sum < -HEADING_CONTROL_I_LIMIT)
    {
        heading_x_raw_error_sum = -HEADING_CONTROL_I_LIMIT;
    }
}

// 初始化传感器并记录初始值
uint8 heading_sensor_init(void)
{
    uint8 state = imu660rb_init();

    heading_sensor_ready = (0 == state);

    if(heading_sensor_ready)
    {
        heading_sensor_read_raw();
        heading_initial_x_raw = imu660rb_acc_x;
    }

    return state;
}

// 更新传感器并执行 PI 控制
// 更新传感器并执行 PID 控制
void heading_sensor_update(int8 x, int8 y, int8 w)
{
    if(heading_sensor_ready)
    {
        int8 heading_w;
        float target_angle;
        float control_output;

        heading_sensor_read_raw();
        target_angle = (float)w;
        if(target_angle != heading_target_angle)
        {
            heading_target_angle = target_angle;
            heading_x_raw_error_sum = 0.0f;
            heading_last_x_raw_error = 0.0f;
        }

        heading_last_x_raw_error = heading_x_raw_error;
        heading_x_raw_error = gyro_z_angle_get_output() - heading_target_angle;
        heading_update_integral();
        control_output = ((float)heading_x_raw_error * HEADING_CONTROL_P
            + heading_x_raw_error_sum * HEADING_CONTROL_I
            + (float)(heading_x_raw_error - heading_last_x_raw_error) * HEADING_CONTROL_D)
            * HEADING_CONTROL_OUTPUT_GAIN * HEADING_CONTROL_OUTPUT_DIR;
        heading_w = heading_limit_w(control_output);
        w = heading_w;
        gyro_z_angle_set_w(w);
        car_move_xyw(x, y, w);
    }
}
