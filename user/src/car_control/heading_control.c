/*********************************************************************************************************************
* IMU660RB 角度传感器基础框架
*********************************************************************************************************************/

#include "heading_control.h"
#include "car_control.h"
#include "../car_params.h"

// 注意单位不是弧度，也不是角度
static uint8 heading_sensor_ready = 0;
static int16 heading_initial_x_raw = 0;
static int16 heading_x_raw_error = 0;
static int16 heading_last_x_raw_error = 0;
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
    if(w > 100.0f)
    {
        return 100;
    }

    if(w < -100.0f)
    {
        return -100;
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
void heading_sensor_update(int8 x, int8 y)
{
    if(heading_sensor_ready)
    {
        int8 w;

        heading_sensor_read_raw();
        heading_last_x_raw_error = heading_x_raw_error;
        heading_x_raw_error = imu660rb_acc_x - heading_initial_x_raw;
        heading_update_integral();
        w = heading_limit_w((float)heading_x_raw_error * HEADING_CONTROL_P
            + heading_x_raw_error_sum * HEADING_CONTROL_I
            + (float)(heading_x_raw_error - heading_last_x_raw_error) * HEADING_CONTROL_D);
        car_move_xyw(x, y, w);
    }
}
