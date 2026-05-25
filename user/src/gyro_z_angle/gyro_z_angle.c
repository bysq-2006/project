/*********************************************************************************************************************
* gyro_z 原始值积分与缩放输出
*********************************************************************************************************************/

#include "gyro_z_angle.h"
#include "../screen_print/screen_print.h"
#include "../car_params.h"
#include <stdio.h>

#define GYRO_Z_PRINT_BUFFER_SIZE    (32)

static int16 gyro_z_raw;
static int32 gyro_z_integral;
static float gyro_z_output;
static int8 gyro_z_control_w;
static char gyro_z_print_buffer[GYRO_Z_PRINT_BUFFER_SIZE];

static int32 float_to_milli_int(float value)
{
    if(value >= 0.0f)
    {
        return (int32)(value * 1000.0f + 0.5f);
    }

    return (int32)(value * 1000.0f - 0.5f);
}

static int32 gyro_z_limit_integral(int32 value)
{
    if((GYRO_Z_INTEGRAL_LIMIT_RAW > 0) && (value > GYRO_Z_INTEGRAL_LIMIT_RAW))
    {
        return GYRO_Z_INTEGRAL_LIMIT_RAW;
    }

    if((GYRO_Z_INTEGRAL_LIMIT_RAW > 0) && (value < -GYRO_Z_INTEGRAL_LIMIT_RAW))
    {
        return -GYRO_Z_INTEGRAL_LIMIT_RAW;
    }

    return value;
}

void gyro_z_angle_reset(void)
{
    gyro_z_raw = 0;
    gyro_z_integral = 0;
    gyro_z_output = 0.0f;
    gyro_z_control_w = 0;
}

void gyro_z_angle_init(void)
{
    gyro_z_angle_reset();
    screen_print_init();
    screen_print_line(0, "gyro_z raw");
}

void gyro_z_angle_update(uint16 dt_ms)
{
    int16 gyro_z_calc;
    int32 output_milli;
    char output_sign = ' ';

    imu660rb_get_gyro();
    gyro_z_raw = imu660rb_gyro_z;
    gyro_z_calc = ((gyro_z_raw > -GYRO_Z_DEAD_ZONE_RAW) && (gyro_z_raw < GYRO_Z_DEAD_ZONE_RAW)) ? 0 : gyro_z_raw;
    gyro_z_integral += (int32)gyro_z_calc * dt_ms;
    gyro_z_integral = gyro_z_limit_integral(gyro_z_integral);
    gyro_z_output = (float)gyro_z_integral * GYRO_Z_OUTPUT_SCALE;

    screen_print_line(0, "IMU660RB gyro raw");
    sprintf(gyro_z_print_buffer, "gyro_z:%6d", gyro_z_raw);
    screen_print_line(1, gyro_z_print_buffer);
    sprintf(gyro_z_print_buffer, "int_z:%ld", (long)gyro_z_integral);
    screen_print_line(2, gyro_z_print_buffer);

    output_milli = float_to_milli_int(gyro_z_output);
    if(output_milli < 0)
    {
        output_sign = '-';
        output_milli = -output_milli;
    }
    sprintf(gyro_z_print_buffer, "out_z:%c%ld.%03ld", output_sign, (long)(output_milli / 1000), (long)(output_milli % 1000));
    screen_print_line(3, gyro_z_print_buffer);
    sprintf(gyro_z_print_buffer, "w:%4d", gyro_z_control_w);
    screen_print_line(4, gyro_z_print_buffer);
}

void gyro_z_angle_set_w(int8 w)
{
    gyro_z_control_w = w;
}

int16 gyro_z_angle_get_raw(void)
{
    return gyro_z_raw;
}

int32 gyro_z_angle_get_integral(void)
{
    return gyro_z_integral;
}

float gyro_z_angle_get_output(void)
{
    return gyro_z_output;
}
