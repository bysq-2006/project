/*********************************************************************************************************************
* gyro_z raw integration and scaled output
*********************************************************************************************************************/

#include "gyro_z_angle.h"

#define GYRO_Z_DEAD_ZONE_RAW        (6)
#define GYRO_Z_OUTPUT_SCALE         (0.002f)
#define GYRO_Z_INTEGRAL_LIMIT_RAW   (2137500)

static int16 gyro_z_raw;
static int32 gyro_z_integral;
static float gyro_z_output;

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
}

void gyro_z_angle_init(void)
{
    gyro_z_angle_reset();
}

void gyro_z_angle_update(uint16 dt_ms)
{
    int16 gyro_z_calc;

    imu660rb_get_gyro();
    gyro_z_raw = imu660rb_gyro_z;
    gyro_z_calc = ((gyro_z_raw > -GYRO_Z_DEAD_ZONE_RAW) && (gyro_z_raw < GYRO_Z_DEAD_ZONE_RAW)) ? 0 : gyro_z_raw;
    gyro_z_integral += (int32)gyro_z_calc * dt_ms;
    gyro_z_integral = gyro_z_limit_integral(gyro_z_integral);
    gyro_z_output = (float)gyro_z_integral * GYRO_Z_OUTPUT_SCALE;
}

void gyro_z_angle_set_w(int8 w)
{
    (void)w;
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
