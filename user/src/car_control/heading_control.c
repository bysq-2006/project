/*********************************************************************************************************************
* Heading hold control.
*********************************************************************************************************************/

#include "heading_control.h"
#include "car_control.h"

#define HEADING_KP_PERCENT              (80)
#define HEADING_DEADBAND_DEG_X10        (10)
#define HEADING_TURN_MIN                (10)
#define HEADING_TURN_MAX                (35)
#define HEADING_UPDATE_PERIOD_S         (0.01f)
#define HEADING_GYRO_Z_SIGN             (1.0f)
#define HEADING_GYRO_CALIBRATION_COUNT  (100)

static uint8 heading_ready = 0;
static float heading_target_yaw = 0.0f;
static float heading_current_yaw = 0.0f;
static float heading_gyro_z_bias = 0.0f;

static float heading_normalize_yaw(float yaw)
{
    while(yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while(yaw < 0.0f)
    {
        yaw += 360.0f;
    }

    return yaw;
}

static float heading_calc_error(float target, float now)
{
    float error = heading_normalize_yaw(target) - heading_normalize_yaw(now);

    while(error > 180.0f)
    {
        error -= 360.0f;
    }
    while(error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

uint8 heading_control_init(void)
{
    uint8 state = imu660rb_init();
    uint16 i;
    float gyro_z_sum = 0.0f;

    heading_ready = (0 == state);
    if(heading_ready)
    {
        for(i = 0; i < HEADING_GYRO_CALIBRATION_COUNT; i ++)
        {
            imu660rb_get_gyro();
            gyro_z_sum += imu660rb_gyro_transition(imu660rb_gyro_z);
            system_delay_ms(5);
        }

        heading_gyro_z_bias = gyro_z_sum / HEADING_GYRO_CALIBRATION_COUNT;
        heading_current_yaw = 0.0f;
        heading_target_yaw = 0.0f;
    }

    return state;
}

void heading_control_lock_current(void)
{
    heading_target_yaw = heading_normalize_yaw(heading_current_yaw);
}

void heading_control_set_target(float yaw_deg)
{
    heading_target_yaw = heading_normalize_yaw(yaw_deg);
}

void car_move_xy_heading(int8 x, int8 y)
{
    int16 turn = 0;
    float error;
    float gyro_z_dps;

    if(heading_ready)
    {
        imu660rb_get_gyro();
        gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z) - heading_gyro_z_bias;
        heading_current_yaw = heading_normalize_yaw(heading_current_yaw + gyro_z_dps * HEADING_UPDATE_PERIOD_S * HEADING_GYRO_Z_SIGN);

        error = heading_calc_error(heading_target_yaw, heading_current_yaw);
        if((error * 10.0f > HEADING_DEADBAND_DEG_X10) || (error * 10.0f < -HEADING_DEADBAND_DEG_X10))
        {
            turn = (int16)(error * HEADING_KP_PERCENT / 100.0f);
            if(turn > HEADING_TURN_MAX)
            {
                turn = HEADING_TURN_MAX;
            }
            else if(turn < -HEADING_TURN_MAX)
            {
                turn = -HEADING_TURN_MAX;
            }
            else if((turn > 0) && (turn < HEADING_TURN_MIN))
            {
                turn = HEADING_TURN_MIN;
            }
            else if((turn < 0) && (turn > -HEADING_TURN_MIN))
            {
                turn = -HEADING_TURN_MIN;
            }
        }
    }

    car_move_xyw(x, y, (int8)turn);
}
