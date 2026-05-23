/*********************************************************************************************************************
* Heading hold control.
*********************************************************************************************************************/

#include "heading_control.h"
#include "car_control.h"

#define HEADING_QUARTERNION_RATE        (IMU660RC_QUARTERNION_120HZ)
#define HEADING_KP_PERCENT              (80)
#define HEADING_DEADBAND_DEG_X10        (10)
#define HEADING_TURN_MAX                (25)

static uint8 heading_ready = 0;
static float heading_target_yaw = 0.0f;

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
    uint8 state = imu660rc_init(HEADING_QUARTERNION_RATE);

    heading_ready = (0 == state);
    if(heading_ready)
    {
        system_delay_ms(100);
        heading_target_yaw = heading_normalize_yaw(imu660rc_yaw);
    }

    return state;
}

void heading_control_lock_current(void)
{
    heading_target_yaw = heading_normalize_yaw(imu660rc_yaw);
}

void heading_control_set_target(float yaw_deg)
{
    heading_target_yaw = heading_normalize_yaw(yaw_deg);
}

void car_move_xy_heading(int8 x, int8 y)
{
    int16 turn = 0;
    float error;

    if(heading_ready)
    {
        error = heading_calc_error(heading_target_yaw, imu660rc_yaw);
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
        }
    }

    car_move_xyw(x, y, (int8)turn);
}
