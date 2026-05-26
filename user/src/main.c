#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "gyro_z_angle/gyro_z_angle.h"

#define TEST_SPEED      (14)
#define TEST_TIME_MS    (2000)
#define CONTROL_DT_MS   (10)

static void heading_test_move(int8 x, int8 y, uint16 time_ms)
{
    uint16 elapsed_ms = 0;

    while(elapsed_ms < time_ms)
    {
        gyro_z_angle_update(CONTROL_DT_MS);
        heading_sensor_update(x, y);
        system_delay_ms(CONTROL_DT_MS);
        elapsed_ms += CONTROL_DT_MS;
    }
}

int main(void)
{
    uint8 imu_state;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();

    imu_state = heading_sensor_init();
    if(imu_state)
    {
        car_stop();
        while(1)
        {
            system_delay_ms(200);
        }
    }

    gyro_z_angle_init();

    heading_test_move(-TEST_SPEED, 0, TEST_TIME_MS);
    heading_test_move(TEST_SPEED, 0, TEST_TIME_MS);

    while(1)
    {
        gyro_z_angle_update(CONTROL_DT_MS);
        heading_sensor_update(0, 0);
        system_delay_ms(CONTROL_DT_MS);
    }
}
