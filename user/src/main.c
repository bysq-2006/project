#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "gyro_z_angle/gyro_z_angle.h"

#define TEST_SPEED      (14)

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

    while(1)
    {
        gyro_z_angle_update(10);
        heading_sensor_update(TEST_SPEED, 0);
        system_delay_ms(10);
    }
}
