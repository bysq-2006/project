#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "screen_print/screen_print.h"

int8 current_x = 5;
int8 current_y = 0;

int main(void)
{
    uint8 imu_state;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();

    imu_state = heading_control_init();
    heading_control_lock_current();

    if(imu_state)
    {
        while(1)
        {
            screen_print_line(0, "IMU660RB init ERR");
            screen_print_line(1, "check sensor type");
            screen_print_line(2, "or wiring pins");
            system_delay_ms(200);
        }
    }

    while(1)
    {
        car_move_xy_heading(current_x, current_y);
        system_delay_ms(10);
    }
}
