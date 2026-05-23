#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"

#define SWITCH_RUN                  (C26)

int8 current_x = 0;
int8 current_y = 0;

static uint8 switch_is_on(gpio_pin_enum pin)
{
    return (GPIO_LOW == gpio_get_level(pin));
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    heading_control_init();
    heading_control_lock_current();

    gpio_init(SWITCH_RUN, GPI, GPIO_HIGH, GPI_PULL_UP);

    while(1)
    {
        if(switch_is_on(SWITCH_RUN))
        {
            car_move_xy_heading(current_x, current_y);
        }
        else
        {
            car_stop();
        }

        system_delay_ms(10);
    }
}
