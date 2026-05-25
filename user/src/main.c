#include "zf_common_headfile.h"
#include "car_control/car_control.h"

#define TEST_SPEED      (14)

static void car_test_step(int8 x, int8 y, int8 w)
{
    car_move_xyw(x, y, w);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();

    while(1)
    {
        car_test_step(TEST_SPEED, 0, 0);
        system_delay_ms(10);
    }
}
