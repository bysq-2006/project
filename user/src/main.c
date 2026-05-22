#include "zf_common_headfile.h"
#include "car_control.h"

#define TEST_MOVE_SPEED             (20)

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(300);

    car_init();
    system_delay_ms(100);

    // x > 0 为向右平移，y = 0 表示不前进/后退。
    car_move_xy(TEST_MOVE_SPEED, 0);

    while(1)
    {
        system_delay_ms(100);
    }
}
