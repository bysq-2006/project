/*********************************************************************************************************************
* RT1064DVL6A Open Source Library
* main.c
*********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "car_control.h"

#define SIDE_MOVE_SPEED             (15)
#define SIDE_MOVE_TIME_MS           (800)

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    car_init();
    interrupt_global_enable(0);

    while(1)
    {
        car_move_xy(SIDE_MOVE_SPEED, 0);
        system_delay_ms(SIDE_MOVE_TIME_MS);
    }
}
