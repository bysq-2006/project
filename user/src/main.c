#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define MAIN_LOOP_DELAY_MS      (50)

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();
    openart_uart_init();
    openart_display_init();

    while(1)
    {
        openart_uart_update();
        openart_display_update();
        system_delay_ms(MAIN_LOOP_DELAY_MS);
    }
}
