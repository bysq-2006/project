#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define DISPLAY_IDLE_LOOP_COUNT     (50000)
#define DISPLAY_PACKET_STEP         (4)

int main(void)
{
    uint16 last_packet_count = 0;
    uint8 pending_packets = 0;
    uint32 idle_loop_count = 0;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();
    openart_uart_init();
    openart_display_init();

    while(1)
    {
        openart_uart_update();

        if(openart_uart_status.packet_count != last_packet_count)
        {
            last_packet_count = openart_uart_status.packet_count;
            pending_packets++;
            idle_loop_count = 0;
        }
        else if(idle_loop_count < DISPLAY_IDLE_LOOP_COUNT)
        {
            idle_loop_count++;
        }

        if((openart_map.updated) ||
           (pending_packets >= DISPLAY_PACKET_STEP) ||
           (idle_loop_count >= DISPLAY_IDLE_LOOP_COUNT))
        {
            openart_display_update();
            openart_uart_clear_updated();
            pending_packets = 0;
            idle_loop_count = 0;
        }
    }
}
