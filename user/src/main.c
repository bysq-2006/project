#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

static void load_fixed_openart_data(void)
{
    uint16 i;
    uint8 row;
    uint8 col;
    static const uint8 fixed_map[16][12] =
    {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1},
        {1, 0, 2, 0, 1, 0, 3, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1},
        {1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 3, 0, 1, 0, 0, 0, 2, 1, 0, 1},
        {1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 3, 0, 1},
        {1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 2, 1},
        {1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    openart_pose.valid = 1;
    openart_pose.updated = 1;
    openart_pose.seq = 0;
    openart_pose.x10 = 550;
    openart_pose.y10 = 750;
    openart_pose.angle10 = 900;

    openart_map.valid = 1;
    openart_map.updated = 1;
    openart_map.seq = 0;
    openart_map.cols = 12;
    openart_map.rows = 16;
    openart_map.width10 = 1200;
    openart_map.height10 = 1600;

    for(i = 0; i < OPENART_MAP_CELL_MAX; i++)
    {
        openart_map.cells[i] = OPENART_CELL_BACKGROUND;
    }

    for(row = 0; row < openart_map.rows; row++)
    {
        for(col = 0; col < openart_map.cols; col++)
        {
            openart_map.cells[row * openart_map.cols + col] = fixed_map[row][col];
        }
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();
    openart_uart_init();
    openart_display_init();
    load_fixed_openart_data();

    while(1)
    {
        /* openart_uart_update(); */
        openart_display_update();
    }
}
