#include "zf_common_headfile.h"
#include "car_control/path_follow_control_local.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

// 加载一份固定地图和车位姿，用于本地测试主控逻辑。
static void load_fixed_openart_data(void)
{
    uint16 i;
    uint8 row;
    uint8 col;
    static const uint8 fixed_map[16][12] =
    {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 2, 0, 0, 0, 0, 0, 3, 0, 1},
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 1, 3, 0, 0, 1, 0, 1},
        {1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1},
        {1, 0, 1, 0, 2, 2, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 1, 1, 3, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    openart_pose.valid = 1;
    openart_pose.updated = 1;
    openart_pose.seq = 0;
    openart_pose.x10 = 150;
    openart_pose.y10 = 650;
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

// 程序入口；初始化测试数据后循环调用主控状态机。
int main(void)
{
    static main_control_context_t main_control;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    openart_display_init();
    load_fixed_openart_data();
    main_control_init(&main_control);

    while(1)
    {
        main_control_update_local(&main_control, &openart_pose, &openart_map);
        openart_display_update();
        system_delay_ms(20);
    }
}
