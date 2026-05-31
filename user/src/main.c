#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "main_control/main_control.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define BOX_PATH_TURN_COST          (10U)

static uint8 main_mark_path_on_map(openart_map_t *map,
                                   const main_control_map_pos_t *path,
                                   main_control_map_pos_t target)
{
    uint16 i;
    uint16 index;

    if((0 == map) || (0 == path) || (!map->valid) || (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    for(i = 1; i < OPENART_MAP_CELL_MAX; i++)
    {
        if((path[i].x >= map->cols) || (path[i].y >= map->rows))
        {
            break;
        }

        index = (uint16)path[i].y * map->cols + path[i].x;
        map->cells[index] = OPENART_CELL_CROSS;

        if((path[i].x == target.x) && (path[i].y == target.y))
        {
            break;
        }
    }

    return 1;
}


static uint8 main_find_best_box_path(const openart_map_t *map,
                                     main_control_map_pos_t start,
                                     const main_control_map_pos_t *goals,
                                     uint16 goal_count,
                                     main_control_map_pos_t *best_path,
                                     main_control_map_pos_t *best_target)
{
    main_control_map_pos_t candidate_path[OPENART_MAP_CELL_MAX];
    uint32 best_cost;
    uint32 cost;
    uint16 i;
    uint16 j;

    if((0 == map) || (0 == goals) || (0 == best_path) || (0 == best_target) || (0 == goal_count))
    {
        return 0;
    }

    best_cost = MAIN_CONTROL_PATH_COST_INVALID;
    for(i = 0; i < goal_count; i++)
    {
        cost = main_control_astar_find_path(map, start, goals[i], candidate_path, BOX_PATH_TURN_COST);
        if(cost < best_cost)
        {
            best_cost = cost;
            *best_target = goals[i];
            for(j = 0; j < OPENART_MAP_CELL_MAX; j++)
            {
                best_path[j] = candidate_path[j];
            }
        }
    }

    return (best_cost != MAIN_CONTROL_PATH_COST_INVALID);
}

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
    openart_map_t search_map;
    openart_map_t display_map;
    main_control_map_pos_t boxes[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t goals[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t best_path[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t best_target;
    uint16 box_count;
    uint16 goal_count;
    uint16 box_index;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    car_stop();
    openart_uart_init();
    openart_display_init();
    load_fixed_openart_data();

    search_map = openart_map;
    display_map = openart_map;

    box_count = main_control_find_boxes(&search_map, boxes, OPENART_MAP_CELL_MAX);
    goal_count = main_control_find_goals(&search_map, goals, OPENART_MAP_CELL_MAX);

    for(box_index = 0; box_index < box_count; box_index++)
    {
        if(main_find_best_box_path(&search_map, boxes[box_index], goals, goal_count, best_path, &best_target))
        {
            main_mark_path_on_map(&display_map, best_path, best_target);
            openart_map = display_map;
            openart_display_update();
        }
    }

    while(1)
    {
        /* openart_uart_update(); */
        openart_display_update();
    }
}
