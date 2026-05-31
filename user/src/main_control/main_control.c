#include "main_control.h"

uint16 main_control_find_boxes(const openart_map_t *map, main_control_map_pos_t *boxes, uint16 max_boxes)
{
    uint8 row;
    uint8 col;
    uint16 count;
    uint16 index;

    count = 0;

    if((0 == map) || (0 == boxes) || (0 == max_boxes) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    for(row = 0; row < map->rows; row++)
    {
        for(col = 0; col < map->cols; col++)
        {
            index = (uint16)row * map->cols + col;
            if(OPENART_CELL_YELLOW_BOX == map->cells[index])
            {
                if(count < max_boxes)
                {
                    boxes[count].x = col;
                    boxes[count].y = row;
                }
                count++;
            }
        }
    }

    return count;
}

uint16 main_control_find_goals(const openart_map_t *map, main_control_map_pos_t *goals, uint16 max_goals)
{
    uint8 row;
    uint8 col;
    uint16 count;
    uint16 index;

    count = 0;

    if((0 == map) || (0 == goals) || (0 == max_goals) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    for(row = 0; row < map->rows; row++)
    {
        for(col = 0; col < map->cols; col++)
        {
            index = (uint16)row * map->cols + col;
            if(OPENART_CELL_GOAL == map->cells[index])
            {
                if(count < max_goals)
                {
                    goals[count].x = col;
                    goals[count].y = row;
                }
                count++;
            }
        }
    }

    return count;
}
