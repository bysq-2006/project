#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "zf_common_headfile.h"
#include "../openart_uart/openart_uart.h"

typedef struct
{
    uint8 x;
    uint8 y;
} main_control_map_pos_t;

#define MAIN_CONTROL_PATH_COST_INVALID  (0xFFFFFFFFu)

uint16 main_control_find_boxes(const openart_map_t *map, main_control_map_pos_t *boxes, uint16 max_boxes);
uint16 main_control_find_goals(const openart_map_t *map, main_control_map_pos_t *goals, uint16 max_goals);
uint8 main_control_get_car_map_pos(const openart_pose_t *pose, const openart_map_t *map, main_control_map_pos_t *car_pos);
uint32 main_control_astar_find_path(const openart_map_t *map,
                                    main_control_map_pos_t start,
                                    main_control_map_pos_t target,
                                    main_control_map_pos_t *path,
                                    uint16 turn_cost);

#endif
