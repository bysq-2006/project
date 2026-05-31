#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "zf_common_headfile.h"
#include "../openart_uart/openart_uart.h"

typedef struct
{
    uint8 x;
    uint8 y;
} main_control_map_pos_t;

uint16 main_control_find_boxes(const openart_map_t *map, main_control_map_pos_t *boxes, uint16 max_boxes);
uint16 main_control_find_goals(const openart_map_t *map, main_control_map_pos_t *goals, uint16 max_goals);

#endif
