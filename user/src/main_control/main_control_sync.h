#ifndef _MAIN_CONTROL_SYNC_H_
#define _MAIN_CONTROL_SYNC_H_

#include "main_control.h"

typedef struct
{
    uint8 valid;
    uint8 initialized;
    uint8 map_changed;

    uint8 box_moved;
    uint8 box_completed;

    main_control_map_pos_t box_from;
    main_control_map_pos_t box_to;
    main_control_map_pos_t completed_goal;

    uint16 box_count;
    uint16 goal_count;
    uint16 move_count;
    uint16 completed_count;
} main_control_sync_status_t;

void main_control_sync_reset(void);
void main_control_sync_apply_push_result(openart_map_t *map, const main_control_context_t *ctx);
const main_control_sync_status_t *main_control_sync_update(const openart_map_t *map);
const main_control_sync_status_t *main_control_sync_get_status(void);

#endif
