#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "map_planner.h"

#define MAIN_CONTROL_PLAN_MAX          (OPENART_MAP_CELL_MAX)
#define MAIN_CONTROL_BOX_PATH_TURN_COST (10U)

typedef enum
{
    MAIN_CONTROL_STATE_IDLE = 0,
    MAIN_CONTROL_STATE_PLAN,
    MAIN_CONTROL_STATE_MOVE_TO_PUSH_POS,
    MAIN_CONTROL_STATE_PUSH_BOX,
    MAIN_CONTROL_STATE_FINISHED,
    MAIN_CONTROL_STATE_ERROR
} main_control_state_t;

typedef struct
{
    main_control_map_pos_t box_pos;
    main_control_map_pos_t goal_pos;

    main_control_map_pos_t box_path[OPENART_MAP_CELL_MAX];
    uint16 box_path_count;
    uint32 box_path_cost;

    main_control_map_pos_t push_pos;
    main_control_map_pos_t car_path[OPENART_MAP_CELL_MAX];
    uint16 car_path_count;
    uint32 car_path_cost;

    uint32 total_cost;
    uint8 valid;
} main_control_plan_t;

typedef struct
{
    main_control_state_t state;

    main_control_map_pos_t boxes[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t goals[OPENART_MAP_CELL_MAX];
    uint16 box_count;
    uint16 goal_count;

    main_control_plan_t plans[MAIN_CONTROL_PLAN_MAX];
    uint16 plan_count;
    uint16 best_plan_index;

    main_control_map_pos_t active_car_path[OPENART_MAP_CELL_MAX];
    uint16 active_car_path_count;

    main_control_map_pos_t active_push_car_path[OPENART_MAP_CELL_MAX];
    uint16 active_push_car_path_count;

    main_control_map_pos_t active_box_start;
    main_control_map_pos_t active_box_end;
    main_control_map_pos_t active_goal;

    uint32 replan_count;
    uint8 has_active_plan;
} main_control_context_t;

typedef struct
{
    main_control_state_t state;
    uint8 valid;
    uint8 plan_ready;
} main_control_output_t;

void main_control_init(main_control_context_t *ctx);
main_control_output_t main_control_update(main_control_context_t *ctx,
                                          openart_pose_t *pose,
                                          openart_map_t *map);
void main_control_finish_move_to_push_pos(main_control_context_t *ctx);
void main_control_finish_push_box(main_control_context_t *ctx);

#endif
