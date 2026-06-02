#include "main_control_find_ids.h"

static uint16 main_control_find_ids_map_index(const openart_map_t *map,
                                              main_control_map_pos_t pos)
{
    return (uint16)pos.y * map->cols + pos.x;
}

static uint8 main_control_find_ids_cell_can_walk(uint8 cell)
{
    return ((OPENART_CELL_BACKGROUND == cell) || (OPENART_CELL_GOAL == cell) ||
            ((OPENART_CELL_GOAL_ID_BASE <= cell) && (cell <= OPENART_CELL_GOAL_ID_MAX)));
}

static uint8 main_control_find_ids_need_goal(uint8 cell)
{
    return (OPENART_CELL_GOAL == cell);
}

static uint8 main_control_find_ids_need_box(uint8 cell)
{
    return (OPENART_CELL_YELLOW_BOX == cell);
}

static uint16 main_control_find_ids_count_path(const main_control_map_pos_t *path,
                                               main_control_map_pos_t target)
{
    uint16 i;

    if(0 == path)
    {
        return 0;
    }

    for(i = 0; i < OPENART_MAP_CELL_MAX; i++)
    {
        if((path[i].x == target.x) && (path[i].y == target.y))
        {
            return (uint16)(i + 1);
        }
    }

    return 0;
}

static int16 main_control_find_ids_face_angle(main_control_map_pos_t stand_pos,
                                              main_control_map_pos_t target)
{
    if(target.x > stand_pos.x)
    {
        return 0;
    }
    if(target.x < stand_pos.x)
    {
        return 180;
    }
    if(target.y < stand_pos.y)
    {
        return 90;
    }

    return 270;
}

static uint8 main_control_find_ids_copy_path(main_control_context_t *ctx,
                                             const main_control_map_pos_t *path,
                                             uint16 count)
{
    uint16 i;

    if((0 == ctx) || (0 == path) || (0 == count) || (count > MAIN_CONTROL_ACTIVE_PATH_MAX))
    {
        return 0;
    }

    for(i = 0; i < count; i++)
    {
        ctx->active_path[i] = path[i];
    }
    ctx->active_path_count = count;

    return 1;
}

static uint8 main_control_find_ids_try_goal_path(main_control_context_t *ctx,
                                                 const openart_map_t *map,
                                                 main_control_map_pos_t car_pos,
                                                 main_control_map_pos_t target,
                                                 uint32 *best_cost)
{
    main_control_map_pos_t path[OPENART_MAP_CELL_MAX];
    uint32 cost;
    uint16 count;

    cost = main_control_astar_find_car_path(map, car_pos, target, path);
    if(MAIN_CONTROL_PATH_COST_INVALID == cost)
    {
        return 0;
    }

    count = main_control_find_ids_count_path(path, target);
    if(count < 2)
    {
        return 0;
    }

    count--;
    if(cost >= *best_cost)
    {
        return 0;
    }

    if(!main_control_find_ids_copy_path(ctx, path, count))
    {
        return 0;
    }

    *best_cost = cost;
    ctx->active_goal = target;
    ctx->target_heading_angle = main_control_find_ids_face_angle(path[count - 1], target);

    return 1;
}

static uint8 main_control_find_ids_try_box_path(main_control_context_t *ctx,
                                                const openart_map_t *map,
                                                main_control_map_pos_t car_pos,
                                                main_control_map_pos_t target,
                                                uint32 *best_cost)
{
    static const int16 dx[4] = {0, 1, 0, -1};
    static const int16 dy[4] = {-1, 0, 1, 0};

    main_control_map_pos_t path[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t stand_pos;
    uint32 cost;
    uint16 count;
    uint16 index;
    uint8 i;
    int16 stand_x;
    int16 stand_y;

    for(i = 0; i < 4; i++)
    {
        stand_x = (int16)target.x + dx[i];
        stand_y = (int16)target.y + dy[i];
        if((stand_x < 0) || (stand_y < 0) ||
           (stand_x >= map->cols) || (stand_y >= map->rows))
        {
            continue;
        }

        stand_pos.x = (uint8)stand_x;
        stand_pos.y = (uint8)stand_y;
        index = main_control_find_ids_map_index(map, stand_pos);
        if(!main_control_find_ids_cell_can_walk(map->cells[index]))
        {
            continue;
        }

        cost = main_control_astar_find_car_path(map, car_pos, stand_pos, path);
        if(cost >= *best_cost)
        {
            continue;
        }

        count = main_control_find_ids_count_path(path, stand_pos);
        if(0 == count)
        {
            continue;
        }

        if(!main_control_find_ids_copy_path(ctx, path, count))
        {
            continue;
        }

        *best_cost = cost;
        ctx->active_goal = target;
        ctx->active_box_current = target;
        ctx->target_heading_angle = main_control_find_ids_face_angle(stand_pos, target);
    }

    return (MAIN_CONTROL_PATH_COST_INVALID != *best_cost);
}

static uint8 main_control_find_ids_build_task(main_control_context_t *ctx,
                                              const openart_pose_t *pose,
                                              const openart_map_t *map,
                                              const main_control_map_pos_t *targets,
                                              uint16 target_count,
                                              uint8 find_goal)
{
    main_control_map_pos_t car_pos;
    uint32 best_cost;
    uint16 i;
    uint16 index;
    uint8 cell;
    uint8 found;

    if((0 == ctx) || (0 == pose) || (0 == map) || (0 == targets) ||
       (!main_control_get_car_map_pos(pose, map, &car_pos)))
    {
        return 0;
    }

    found = 0;
    best_cost = MAIN_CONTROL_PATH_COST_INVALID;
    ctx->active_path_count = 0;

    for(i = 0; i < target_count; i++)
    {
        if((targets[i].x >= map->cols) || (targets[i].y >= map->rows))
        {
            continue;
        }

        index = main_control_find_ids_map_index(map, targets[i]);
        cell = map->cells[index];
        if(find_goal)
        {
            if(main_control_find_ids_need_goal(cell))
            {
                found |= main_control_find_ids_try_goal_path(ctx, map, car_pos, targets[i], &best_cost);
            }
        }
        else
        {
            if(main_control_find_ids_need_box(cell))
            {
                found |= main_control_find_ids_try_box_path(ctx, map, car_pos, targets[i], &best_cost);
            }
        }
    }

    return found;
}

uint8 main_control_find_ids_main(main_control_context_t *ctx,
                                 const openart_pose_t *pose,
                                 const openart_map_t *map)
{
    if((0 == ctx) || (0 == pose) || (0 == map))
    {
        return 0;
    }

    ctx->box_count = main_control_find_boxes(map, ctx->boxes, OPENART_MAP_CELL_MAX);
    ctx->goal_count = main_control_find_goals(map, ctx->goals, OPENART_MAP_CELL_MAX);

    if(main_control_find_ids_build_task(ctx, pose, map, ctx->goals, ctx->goal_count, 1) ||
       main_control_find_ids_build_task(ctx, pose, map, ctx->boxes, ctx->box_count, 0))
    {
        ctx->has_active_plan = 0;
        main_control_add_task(ctx, MAIN_CONTROL_STATE_RUN_PATH);
        main_control_add_task(ctx, MAIN_CONTROL_STATE_TURN);
        ctx->wait_ms = 500;
        main_control_add_task(ctx, MAIN_CONTROL_STATE_WAIT);
        main_control_add_task(ctx, MAIN_CONTROL_STATE_SCAN_ID);
        main_control_add_task(ctx, MAIN_CONTROL_STATE_FIND_IDS);
        main_control_shift_task(ctx);
        return 0;
    }

    return 1;
}
