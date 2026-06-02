#include "main_control_plan.h"

static void main_control_copy_path(main_control_map_pos_t *dst,
                                   const main_control_map_pos_t *src,
                                   uint16 count)
{
    uint16 i;

    if((0 == dst) || (0 == src))
    {
        return;
    }

    if(count > OPENART_MAP_CELL_MAX)
    {
        count = OPENART_MAP_CELL_MAX;
    }

    for(i = 0; i < count; i++)
    {
        dst[i] = src[i];
    }
}

static uint16 main_control_count_path_to_target(const main_control_map_pos_t *path,
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

static uint8 main_control_calc_push_pos(const main_control_map_pos_t *box_path,
                                        main_control_map_pos_t *push_pos)
{
    int16 dx;
    int16 dy;
    int16 push_x;
    int16 push_y;

    if((0 == box_path) || (0 == push_pos))
    {
        return 0;
    }

    dx = (int16)box_path[1].x - (int16)box_path[0].x;
    dy = (int16)box_path[1].y - (int16)box_path[0].y;
    push_x = (int16)box_path[0].x - dx;
    push_y = (int16)box_path[0].y - dy;

    if((push_x < 0) || (push_y < 0) || (push_x > 255) || (push_y > 255))
    {
        return 0;
    }

    push_pos->x = (uint8)push_x;
    push_pos->y = (uint8)push_y;

    return 1;
}

static uint16 main_control_find_push_segment_end(const main_control_map_pos_t *box_path,
                                                 uint16 box_path_count)
{
    int16 first_dx;
    int16 first_dy;
    int16 dx;
    int16 dy;
    uint16 i;

    if((0 == box_path) || (box_path_count < 2))
    {
        return 0;
    }

    first_dx = (int16)box_path[1].x - (int16)box_path[0].x;
    first_dy = (int16)box_path[1].y - (int16)box_path[0].y;

    for(i = 2; i < box_path_count; i++)
    {
        dx = (int16)box_path[i].x - (int16)box_path[i - 1].x;
        dy = (int16)box_path[i].y - (int16)box_path[i - 1].y;
        if((dx != first_dx) || (dy != first_dy))
        {
            return (uint16)(i - 1);
        }
    }

    return (uint16)(box_path_count - 1);
}

static uint8 main_control_fill_push_car_path(const main_control_map_pos_t *box_path,
                                             uint16 box_path_count,
                                             main_control_map_pos_t *push_car_path,
                                             uint16 *push_car_path_count,
                                             main_control_map_pos_t *box_end)
{
    uint16 i;
    uint16 segment_end;

    if((0 == box_path) || (0 == push_car_path) || (0 == push_car_path_count) ||
       (0 == box_end) || (box_path_count < 2))
    {
        return 0;
    }

    segment_end = main_control_find_push_segment_end(box_path, box_path_count);
    if(0 == segment_end)
    {
        return 0;
    }

    for(i = 0; i < segment_end; i++)
    {
        push_car_path[i] = box_path[i];
    }

    *push_car_path_count = segment_end;
    *box_end = box_path[segment_end];

    return 1;
}

static uint8 main_control_fill_active_path(main_control_context_t *ctx,
                                           const main_control_plan_t *plan)
{
    main_control_map_pos_t push_car_path[OPENART_MAP_CELL_MAX];
    uint16 push_car_path_count;
    uint16 active_count;
    uint16 i;

    if((0 == ctx) || (0 == plan) || (!plan->valid))
    {
        return 0;
    }

    if(!main_control_fill_push_car_path(plan->box_path,
                                        plan->box_path_count,
                                        push_car_path,
                                        &push_car_path_count,
                                        &ctx->active_box_end))
    {
        return 0;
    }

    if((uint32)plan->car_path_count + (uint32)push_car_path_count > MAIN_CONTROL_ACTIVE_PATH_MAX)
    {
        return 0;
    }

    active_count = 0;
    for(i = 0; i < plan->car_path_count; i++)
    {
        ctx->active_path[active_count++] = plan->car_path[i];
    }
    for(i = 0; i < push_car_path_count; i++)
    {
        ctx->active_path[active_count++] = push_car_path[i];
    }

    ctx->active_path_count = active_count;

    return (0 != active_count);
}

static uint8 main_control_fill_return_path(main_control_context_t *ctx,
                                           const openart_pose_t *pose,
                                           const openart_map_t *map)
{
    main_control_map_pos_t car_pos;
    main_control_map_pos_t target;
    uint32 cost;
    uint16 path_count;

    if((0 == ctx) || (0 == pose) || (0 == map) || (map->rows <= 1))
    {
        return 0;
    }

    if(!main_control_get_car_map_pos(pose, map, &car_pos))
    {
        return 0;
    }

    target.x = (uint8)(map->cols / 2);
    target.y = 1;

    cost = main_control_astar_find_car_path(map, car_pos, target, ctx->active_path);
    if(MAIN_CONTROL_PATH_COST_INVALID == cost)
    {
        return 0;
    }

    path_count = main_control_count_path_to_target(ctx->active_path, target);
    if(0 == path_count)
    {
        return 0;
    }

    ctx->active_path_count = path_count;
    ctx->has_active_plan = 0;

    return 1;
}

static uint8 main_control_build_box_plan(main_control_plan_t *plan,
                                         const openart_map_t *map,
                                         main_control_map_pos_t car_pos,
                                         main_control_map_pos_t box_pos,
                                         const main_control_map_pos_t *goals,
                                         uint16 goal_count)
{
    main_control_map_pos_t candidate_path[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t best_goal;
    uint32 best_box_cost;
    uint32 car_cost;
    uint32 cost;
    uint16 best_box_path_count;
    uint16 car_path_count;
    uint16 i;

    if((0 == plan) || (0 == map) || (0 == goals) || (0 == goal_count))
    {
        return 0;
    }

    plan->valid = 0;
    plan->box_pos = box_pos;
    best_box_cost = MAIN_CONTROL_PATH_COST_INVALID;
    best_box_path_count = 0;

    for(i = 0; i < goal_count; i++)
    {
        cost = main_control_astar_find_path(map, box_pos, goals[i], candidate_path, MAIN_CONTROL_BOX_PATH_TURN_COST);
        if(cost < best_box_cost)
        {
            best_box_path_count = main_control_count_path_to_target(candidate_path, goals[i]);
            if(best_box_path_count >= 2)
            {
                best_box_cost = cost;
                best_goal = goals[i];
                main_control_copy_path(plan->box_path, candidate_path, best_box_path_count);
            }
        }
    }

    if((MAIN_CONTROL_PATH_COST_INVALID == best_box_cost) || (best_box_path_count < 2))
    {
        return 0;
    }

    plan->goal_pos = best_goal;
    plan->box_path_count = best_box_path_count;
    plan->box_path_cost = best_box_cost;

    if(!main_control_calc_push_pos(plan->box_path, &plan->push_pos))
    {
        return 0;
    }

    car_cost = main_control_astar_find_car_path(map, car_pos, plan->push_pos, plan->car_path);
    if(MAIN_CONTROL_PATH_COST_INVALID == car_cost)
    {
        return 0;
    }

    car_path_count = main_control_count_path_to_target(plan->car_path, plan->push_pos);
    if(0 == car_path_count)
    {
        return 0;
    }

    plan->car_path_count = car_path_count;
    plan->car_path_cost = car_cost;
    plan->total_cost = car_cost + best_box_cost;
    plan->valid = 1;

    return 1;
}

uint8 main_control_build_best_plan(main_control_context_t *ctx,
                                   const openart_pose_t *pose,
                                   const openart_map_t *map)
{
    main_control_map_pos_t car_pos;
    uint32 best_cost;
    uint16 i;
    uint16 plan_count;

    if((0 == ctx) || (0 == pose) || (0 == map))
    {
        return 0;
    }

    ctx->has_active_plan = 0;
    ctx->box_count = main_control_find_boxes(map, ctx->boxes, OPENART_MAP_CELL_MAX);
    ctx->goal_count = main_control_find_goals(map, ctx->goals, OPENART_MAP_CELL_MAX);
    ctx->plan_count = 0;
    ctx->best_plan_index = 0;
    ctx->active_path_count = 0;

    if((0 == ctx->box_count) || (0 == ctx->goal_count))
    {
        if(main_control_fill_return_path(ctx, pose, map))
        {
            ctx->state = MAIN_CONTROL_STATE_RUN_PATH;
        }
        else
        {
            ctx->state = MAIN_CONTROL_STATE_FINISHED;
        }
        return 0;
    }

    if(!main_control_get_car_map_pos(pose, map, &car_pos))
    {
        ctx->state = MAIN_CONTROL_STATE_ERROR;
        return 0;
    }

    plan_count = ctx->box_count;
    if(plan_count > MAIN_CONTROL_PLAN_MAX)
    {
        plan_count = MAIN_CONTROL_PLAN_MAX;
    }

    best_cost = MAIN_CONTROL_PATH_COST_INVALID;
    for(i = 0; i < plan_count; i++)
    {
        ctx->plans[i].valid = 0;
        if(main_control_build_box_plan(&ctx->plans[i], map, car_pos, ctx->boxes[i], ctx->goals, ctx->goal_count))
        {
            ctx->plan_count++;
            if(ctx->plans[i].total_cost < best_cost)
            {
                best_cost = ctx->plans[i].total_cost;
                ctx->best_plan_index = i;
            }
        }
    }

    if(MAIN_CONTROL_PATH_COST_INVALID == best_cost)
    {
        ctx->state = MAIN_CONTROL_STATE_ERROR;
        return 0;
    }

    if(!main_control_fill_active_path(ctx, &ctx->plans[ctx->best_plan_index]))
    {
        ctx->state = MAIN_CONTROL_STATE_ERROR;
        return 0;
    }

    ctx->active_box_start = ctx->plans[ctx->best_plan_index].box_pos;
    ctx->active_box_current = ctx->active_box_start;
    ctx->active_goal = ctx->plans[ctx->best_plan_index].goal_pos;
    ctx->has_active_plan = 1;
    ctx->replan_count++;

    return 1;
}
