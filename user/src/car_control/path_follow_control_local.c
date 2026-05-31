/*********************************************************************************************************************
* path_follow_control_local.c
*
* Local test version of path following.
* It reuses the real path following logic and then advances the pose structure
* directly so the code can be exercised without car_move_xy().
*********************************************************************************************************************/

#include "path_follow_control_local.h"

// 把本地模拟坐标限制在地图范围内。
static int32 path_follow_local_clamp_int32(int32 value, int32 min_value, int32 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

// 根据路径跟随输出直接修改 pose，模拟小车移动一步。
static void path_follow_local_apply_pose(openart_pose_t *pose,
                                         const openart_map_t *map,
                                         const path_follow_output_t *output)
{
    int32 next_x10;
    int32 next_y10;

    if((0 == pose) || (0 == map) || (0 == output) || (!pose->valid) || (!output->valid) || output->finished)
    {
        return;
    }

    /*
     * Treat the returned x/y as direct per-step deltas in map x10/y10 units.
     * This is a simple offline simulation model, so the result is intentionally coarse.
     */
    next_x10 = (int32)pose->x10 + (int32)output->x;
    next_y10 = (int32)pose->y10 + (int32)output->y;

    /*
     * Avoid overshooting the current target when the simulated step would move
     * beyond it on either axis.
     */
    if((output->x > 0) && (next_x10 > output->target_x10))
    {
        next_x10 = output->target_x10;
    }
    else if((output->x < 0) && (next_x10 < output->target_x10))
    {
        next_x10 = output->target_x10;
    }

    if((output->y > 0) && (next_y10 > output->target_y10))
    {
        next_y10 = output->target_y10;
    }
    else if((output->y < 0) && (next_y10 < output->target_y10))
    {
        next_y10 = output->target_y10;
    }

    if(map->width10 > 0)
    {
        next_x10 = path_follow_local_clamp_int32(next_x10, 0, (int32)map->width10 - 1);
    }
    if(map->height10 > 0)
    {
        next_y10 = path_follow_local_clamp_int32(next_y10, 0, (int32)map->height10 - 1);
    }

    pose->x10 = (int16)next_x10;
    pose->y10 = (int16)next_y10;
    pose->updated = 1;
    pose->seq++;
}

// 本地版路径跟随入口；不调用真实车控接口，只修改 pose。
path_follow_output_t path_follow_update_local(openart_pose_t *pose,
                                              const openart_map_t *map,
                                              main_control_map_pos_t *path,
                                              uint16 *path_count,
                                              int8 x_speed,
                                              int8 y_speed,
                                              uint8 arrive_percent)
{
    path_follow_output_t output;

    output = path_follow_update(pose, map, path, path_count, x_speed, y_speed, arrive_percent);
    path_follow_local_apply_pose(pose, map, &output);

    return output;
}

// 清空本地主控包装函数的输出。
static void main_control_local_clear_output(main_control_local_output_t *output)
{
    if(0 == output)
    {
        return;
    }

    output->control.state = MAIN_CONTROL_STATE_IDLE;
    output->control.valid = 0;
    output->control.plan_ready = 0;
    output->follow.x = 0;
    output->follow.y = 0;
    output->follow.valid = 0;
    output->follow.arrived = 0;
    output->follow.finished = 0;
    output->follow.target.x = 0;
    output->follow.target.y = 0;
    output->follow.target_x10 = 0;
    output->follow.target_y10 = 0;
    output->motion_finished = 0;
}

// 本地测试时直接修改地图，把箱子从起点移动到本段终点。
static uint8 main_control_local_same_pos(main_control_map_pos_t a, main_control_map_pos_t b)
{
    return ((a.x == b.x) && (a.y == b.y));
}

static uint16 main_control_local_map_index(const openart_map_t *map, main_control_map_pos_t pos)
{
    return (uint16)pos.y * map->cols + pos.x;
}

static uint8 main_control_local_map_pos_valid(const openart_map_t *map, main_control_map_pos_t pos)
{
    return ((0 != map) && map->valid && (0 != map->cols) && (0 != map->rows) &&
            (pos.x < map->cols) && (pos.y < map->rows));
}

static void main_control_local_set_box_pos(openart_map_t *map,
                                           main_control_map_pos_t old_pos,
                                           main_control_map_pos_t new_pos,
                                           main_control_map_pos_t goal_pos)
{
    if((!main_control_local_map_pos_valid(map, old_pos)) ||
       (!main_control_local_map_pos_valid(map, new_pos)) ||
       (!main_control_local_map_pos_valid(map, goal_pos)))
    {
        return;
    }

    map->cells[main_control_local_map_index(map, old_pos)] = OPENART_CELL_BACKGROUND;

    if(main_control_local_same_pos(new_pos, goal_pos))
    {
        map->cells[main_control_local_map_index(map, new_pos)] = OPENART_CELL_BACKGROUND;
    }
    else
    {
        map->cells[main_control_local_map_index(map, new_pos)] = OPENART_CELL_YELLOW_BOX;
    }

    map->updated = 1;
    map->seq++;
}

static void main_control_local_sync_box_push(openart_map_t *map,
                                             main_control_context_t *ctx,
                                             const openart_pose_t *pose)
{
    main_control_map_pos_t next_box_pos;
    main_control_map_pos_t car_pos;

    if((0 == ctx) || (!ctx->has_active_plan) || (MAIN_CONTROL_STATE_PUSH_BOX != ctx->state))
    {
        return;
    }

    if(0 == ctx->active_push_car_path_count)
    {
        next_box_pos = ctx->active_box_end;
    }
    else
    {
        next_box_pos = ctx->active_push_car_path[0];
        if(main_control_get_car_map_pos(pose, map, &car_pos) &&
           main_control_local_same_pos(car_pos, ctx->active_push_car_path[0]))
        {
            if(ctx->active_push_car_path_count > 1)
            {
                next_box_pos = ctx->active_push_car_path[1];
            }
            else
            {
                next_box_pos = ctx->active_box_end;
            }
        }
    }

    if(!main_control_local_same_pos(ctx->active_box_current, next_box_pos))
    {
        main_control_local_set_box_pos(map,
                                       ctx->active_box_current,
                                       next_box_pos,
                                       ctx->active_goal);
        ctx->active_box_current = next_box_pos;
    }
}

// 本地主控入口；调用纯主控状态机，并用本地路径跟随模拟执行。
main_control_local_output_t main_control_update_local(main_control_context_t *ctx,
                                                      openart_pose_t *pose,
                                                      openart_map_t *map)
{
    main_control_local_output_t output;

    main_control_local_clear_output(&output);
    output.control = main_control_update(ctx, pose, map);

    if((0 == ctx) || (0 == pose) || (0 == map) || (!output.control.valid))
    {
        return output;
    }

    if(MAIN_CONTROL_STATE_MOVE_TO_PUSH_POS == ctx->state)
    {
        output.follow = path_follow_update_local(pose,
                                                 map,
                                                 ctx->active_car_path,
                                                 &ctx->active_car_path_count,
                                                 MAIN_CONTROL_LOCAL_X_SPEED,
                                                 MAIN_CONTROL_LOCAL_Y_SPEED,
                                                 MAIN_CONTROL_LOCAL_ARRIVE_PERCENT);
        if(output.follow.valid && output.follow.finished)
        {
            main_control_finish_move_to_push_pos(ctx);
            output.motion_finished = 1;
        }
        else if(!output.follow.valid)
        {
            ctx->state = MAIN_CONTROL_STATE_ERROR;
        }
    }
    else if(MAIN_CONTROL_STATE_PUSH_BOX == ctx->state)
    {
        output.follow = path_follow_update_local(pose,
                                                 map,
                                                 ctx->active_push_car_path,
                                                 &ctx->active_push_car_path_count,
                                                 MAIN_CONTROL_LOCAL_X_SPEED,
                                                 MAIN_CONTROL_LOCAL_Y_SPEED,
                                                 MAIN_CONTROL_LOCAL_ARRIVE_PERCENT);
        if(output.follow.valid)
        {
            main_control_local_sync_box_push(map, ctx, pose);
        }
        if(output.follow.valid && output.follow.finished)
        {
            main_control_finish_push_box(ctx);
            output.motion_finished = 1;
        }
        else if(!output.follow.valid)
        {
            ctx->state = MAIN_CONTROL_STATE_ERROR;
        }
    }

    output.control.state = ctx->state;

    return output;
}
