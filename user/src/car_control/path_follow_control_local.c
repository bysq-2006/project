/*********************************************************************************************************************
* path_follow_control_local.c
*
* Local test version of path following.
* It reuses the real path following logic and then advances the pose structure
* directly so the code can be exercised without car_move_xy().
*********************************************************************************************************************/

#include "path_follow_control_local.h"
#include <math.h>

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

static void path_follow_local_apply_pose(openart_pose_t *pose,
                                         const openart_map_t *map,
                                         const path_follow_output_t *output)
{
    int32 next_x10;
    int32 next_y10;
    int16 map_dx;
    int16 map_dy;
    float angle_rad;
    float sin_angle;
    float cos_angle;

    if((0 == pose) || (0 == map) || (0 == output) || (!pose->valid) || (!output->valid) || output->finished)
    {
        return;
    }

    angle_rad = (float)pose->angle10 * PATH_FOLLOW_ANGLE10_TO_RAD;
    sin_angle = sinf(angle_rad);
    cos_angle = cosf(angle_rad);
    map_dx = (int16)((float)output->x * sin_angle + (float)output->y * cos_angle);
    map_dy = (int16)((float)output->x * cos_angle - (float)output->y * sin_angle);

    next_x10 = (int32)pose->x10 + (int32)map_dx;
    next_y10 = (int32)pose->y10 + (int32)map_dy;

    /*
     * Avoid overshooting the current target when the simulated step would move
     * beyond it on either axis.
     */
    if((map_dx > 0) && (next_x10 > output->target_x10))
    {
        next_x10 = output->target_x10;
    }
    else if((map_dx < 0) && (next_x10 < output->target_x10))
    {
        next_x10 = output->target_x10;
    }

    if((map_dy > 0) && (next_y10 > output->target_y10))
    {
        next_y10 = output->target_y10;
    }
    else if((map_dy < 0) && (next_y10 < output->target_y10))
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

// 本地仿真里的转向输入：直接更新车头角度。
static void path_follow_local_apply_heading(openart_pose_t *pose, int8 w)
{
    int32 angle10;

    if((0 == pose) || (!pose->valid))
    {
        return;
    }

    angle10 = (int32)pose->angle10 + (int32)w * 10;
    while(angle10 < 0)
    {
        angle10 += 3600;
    }
    while(angle10 >= 3600)
    {
        angle10 -= 3600;
    }

    pose->angle10 = (uint16)angle10;
    pose->updated = 1;
    pose->seq++;
}

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

    if(MAIN_CONTROL_STATE_RUN_PATH == ctx->state[0])
    {
        output.follow = path_follow_update_local(pose,
                                                 map,
                                                 ctx->active_path,
                                                 &ctx->active_path_count,
                                                 MAIN_CONTROL_LOCAL_X_SPEED,
                                                 MAIN_CONTROL_LOCAL_Y_SPEED,
                                                 MAIN_CONTROL_LOCAL_ARRIVE_PERCENT);
        if(output.follow.valid)
        {
            path_follow_local_apply_heading(pose, (int8)ctx->target_heading_angle);
        }
        if(output.follow.valid && output.follow.finished)
        {
            main_control_finish_path(ctx);
            output.motion_finished = 1;
        }
        else if(!output.follow.valid)
        {
            main_control_add_task(ctx, MAIN_CONTROL_STATE_ERROR);
            main_control_shift_task(ctx);
        }
    }

    output.control.state = ctx->state[0];

    return output;
}
