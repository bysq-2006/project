/*********************************************************************************************************************
* path_follow_control.c
*********************************************************************************************************************/

#include "path_follow_control.h"

static int16 path_follow_abs_int16(int16 value)
{
    return (value >= 0) ? value : -value;
}

static int8 path_follow_abs_int8(int8 value)
{
    return (value >= 0) ? value : -value;
}

static int8 path_follow_apply_sign(int16 diff, int8 value)
{
    if(diff < 0)
    {
        return (int8)-value;
    }

    return value;
}

static int8 path_follow_limit_speed(int32 value, int8 max_speed)
{
    int8 abs_max;

    abs_max = path_follow_abs_int8(max_speed);
    if(value > abs_max)
    {
        return abs_max;
    }
    if(value < -abs_max)
    {
        return (int8)-abs_max;
    }

    return (int8)value;
}

static void path_follow_shift_path(main_control_map_pos_t *path, uint16 *path_count)
{
    uint16 i;

    if((0 == path) || (0 == path_count) || (0 == *path_count))
    {
        return;
    }

    // 到达当前目标点后，把后面的路径点整体向前挪一格，新的 path[0] 就是下一个目标。
    for(i = 1; i < *path_count; i++)
    {
        path[i - 1] = path[i];
    }

    (*path_count)--;
}

static uint8 path_follow_grid_to_abs(const openart_map_t *map,
                                     main_control_map_pos_t pos,
                                     int16 *x10,
                                     int16 *y10)
{
    int32 target_x10;
    int32 target_y10;

    if((0 == map) || (0 == x10) || (0 == y10) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows) ||
       (0 == map->width10) || (0 == map->height10) ||
       (pos.x >= map->cols) || (pos.y >= map->rows))
    {
        return 0;
    }

    // 把格子坐标换成绝对坐标。这里取格子中心点：
    // x = (列号 + 0.5) * 地图宽度 / 列数，y = (行号 + 0.5) * 地图高度 / 行数。
    target_x10 = ((int32)pos.x * 2 + 1) * map->width10 / ((int32)map->cols * 2);
    target_y10 = ((int32)pos.y * 2 + 1) * map->height10 / ((int32)map->rows * 2);

    *x10 = (int16)target_x10;
    *y10 = (int16)target_y10;

    return 1;
}

static uint8 path_follow_is_arrived(const openart_pose_t *pose,
                                    const openart_map_t *map,
                                    int16 target_x10,
                                    int16 target_y10,
                                    uint8 arrive_percent)
{
    int16 dx;
    int16 dy;
    int32 threshold_x10;
    int32 threshold_y10;

    if((0 == pose) || (0 == map) || (!pose->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    dx = (int16)(pose->x10 - target_x10);
    dy = (int16)(pose->y10 - target_y10);

    // 到达范围按“单个格子的百分比”计算。arrive_percent 越大，越早切换到下一个路径点。
    threshold_x10 = ((int32)map->width10 * arrive_percent) / ((int32)map->cols * 100);
    threshold_y10 = ((int32)map->height10 * arrive_percent) / ((int32)map->rows * 100);

    return ((path_follow_abs_int16(dx) <= threshold_x10) &&
            (path_follow_abs_int16(dy) <= threshold_y10));
}

static void path_follow_calc_speed(int16 dx,
                                   int16 dy,
                                   int8 x_speed,
                                   int8 y_speed,
                                   path_follow_output_t *output)
{
    int16 abs_dx;
    int16 abs_dy;
    int8 max_x;
    int8 max_y;
    int32 x_value;
    int32 y_value;

    output->x = 0;
    output->y = 0;

    abs_dx = path_follow_abs_int16(dx);
    abs_dy = path_follow_abs_int16(dy);
    max_x = path_follow_abs_int8(x_speed);
    max_y = path_follow_abs_int8(y_speed);

    if((0 == abs_dx) && (0 == abs_dy))
    {
        return;
    }
    if((0 == max_x) && (0 == max_y))
    {
        return;
    }

    if(0 == abs_dx)
    {
        output->y = path_follow_apply_sign(dy, max_y);
        return;
    }
    if(0 == abs_dy)
    {
        output->x = path_follow_apply_sign(dx, max_x);
        return;
    }
    if(0 == max_x)
    {
        output->y = path_follow_apply_sign(dy, max_y);
        return;
    }
    if(0 == max_y)
    {
        output->x = path_follow_apply_sign(dx, max_x);
        return;
    }

    if(((int32)abs_dx * max_y) > ((int32)abs_dy * max_x))
    {
        // x 方向差距更大时，让 x 轴跑满给定速度，y 轴按直线方向比例缩小。
        x_value = max_x;
        y_value = ((int32)abs_dy * max_x) / abs_dx;
        if(0 == y_value)
        {
            y_value = 1;
        }
    }
    else
    {
        // y 方向差距更大时，让 y 轴跑满给定速度，x 轴按直线方向比例缩小。
        y_value = max_y;
        x_value = ((int32)abs_dx * max_y) / abs_dy;
        if(0 == x_value)
        {
            x_value = 1;
        }
    }

    output->x = path_follow_apply_sign(dx, path_follow_limit_speed(x_value, max_x));
    output->y = path_follow_apply_sign(dy, path_follow_limit_speed(y_value, max_y));
}

path_follow_output_t path_follow_update(const openart_pose_t *pose,
                                        const openart_map_t *map,
                                        main_control_map_pos_t *path,
                                        uint16 *path_count,
                                        int8 x_speed,
                                        int8 y_speed,
                                        uint8 arrive_percent)
{
    path_follow_output_t output;
    int16 target_x10;
    int16 target_y10;
    int16 dx;
    int16 dy;

    output.x = 0;
    output.y = 0;
    output.valid = 0;
    output.arrived = 0;
    output.finished = 0;
    output.target.x = 0;
    output.target.y = 0;
    output.target_x10 = 0;
    output.target_y10 = 0;

    if((0 == pose) || (0 == map) || (0 == path) || (0 == path_count) ||
       (!pose->valid) || (!map->valid))
    {
        return output;
    }

    if(0 == *path_count)
    {
        output.valid = 1;
        output.finished = 1;
        return output;
    }

    // 先连续跳过已经到达的路径点。这样 path[0] 是车当前位置时，不需要多等一次调用。
    while(0 != *path_count)
    {
        if(!path_follow_grid_to_abs(map, path[0], &target_x10, &target_y10))
        {
            return output;
        }

        if(!path_follow_is_arrived(pose, map, target_x10, target_y10, arrive_percent))
        {
            break;
        }

        output.arrived = 1;
        path_follow_shift_path(path, path_count);
    }

    // 如果刚刚移除的是最后一个路径点，说明整条路径已经走完。
    if(0 == *path_count)
    {
        output.valid = 1;
        output.finished = 1;
        return output;
    }

    output.valid = 1;
    output.target = path[0];
    output.target_x10 = target_x10;
    output.target_y10 = target_y10;

    // 根据当前车位置到目标点中心的直线方向，计算 x/y 两个方向应该给的速度。
    dx = (int16)(target_x10 - pose->x10);
    dy = (int16)(target_y10 - pose->y10);
    path_follow_calc_speed(dx, dy, x_speed, y_speed, &output);

    return output;
}
