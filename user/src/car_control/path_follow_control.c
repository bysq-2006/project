/*********************************************************************************************************************
* path_follow_control.c
*********************************************************************************************************************/

#include "path_follow_control.h"
#include <math.h>

#define PATH_FOLLOW_PID_X_P                 (0.2f)
#define PATH_FOLLOW_PID_X_I                 (0.001f)
#define PATH_FOLLOW_PID_X_D                 (1.2f)

#define PATH_FOLLOW_PID_Y_P                 (0.15f)
#define PATH_FOLLOW_PID_Y_I                 (0.001f)
#define PATH_FOLLOW_PID_Y_D                 (1.2f)
#define PATH_FOLLOW_PID_I_LIMIT             (1000.0f)

typedef struct
{
    float error_sum;
    float last_error;
} path_follow_pid_axis_t;

static path_follow_pid_axis_t path_follow_pid_x = {0.0f, 0.0f};
static path_follow_pid_axis_t path_follow_pid_y = {0.0f, 0.0f};
static uint8 path_follow_pid_has_target = 0;
static int16 path_follow_pid_target_x10 = 0;
static int16 path_follow_pid_target_y10 = 0;

static int16 path_follow_abs_int16(int16 value)
{
    return (value >= 0) ? value : -value;
}

static int8 path_follow_abs_int8(int8 value)
{
    return (value >= 0) ? value : -value;
}

static float path_follow_limit_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }

    if(value < min_value)
    {
        return min_value;
    }

    return value;
}

static int8 path_follow_float_to_int8(float value)
{
    if(value > 0.0f)
    {
        return (int8)(value + 0.5f);
    }

    if(value < 0.0f)
    {
        return (int8)(value - 0.5f);
    }

    return 0;
}

static void path_follow_map_diff_to_car_diff(int16 map_dx,
                                             int16 map_dy,
                                             uint16 angle10,
                                             float *car_dx,
                                             float *car_dy)
{
    float angle_rad;
    float sin_angle;
    float cos_angle;

    angle_rad = (float)angle10 * PATH_FOLLOW_ANGLE10_TO_RAD;
    sin_angle = sinf(angle_rad);
    cos_angle = cosf(angle_rad);

    *car_dx = (float)map_dx * sin_angle + (float)map_dy * cos_angle;
    *car_dy = (float)map_dx * cos_angle - (float)map_dy * sin_angle;
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


static void path_follow_reset_pid(float x_error, float y_error)
{
    path_follow_pid_x.error_sum = 0.0f;
    path_follow_pid_x.last_error = x_error;
    path_follow_pid_y.error_sum = 0.0f;
    path_follow_pid_y.last_error = y_error;
}

static void path_follow_prepare_pid(int16 target_x10,
                                    int16 target_y10,
                                    float x_error,
                                    float y_error)
{
    if((!path_follow_pid_has_target) ||
       (path_follow_pid_target_x10 != target_x10) ||
       (path_follow_pid_target_y10 != target_y10))
    {
        path_follow_reset_pid(x_error, y_error);
        path_follow_pid_has_target = 1;
        path_follow_pid_target_x10 = target_x10;
        path_follow_pid_target_y10 = target_y10;
    }
}

static int8 path_follow_pid_calc_axis(path_follow_pid_axis_t *pid,
                                      float error,
                                      float p,
                                      float i,
                                      float d,
                                      int8 speed_limit)
{
    float output;
    float limit;

    limit = (float)path_follow_abs_int8(speed_limit);
    if(limit <= 0.0f)
    {
        pid->last_error = error;
        return 0;
    }

    pid->error_sum += error;
    pid->error_sum = path_follow_limit_float(pid->error_sum,
                                             -PATH_FOLLOW_PID_I_LIMIT,
                                             PATH_FOLLOW_PID_I_LIMIT);

    output = error * p
           + pid->error_sum * i
           + (error - pid->last_error) * d;
    pid->last_error = error;
    output = path_follow_limit_float(output, -limit, limit);

    return path_follow_float_to_int8(output);
}

static void path_follow_calc_speed(float dx,
                                   float dy,
                                   int8 x_speed,
                                   int8 y_speed,
                                   path_follow_output_t *output)
{
    path_follow_prepare_pid(output->target_x10,
                            output->target_y10,
                            dx,
                            dy);

    output->x = path_follow_pid_calc_axis(&path_follow_pid_x,
                                          dx,
                                          PATH_FOLLOW_PID_X_P,
                                          PATH_FOLLOW_PID_X_I,
                                          PATH_FOLLOW_PID_X_D,
                                          x_speed);
    output->y = path_follow_pid_calc_axis(&path_follow_pid_y,
                                          dy,
                                          PATH_FOLLOW_PID_Y_P,
                                          PATH_FOLLOW_PID_Y_I,
                                          PATH_FOLLOW_PID_Y_D,
                                          y_speed);
}

path_follow_output_t path_follow_update(const openart_pose_t *pose,
                                        const openart_map_t *map,
                                        main_control_map_pos_t *path,
                                        uint16 *path_count,
                                        int8 x_speed,
                                        int8 y_speed,
                                        uint8 arrive_percent)
{
    path_follow_output_t output = {0};
    int16 target_x10;
    int16 target_y10;
    int16 dx;
    int16 dy;
    float car_dx;
    float car_dy;

    if((0 == pose) || (0 == map) || (0 == path) || (0 == path_count) ||
       (!pose->valid) || (!map->valid))
    {
        path_follow_pid_has_target = 0;
        path_follow_reset_pid(0.0f, 0.0f);
        return output;
    }

    if(0 == *path_count)
    {
        path_follow_pid_has_target = 0;
        path_follow_reset_pid(0.0f, 0.0f);
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
        path_follow_pid_has_target = 0;
        path_follow_reset_pid(0.0f, 0.0f);
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
    path_follow_map_diff_to_car_diff(dx, dy, pose->angle10, &car_dx, &car_dy);
    path_follow_calc_speed(car_dx, car_dy, x_speed, y_speed, &output);

    return output;
}
