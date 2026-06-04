/*********************************************************************************************************************
* path_follow_control.c
*********************************************************************************************************************/

#include "path_follow_control.h"

#define PATH_FOLLOW_PID_X_P                 (0.18f)
#define PATH_FOLLOW_PID_X_I                 (0.0010f)
#define PATH_FOLLOW_PID_X_D                 (0.1f)
#define PATH_FOLLOW_PID_Y_P                 (0.12f)
#define PATH_FOLLOW_PID_Y_I                 (0.0010f)
#define PATH_FOLLOW_PID_Y_D                 (0.1f)
#define PATH_FOLLOW_PID_I_LIMIT             (1000.0f)

/*
 * 脉冲式运动参数：
 * path_follow_update 每调用一次，计数 +1。
 * 例如主循环 20ms 调一次：MOVE_CALLS=4 约等于动 80ms，STOP_CALLS=6 约等于停 120ms。
 * 跑过头就减小 MOVE_CALLS 或增大 STOP_CALLS。
 * 不动/走太慢就增大 MOVE_CALLS 或减小 STOP_CALLS。
 */
#define PATH_FOLLOW_PULSE_MOVE_CALLS        (2)
#define PATH_FOLLOW_PULSE_STOP_CALLS        (1)

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

static uint8 path_follow_pulse_has_target = 0;
static int16 path_follow_pulse_target_x10 = 0;
static int16 path_follow_pulse_target_y10 = 0;
static uint8 path_follow_pulse_is_moving = 1;
static uint16 path_follow_pulse_counter = 0;

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

static void path_follow_reset_pulse(void)
{
    path_follow_pulse_has_target = 0;
    path_follow_pulse_target_x10 = 0;
    path_follow_pulse_target_y10 = 0;
    path_follow_pulse_is_moving = 1;
    path_follow_pulse_counter = 0;
}

static void path_follow_prepare_pulse(int16 target_x10, int16 target_y10)
{
    if((!path_follow_pulse_has_target) ||
       (path_follow_pulse_target_x10 != target_x10) ||
       (path_follow_pulse_target_y10 != target_y10))
    {
        path_follow_pulse_has_target = 1;
        path_follow_pulse_target_x10 = target_x10;
        path_follow_pulse_target_y10 = target_y10;
        path_follow_pulse_is_moving = 1;
        path_follow_pulse_counter = 0;
    }
}

static uint8 path_follow_pulse_allow_move(void)
{
    if(PATH_FOLLOW_PULSE_MOVE_CALLS <= 0)
    {
        return 0;
    }

    if(PATH_FOLLOW_PULSE_STOP_CALLS <= 0)
    {
        return 1;
    }

    if(path_follow_pulse_is_moving)
    {
        path_follow_pulse_counter++;
        if(path_follow_pulse_counter >= PATH_FOLLOW_PULSE_MOVE_CALLS)
        {
            path_follow_pulse_is_moving = 0;
            path_follow_pulse_counter = 0;
        }
        return 1;
    }

    path_follow_pulse_counter++;
    if(path_follow_pulse_counter >= PATH_FOLLOW_PULSE_STOP_CALLS)
    {
        path_follow_pulse_is_moving = 1;
        path_follow_pulse_counter = 0;
    }

    return 0;
}

static void path_follow_map_diff_to_car_diff(int16 map_dx,
                                             int16 map_dy,
                                             uint16 angle10,
                                             int16 *car_dx,
                                             int16 *car_dy)
{
    uint16 angle_norm;
    uint8 direction;

    /*
     * 这里不再使用连续角度 sin/cos。
     * 先把车头角度强制吸附到最近的四个方向：
     *   0:   0 度附近
     *   1:  90 度附近
     *   2: 180 度附近
     *   3: 270 度附近
     *
     * angle10 单位是 0.1 度，所以 90 度 = 900。
     * +450 表示四舍五入到最近的 90 度，而不是直接向下取整。
     */
    angle_norm = angle10 % 3600;
    direction = (uint8)(((angle_norm + 450) / 900) % 4);

    switch(direction)
    {
        case 0:
            *car_dx = map_dy;
            *car_dy = map_dx;
            break;

        case 1:
            *car_dx = map_dx;
            *car_dy = (int16)(-map_dy);
            break;

        case 2:
            *car_dx = (int16)(-map_dy);
            *car_dy = (int16)(-map_dx);
            break;

        default:
            *car_dx = (int16)(-map_dx);
            *car_dy = map_dy;
            break;
    }
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

    return (int8)output;
}

static void path_follow_calc_speed(int16 dx,
                                   int16 dy,
                                   int8 x_speed,
                                   int8 y_speed,
                                   path_follow_output_t *output)
{
    path_follow_prepare_pid(output->target_x10,
                            output->target_y10,
                            (float)dx,
                            (float)dy);

    output->x = path_follow_pid_calc_axis(&path_follow_pid_x,
                                          (float)dx,
                                          PATH_FOLLOW_PID_X_P,
                                          PATH_FOLLOW_PID_X_I,
                                          PATH_FOLLOW_PID_X_D,
                                          x_speed);
    output->y = path_follow_pid_calc_axis(&path_follow_pid_y,
                                          (float)dy,
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
    int16 car_dx;
    int16 car_dy;

    if((0 == pose) || (0 == map) || (0 == path) || (0 == path_count) ||
       (!pose->valid) || (!map->valid))
    {
        path_follow_pid_has_target = 0;
        path_follow_reset_pid(0.0f, 0.0f);
        path_follow_reset_pulse();
        return output;
    }

    if(0 == *path_count)
    {
        path_follow_pid_has_target = 0;
        path_follow_reset_pid(0.0f, 0.0f);
        path_follow_reset_pulse();
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
        path_follow_reset_pulse();
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

    path_follow_prepare_pulse(output.target_x10, output.target_y10);
    if(path_follow_pulse_allow_move())
    {
        path_follow_calc_speed(car_dx, car_dy, x_speed, y_speed, &output);
    }
    else
    {
        /* 停顿阶段：不给电机输出，让车体惯性消掉，同时等待视觉位置刷新。 */
        output.x = 0;
        output.y = 0;
        path_follow_reset_pid((float)car_dx, (float)car_dy);
    }

    return output;
}
