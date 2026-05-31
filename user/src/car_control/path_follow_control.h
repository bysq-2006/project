/*********************************************************************************************************************
* path_follow_control.h
*********************************************************************************************************************/

#ifndef _path_follow_control_h_
#define _path_follow_control_h_

#include "zf_common_headfile.h"
#include "../main_control/main_control.h"

typedef struct
{
    // 本次建议给 car_move_xy() 的 x 轴速度，正数向右，负数向左。
    int8 x;
    // 本次建议给 car_move_xy() 的 y 轴速度，正数向前，负数向后。
    int8 y;
    // 1 表示本次输出有效；0 表示输入参数、地图或车位姿无效。
    uint8 valid;
    // 1 表示本次调用时已经到达并移除了原来的第一个路径点。
    uint8 arrived;
    // 1 表示路径已经走完，此时 x/y 为 0，外部可以 car_stop()。
    uint8 finished;
    // 本次实际追踪的路径点，也就是移动后的 path[0]。
    main_control_map_pos_t target;
    // target 换算后的绝对 x 坐标，单位与 openart_pose_t.x10 相同。
    int16 target_x10;
    // target 换算后的绝对 y 坐标，单位与 openart_pose_t.y10 相同。
    int16 target_y10;
} path_follow_output_t;

/*
 * 路径跟随主函数。
 *
 * 典型用法：
 *
 *     path_follow_output_t out;
 *
 *     out = path_follow_update(&openart_pose, &openart_map,
 *                              path, &path_count,
 *                              40, 40, 30);
 *
 *     if(out.valid && !out.finished)
 *     {
 *         car_move_xy(out.x, out.y);
 *     }
 *     else
 *     {
 *         car_stop();
 *     }
 *
 * 参数说明：
 * pose            当前小车绝对坐标，使用 openart_pose_t.x10/y10。本函数只读取，不会修改。
 * map             当前地图，用 map->cols/rows 和 width10/height10 把格子坐标换成绝对坐标。本函数只读取，不会修改。
 * path            可修改的路径数组，元素类型是 main_control_map_pos_t。
 * path_count      路径点数量指针。到达第一个点后，本函数会把 path 整体前移一位并让数量减 1。
 * x_speed         x 轴最大速度幅值，只需要传正数；方向由本函数按目标点位置自动决定。
 * y_speed         y 轴最大速度幅值，只需要传正数；方向由本函数按目标点位置自动决定。
 * arrive_percent  到达判定范围，占单个格子宽高的百分比。例如 30 表示进入目标格子中心附近 30% 范围。
 *
 * 返回值：
 * valid 为 1 且 finished 为 0 时，把返回的 x/y 直接传给 car_move_xy() 即可。
 * finished 为 1 时表示路径为空或刚刚走完，建议外部停车。
 */
path_follow_output_t path_follow_update(const openart_pose_t *pose,
                                        const openart_map_t *map,
                                        main_control_map_pos_t *path,
                                        uint16 *path_count,
                                        int8 x_speed,
                                        int8 y_speed,
                                        uint8 arrive_percent);

#endif
