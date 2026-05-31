#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "map_planner.h"

// 主控候选方案的最大数量。
#define MAIN_CONTROL_PLAN_MAX          (OPENART_MAP_CELL_MAX)
// 箱子路径转弯代价。
#define MAIN_CONTROL_BOX_PATH_TURN_COST (10U)

typedef enum
{
    // 空闲状态。
    MAIN_CONTROL_STATE_IDLE = 0,
    // 正在规划路径。
    MAIN_CONTROL_STATE_PLAN,
    // 正在走到推点。
    MAIN_CONTROL_STATE_MOVE_TO_PUSH_POS,
    // 正在推动箱子。
    MAIN_CONTROL_STATE_PUSH_BOX,
    // 所有任务完成。
    MAIN_CONTROL_STATE_FINISHED,
    // 运行出错。
    MAIN_CONTROL_STATE_ERROR
} main_control_state_t;

// 单个箱子对应的一条候选执行方案。
typedef struct
{
    // 当前箱子坐标。
    main_control_map_pos_t box_pos;
    // 当前目标点坐标。
    main_control_map_pos_t goal_pos;

    // 箱子从当前位置推到目标点的路径。
    main_control_map_pos_t box_path[OPENART_MAP_CELL_MAX];
    // 箱子路径有效点数。
    uint16 box_path_count;
    // 箱子路径代价。
    uint32 box_path_cost;

    // 小车推箱子前要站到的位置。
    main_control_map_pos_t push_pos;
    // 小车到推点的路径。
    main_control_map_pos_t car_path[OPENART_MAP_CELL_MAX];
    // 小车路径有效点数。
    uint16 car_path_count;
    // 小车路径代价。
    uint32 car_path_cost;

    // 该方案的综合代价。
    uint32 total_cost;
    // 该方案是否有效。
    uint8 valid;
} main_control_plan_t;

// 主控长期保存的上下文数据。
typedef struct
{
    // 当前主控状态。
    main_control_state_t state;

    // 地图里找到的箱子列表。
    main_control_map_pos_t boxes[OPENART_MAP_CELL_MAX];
    // 地图里找到的目标点列表。
    main_control_map_pos_t goals[OPENART_MAP_CELL_MAX];
    // 当前箱子数量。
    uint16 box_count;
    // 当前目标点数量。
    uint16 goal_count;

    // 所有候选方案。
    main_control_plan_t plans[MAIN_CONTROL_PLAN_MAX];
    // 当前有效方案数量。
    uint16 plan_count;
    // 当前最优方案下标。
    uint16 best_plan_index;

    // 当前要走的小车路径。
    main_control_map_pos_t active_car_path[OPENART_MAP_CELL_MAX];
    // 当前小车路径长度。
    uint16 active_car_path_count;

    // 当前推箱子阶段的小车路径。
    main_control_map_pos_t active_push_car_path[OPENART_MAP_CELL_MAX];
    // 当前推箱子路径长度。
    uint16 active_push_car_path_count;

    // 当前方案中的箱子起点。
    main_control_map_pos_t active_box_start;
    // 本地测试里箱子的当前坐标。
    main_control_map_pos_t active_box_current;
    // 当前方案中的箱子终点。
    main_control_map_pos_t active_box_end;
    // 当前方案中的目标点。
    main_control_map_pos_t active_goal;

    // 重新规划次数。
    uint32 replan_count;
    // 当前是否有可执行方案。
    uint8 has_active_plan;
} main_control_context_t;

// 单次更新的返回结果。
typedef struct
{
    // 更新后的主控状态。
    main_control_state_t state;
    // 本次调用是否有效。
    uint8 valid;
    // 本次是否刚刚生成新方案。
    uint8 plan_ready;
} main_control_output_t;

// 初始化主控上下文。
void main_control_init(main_control_context_t *ctx);
// 推进一次主控状态机。
main_control_output_t main_control_update(main_control_context_t *ctx,
                                          openart_pose_t *pose,
                                          openart_map_t *map);
// 通知主控小车已到达推点。
void main_control_finish_move_to_push_pos(main_control_context_t *ctx);
// 通知主控当前推箱子阶段已完成。
void main_control_finish_push_box(main_control_context_t *ctx);

#endif
