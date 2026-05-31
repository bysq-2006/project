#include "main_control.h"

// 判断地图格子是否允许 A* 通行。
static uint8 main_control_is_walkable_cell(uint8 cell)
{
    return ((OPENART_CELL_BACKGROUND == cell) || (OPENART_CELL_GOAL == cell));
}

// 把二维地图坐标转换成 cells[] 一维下标。
static uint16 main_control_map_index(const openart_map_t *map, uint8 x, uint8 y)
{
    return (uint16)y * map->cols + x;
}

// 计算当前点到目标点的曼哈顿距离启发值。
static uint32 main_control_astar_heuristic(main_control_map_pos_t pos, main_control_map_pos_t target)
{
    uint32 dx;
    uint32 dy;

    dx = (pos.x > target.x) ? (uint32)(pos.x - target.x) : (uint32)(target.x - pos.x);
    dy = (pos.y > target.y) ? (uint32)(pos.y - target.y) : (uint32)(target.y - pos.y);

    return (dx + dy) * MAIN_CONTROL_ASTAR_MOVE_COST;
}

// 把 A* 内部状态编号还原成地图坐标。
static main_control_map_pos_t main_control_astar_state_pos(const openart_map_t *map, uint16 state)
{
    uint16 index;
    main_control_map_pos_t pos;

    index = state / MAIN_CONTROL_ASTAR_DIR_COUNT;
    pos.x = (uint8)(index % map->cols);
    pos.y = (uint8)(index / map->cols);

    return pos;
}

// 在地图中查找所有箱子坐标。
uint16 main_control_find_boxes(const openart_map_t *map, main_control_map_pos_t *boxes, uint16 max_boxes)
{
    uint8 row;
    uint8 col;
    uint16 count;
    uint16 index;

    count = 0;

    if((0 == map) || (0 == boxes) || (0 == max_boxes) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    for(row = 0; row < map->rows; row++)
    {
        for(col = 0; col < map->cols; col++)
        {
            index = (uint16)row * map->cols + col;
            if(OPENART_CELL_YELLOW_BOX == map->cells[index])
            {
                if(count < max_boxes)
                {
                    boxes[count].x = col;
                    boxes[count].y = row;
                }
                count++;
            }
        }
    }

    return count;
}

// 在地图中查找所有目标点坐标。
uint16 main_control_find_goals(const openart_map_t *map, main_control_map_pos_t *goals, uint16 max_goals)
{
    uint8 row;
    uint8 col;
    uint16 count;
    uint16 index;

    count = 0;

    if((0 == map) || (0 == goals) || (0 == max_goals) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    for(row = 0; row < map->rows; row++)
    {
        for(col = 0; col < map->cols; col++)
        {
            index = (uint16)row * map->cols + col;
            if(OPENART_CELL_GOAL == map->cells[index])
            {
                if(count < max_goals)
                {
                    goals[count].x = col;
                    goals[count].y = row;
                }
                count++;
            }
        }
    }

    return count;
}

// 把车的精确坐标换算成地图上的格子坐标。
uint8 main_control_get_car_map_pos(const openart_pose_t *pose, const openart_map_t *map, main_control_map_pos_t *car_pos)
{
    int32 col;
    int32 row;

    if((0 == pose) || (0 == map) || (0 == car_pos) || (!pose->valid) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows) || (0 == map->width10) || (0 == map->height10) ||
       (pose->x10 < 0) || (pose->y10 < 0) ||
       ((int32)pose->x10 >= (int32)map->width10) || ((int32)pose->y10 >= (int32)map->height10))
    {
        return 0;
    }

    col = ((int32)pose->x10 * map->cols) / map->width10;
    row = ((int32)pose->y10 * map->rows) / map->height10;

    car_pos->x = (uint8)col;
    car_pos->y = (uint8)row;

    return 1;
}

// 普通 A*，用于小车自身寻路。输出路径按起点到终点顺序写入。
uint32 main_control_astar_find_car_path(const openart_map_t *map,
                                        main_control_map_pos_t start,
                                        main_control_map_pos_t target,
                                        main_control_map_pos_t *path)
{
    static uint32 g_cost[OPENART_MAP_CELL_MAX];
    static int16 parent[OPENART_MAP_CELL_MAX];
    static uint8 open[OPENART_MAP_CELL_MAX];
    static uint8 closed[OPENART_MAP_CELL_MAX];
    static main_control_map_pos_t reverse_path[OPENART_MAP_CELL_MAX];
    static const int16 dx[4] = {0, 1, 0, -1};
    static const int16 dy[4] = {-1, 0, 1, 0};

    uint16 cell_count;
    uint16 i;
    uint16 current;
    uint16 next_index;
    uint16 start_index;
    uint16 target_index;
    uint16 best_target_index;
    uint16 path_index;
    uint8 dir;
    int16 next_x;
    int16 next_y;
    uint32 best_f;
    uint32 next_g;
    uint32 target_cost;
    main_control_map_pos_t current_pos;

    if((0 == map) || (0 == path) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return MAIN_CONTROL_PATH_COST_INVALID;
    }

    cell_count = (uint16)map->cols * map->rows;
    if((start.x >= map->cols) || (start.y >= map->rows) ||
       (target.x >= map->cols) || (target.y >= map->rows))
    {
        return MAIN_CONTROL_PATH_COST_INVALID;
    }

    start_index = main_control_map_index(map, start.x, start.y);
    target_index = main_control_map_index(map, target.x, target.y);
    if((!main_control_is_walkable_cell(map->cells[start_index])) ||
       (!main_control_is_walkable_cell(map->cells[target_index])))
    {
        return MAIN_CONTROL_PATH_COST_INVALID;
    }

    for(i = 0; i < cell_count; i++)
    {
        g_cost[i] = MAIN_CONTROL_PATH_COST_INVALID;
        parent[i] = -1;
        open[i] = 0;
        closed[i] = 0;
    }

    current = start_index;
    g_cost[current] = 0;
    open[current] = 1;
    best_target_index = OPENART_MAP_CELL_MAX;
    target_cost = MAIN_CONTROL_PATH_COST_INVALID;

    while(1)
    {
        current = OPENART_MAP_CELL_MAX;
        best_f = MAIN_CONTROL_PATH_COST_INVALID;

        for(i = 0; i < cell_count; i++)
        {
            if(open[i])
            {
                current_pos.x = (uint8)(i % map->cols);
                current_pos.y = (uint8)(i / map->cols);
                next_g = g_cost[i] + main_control_astar_heuristic(current_pos, target);
                if(next_g < best_f)
                {
                    best_f = next_g;
                    current = i;
                }
            }
        }

        if(OPENART_MAP_CELL_MAX == current)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        open[current] = 0;
        closed[current] = 1;

        if(current == target_index)
        {
            best_target_index = current;
            target_cost = g_cost[current];
            break;
        }

        current_pos.x = (uint8)(current % map->cols);
        current_pos.y = (uint8)(current / map->cols);

        for(dir = MAIN_CONTROL_ASTAR_DIR_UP; dir <= MAIN_CONTROL_ASTAR_DIR_LEFT; dir++)
        {
            next_x = (int16)current_pos.x + dx[dir];
            next_y = (int16)current_pos.y + dy[dir];

            if((next_x < 0) || (next_y < 0) ||
               (next_x >= map->cols) || (next_y >= map->rows))
            {
                continue;
            }

            next_index = main_control_map_index(map, (uint8)next_x, (uint8)next_y);
            if((!main_control_is_walkable_cell(map->cells[next_index])) ||
               closed[next_index])
            {
                continue;
            }

            next_g = g_cost[current] + MAIN_CONTROL_ASTAR_MOVE_COST;
            if(next_g < g_cost[next_index])
            {
                g_cost[next_index] = next_g;
                parent[next_index] = (int16)current;
                open[next_index] = 1;
            }
        }
    }

    current = best_target_index;
    path_index = 0;

    while(OPENART_MAP_CELL_MAX != current)
    {
        if(path_index >= OPENART_MAP_CELL_MAX)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        reverse_path[path_index].x = (uint8)(current % map->cols);
        reverse_path[path_index].y = (uint8)(current / map->cols);
        if((reverse_path[path_index].x == start.x) && (reverse_path[path_index].y == start.y))
        {
            for(i = 0; i <= path_index; i++)
            {
                path[i] = reverse_path[path_index - i];
            }
            return target_cost;
        }

        if(parent[current] < 0)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        current = (uint16)parent[current];
        path_index++;
    }

    return MAIN_CONTROL_PATH_COST_INVALID;
}

// 推箱子游戏的 A * 算法。该算法计入转向代价，并校验角色所在格子状态。
uint32 main_control_astar_find_path(const openart_map_t *map,
                                    main_control_map_pos_t start,
                                    main_control_map_pos_t target,
                                    main_control_map_pos_t *path,
                                    uint16 turn_cost)
{
    static uint32 g_cost[MAIN_CONTROL_ASTAR_STATE_MAX];
    static int16 parent[MAIN_CONTROL_ASTAR_STATE_MAX];
    static uint8 open[MAIN_CONTROL_ASTAR_STATE_MAX];
    static uint8 closed[MAIN_CONTROL_ASTAR_STATE_MAX];
    static main_control_map_pos_t reverse_path[OPENART_MAP_CELL_MAX];
    static const int16 dx[4] = {0, 1, 0, -1};
    static const int16 dy[4] = {-1, 0, 1, 0};

    uint16 cell_count;
    uint16 state_count;
    uint16 i;
    uint16 current;
    uint16 current_index;
    uint16 next_index;
    uint16 next_state;
    uint16 push_index;
    uint16 start_index;
    uint16 target_index;
    uint16 best_target_state;
    uint16 path_index;
    uint8 dir;
    uint8 current_dir;
    int16 next_x;
    int16 next_y;
    int16 push_x;
    int16 push_y;
    uint32 best_f;
    uint32 next_g;
    uint32 target_cost;
    main_control_map_pos_t current_pos;

    if((0 == map) || (0 == path) || (!map->valid) ||
       (0 == map->cols) || (0 == map->rows))
    {
        return MAIN_CONTROL_PATH_COST_INVALID;
    }

    cell_count = (uint16)map->cols * map->rows;
    if((start.x >= map->cols) || (start.y >= map->rows) ||
       (target.x >= map->cols) || (target.y >= map->rows))
    {
        return MAIN_CONTROL_PATH_COST_INVALID;
    }

    start_index = main_control_map_index(map, start.x, start.y);
    target_index = main_control_map_index(map, target.x, target.y);

    state_count = cell_count * MAIN_CONTROL_ASTAR_DIR_COUNT;
    for(i = 0; i < state_count; i++)
    {
        g_cost[i] = MAIN_CONTROL_PATH_COST_INVALID;
        parent[i] = -1;
        open[i] = 0;
        closed[i] = 0;
    }

    current = start_index * MAIN_CONTROL_ASTAR_DIR_COUNT + MAIN_CONTROL_ASTAR_DIR_NONE;
    g_cost[current] = 0;
    open[current] = 1;
    best_target_state = MAIN_CONTROL_ASTAR_STATE_MAX;
    target_cost = MAIN_CONTROL_PATH_COST_INVALID;

    while(1)
    {
        current = MAIN_CONTROL_ASTAR_STATE_MAX;
        best_f = MAIN_CONTROL_PATH_COST_INVALID;

        for(i = 0; i < state_count; i++)
        {
            if(open[i])
            {
                current_pos = main_control_astar_state_pos(map, i);
                next_g = g_cost[i] + main_control_astar_heuristic(current_pos, target);
                if(next_g < best_f)
                {
                    best_f = next_g;
                    current = i;
                }
            }
        }

        if(MAIN_CONTROL_ASTAR_STATE_MAX == current)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        open[current] = 0;
        closed[current] = 1;
        current_index = current / MAIN_CONTROL_ASTAR_DIR_COUNT;

        if(current_index == target_index)
        {
            best_target_state = current;
            target_cost = g_cost[current];
            break;
        }

        current_pos = main_control_astar_state_pos(map, current);
        current_dir = (uint8)(current % MAIN_CONTROL_ASTAR_DIR_COUNT);

        for(dir = MAIN_CONTROL_ASTAR_DIR_UP; dir <= MAIN_CONTROL_ASTAR_DIR_LEFT; dir++)
        {
            next_x = (int16)current_pos.x + dx[dir];
            next_y = (int16)current_pos.y + dy[dir];

            if((next_x < 0) || (next_y < 0) ||
               (next_x >= map->cols) || (next_y >= map->rows))
            {
                continue;
            }

            next_index = main_control_map_index(map, (uint8)next_x, (uint8)next_y);
            if((start_index != next_index) &&
               (!main_control_is_walkable_cell(map->cells[next_index])))
            {
                continue;
            }

            push_x = (int16)current_pos.x - dx[dir];
            push_y = (int16)current_pos.y - dy[dir];

            if((push_x < 0) || (push_y < 0) ||
               (push_x >= map->cols) || (push_y >= map->rows))
            {
                continue;
            }

            push_index = main_control_map_index(map, (uint8)push_x, (uint8)push_y);
            if((start_index != push_index) &&
               (!main_control_is_walkable_cell(map->cells[push_index])))
            {
                continue;
            }

            next_state = next_index * MAIN_CONTROL_ASTAR_DIR_COUNT + dir;
            if(closed[next_state])
            {
                continue;
            }

            next_g = g_cost[current] + MAIN_CONTROL_ASTAR_MOVE_COST;
            if((MAIN_CONTROL_ASTAR_DIR_NONE != current_dir) && (current_dir != dir))
            {
                next_g += turn_cost;
            }

            if(next_g < g_cost[next_state])
            {
                g_cost[next_state] = next_g;
                parent[next_state] = (int16)current;
                open[next_state] = 1;
            }
        }
    }

    current = best_target_state;
    path_index = 0;

    while(MAIN_CONTROL_ASTAR_STATE_MAX != current)
    {
        if(path_index >= OPENART_MAP_CELL_MAX)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        reverse_path[path_index] = main_control_astar_state_pos(map, current);
        if((reverse_path[path_index].x == start.x) && (reverse_path[path_index].y == start.y))
        {
            for(i = 0; i <= path_index; i++)
            {
                path[i] = reverse_path[path_index - i];
            }
            return target_cost;
        }

        if(parent[current] < 0)
        {
            return MAIN_CONTROL_PATH_COST_INVALID;
        }

        current = (uint16)parent[current];
        path_index++;
    }

    return MAIN_CONTROL_PATH_COST_INVALID;
}
