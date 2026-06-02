#include "main_control_sync.h"

static main_control_sync_status_t sync_status;
static uint8 sim_cells[OPENART_MAP_CELL_MAX];
static uint16 sim_cell_count;
static uint8 sim_cols;
static uint8 sim_rows;
static uint8 sim_valid;
static uint8 last_car_valid;
static main_control_map_pos_t last_car;

static uint16 map_index(const openart_map_t *map, main_control_map_pos_t pos)
{
    return (uint16)pos.y * map->cols + pos.x;
}

static uint8 valid_map(const openart_map_t *map, uint16 *cell_count)
{
    if((0 == map) || (!map->valid) || (0 == map->cols) || (0 == map->rows))
    {
        return 0;
    }

    *cell_count = (uint16)map->cols * map->rows;
    return (*cell_count <= OPENART_MAP_CELL_MAX);
}

static uint8 valid_xy(const openart_map_t *map, int16 x, int16 y)
{
    return ((0 <= x) && (0 <= y) && (x < map->cols) && (y < map->rows));
}

static uint8 can_push_to(uint8 cell)
{
    return ((OPENART_CELL_BACKGROUND == cell) || (OPENART_CELL_GOAL == cell) ||
            ((OPENART_CELL_GOAL_ID_BASE <= cell) && (cell <= OPENART_CELL_GOAL_ID_MAX)));
}

static uint8 is_box_cell(uint8 cell)
{
    return ((OPENART_CELL_YELLOW_BOX == cell) ||
            ((OPENART_CELL_BOX_ID_BASE <= cell) && (cell <= OPENART_CELL_BOX_ID_MAX)));
}

static uint8 is_goal_cell(uint8 cell)
{
    return ((OPENART_CELL_GOAL == cell) ||
            ((OPENART_CELL_GOAL_ID_BASE <= cell) && (cell <= OPENART_CELL_GOAL_ID_MAX)));
}

static uint8 box_matches_goal(uint8 box_cell, uint8 goal_cell)
{
    if((OPENART_CELL_YELLOW_BOX == box_cell) && (OPENART_CELL_GOAL == goal_cell))
    {
        return 1;
    }
    if((OPENART_CELL_BOX_ID_BASE <= box_cell) && (box_cell <= OPENART_CELL_BOX_ID_MAX) &&
       (OPENART_CELL_GOAL_ID_BASE <= goal_cell) && (goal_cell <= OPENART_CELL_GOAL_ID_MAX))
    {
        return ((box_cell - OPENART_CELL_BOX_ID_BASE) == (goal_cell - OPENART_CELL_GOAL_ID_BASE));
    }

    return 0;
}

static uint8 is_box_id_cell(uint8 cell)
{
    return ((OPENART_CELL_BOX_ID_BASE <= cell) && (cell <= OPENART_CELL_BOX_ID_MAX));
}

static uint8 is_goal_id_cell(uint8 cell)
{
    return ((OPENART_CELL_GOAL_ID_BASE <= cell) && (cell <= OPENART_CELL_GOAL_ID_MAX));
}

static void apply_map_id_updates(const openart_map_t *map, uint16 cell_count)
{
    uint16 i;

    for(i = 0; i < cell_count; i++)
    {
        // 扫描状态会直接把地图里的普通目标/箱子改成编号格子，这里同步到模拟地图。
        if(is_goal_id_cell(map->cells[i]) || is_box_id_cell(map->cells[i]))
        {
            sim_cells[i] = map->cells[i];
        }
    }
}

static void copy_cells(uint8 *dst, const uint8 *src, uint16 count)
{
    uint16 i;

    for(i = 0; i < count; i++)
    {
        dst[i] = src[i];
    }
}

static void reset_sim_map(const openart_map_t *map, uint16 cell_count)
{
    copy_cells(sim_cells, map->cells, cell_count);
    sim_cell_count = cell_count;
    sim_cols = map->cols;
    sim_rows = map->rows;
    sim_valid = 1;
    last_car_valid = 0;
}

static void set_pose_cell(openart_pose_t *pose, const openart_map_t *map, main_control_map_pos_t pos)
{
    pose->x10 = (int16)(((uint32)pos.x * 2U + 1U) * map->width10 / ((uint32)map->cols * 2U));
    pose->y10 = (int16)(((uint32)pos.y * 2U + 1U) * map->height10 / ((uint32)map->rows * 2U));
    pose->updated = 1;
    pose->seq++;
}

static void count_sim_cells(void)
{
    uint16 i;

    sync_status.box_count = 0;
    sync_status.goal_count = 0;
    for(i = 0; i < sim_cell_count; i++)
    {
        if(is_box_cell(sim_cells[i]))
        {
            sync_status.box_count++;
        }
        else if(is_goal_cell(sim_cells[i]))
        {
            sync_status.goal_count++;
        }
    }
}

static void update_collision(openart_pose_t *pose, openart_map_t *map)
{
    main_control_map_pos_t car;
    main_control_map_pos_t next;
    int16 dx;
    int16 dy;
    int16 next_x;
    int16 next_y;
    uint16 box_index;
    uint16 next_index;
    uint8 box_cell;
    uint8 next_cell;

    if(!main_control_get_car_map_pos(pose, map, &car))
    {
        last_car_valid = 0;
        return;
    }
    if(!last_car_valid)
    {
        last_car = car;
        last_car_valid = 1;
        return;
    }

    dx = (int16)car.x - last_car.x;
    dy = (int16)car.y - last_car.y;
    if((0 == dx) && (0 == dy))
    {
        return;
    }
    if(((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy)) != 1)
    {
        last_car = car;
        return;
    }

    box_index = map_index(map, car);
    box_cell = sim_cells[box_index];
    if(!is_box_cell(box_cell))
    {
        last_car = car;
        return;
    }

    next_x = (int16)car.x + dx;
    next_y = (int16)car.y + dy;
    if(!valid_xy(map, next_x, next_y))
    {
        set_pose_cell(pose, map, last_car);
        return;
    }

    next.x = (uint8)next_x;
    next.y = (uint8)next_y;
    next_index = map_index(map, next);
    next_cell = sim_cells[next_index];
    if(!can_push_to(next_cell))
    {
        set_pose_cell(pose, map, last_car);
        return;
    }
    if(is_goal_cell(next_cell) && (!box_matches_goal(box_cell, next_cell)))
    {
        set_pose_cell(pose, map, last_car);
        return;
    }

    sim_cells[box_index] = OPENART_CELL_BACKGROUND;
    sim_cells[next_index] = is_goal_cell(next_cell) ? OPENART_CELL_BACKGROUND : box_cell;

    sync_status.map_changed = 1;
    sync_status.box_from = car;
    sync_status.box_to = next;
    if(is_goal_cell(next_cell))
    {
        sync_status.box_completed = 1;
        sync_status.completed_goal = next;
        sync_status.completed_count++;
    }
    else
    {
        sync_status.box_moved = 1;
        sync_status.move_count++;
    }

    last_car = car;
}

void main_control_sync_reset(void)
{
    static const main_control_sync_status_t empty_status = {0};

    sync_status = empty_status;
    sim_cell_count = 0;
    sim_cols = 0;
    sim_rows = 0;
    sim_valid = 0;
    last_car_valid = 0;
}

const main_control_sync_status_t *main_control_sync_update(openart_pose_t *pose, openart_map_t *map)
{
    uint16 cell_count;

    sync_status.valid = 0;
    sync_status.map_changed = 0;
    sync_status.box_moved = 0;
    sync_status.box_completed = 0;
    sync_status.box_count = 0;
    sync_status.goal_count = 0;

    if((0 == pose) || (!valid_map(map, &cell_count)))
    {
        return &sync_status;
    }

    if((!sim_valid) || (sim_cols != map->cols) || (sim_rows != map->rows) || (sim_cell_count != cell_count))
    {
        reset_sim_map(map, cell_count);
    }

    sync_status.valid = 1;
    sync_status.initialized = 1;
    apply_map_id_updates(map, cell_count);
    update_collision(pose, map);
    count_sim_cells();

    copy_cells(map->cells, sim_cells, cell_count);
    map->updated = 1;
    return &sync_status;
}

const main_control_sync_status_t *main_control_sync_get_status(void)
{
    return &sync_status;
}
