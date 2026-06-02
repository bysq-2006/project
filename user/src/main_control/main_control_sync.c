#include "main_control_sync.h"

static main_control_sync_status_t sync_status;
static uint8 last_cells[OPENART_MAP_CELL_MAX];
static uint16 last_cell_count;
static uint8 last_cols;
static uint8 last_rows;

static uint8 is_box(uint8 cell)
{
    return (OPENART_CELL_YELLOW_BOX == cell);
}

static uint8 is_goal(uint8 cell)
{
    return (OPENART_CELL_GOAL == cell);
}

static uint16 map_index(const openart_map_t *map, main_control_map_pos_t pos)
{
    return (uint16)pos.y * map->cols + pos.x;
}

static main_control_map_pos_t index_pos(const openart_map_t *map, uint16 index)
{
    main_control_map_pos_t pos;

    pos.x = (uint8)(index % map->cols);
    pos.y = (uint8)(index / map->cols);
    return pos;
}

static uint8 same_pos(main_control_map_pos_t a, main_control_map_pos_t b)
{
    return ((a.x == b.x) && (a.y == b.y));
}

static uint8 valid_pos(const openart_map_t *map, main_control_map_pos_t pos)
{
    return ((0 != map) && map->valid && (0 != map->cols) && (0 != map->rows) &&
            (pos.x < map->cols) && (pos.y < map->rows));
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

static void store_map(const openart_map_t *map, uint16 cell_count)
{
    uint16 i;

    for(i = 0; i < cell_count; i++)
    {
        last_cells[i] = map->cells[i];
    }

    last_cell_count = cell_count;
    last_cols = map->cols;
    last_rows = map->rows;
}

void main_control_sync_reset(void)
{
    static const main_control_sync_status_t empty_status = {0};

    sync_status = empty_status;
    last_cell_count = 0;
    last_cols = 0;
    last_rows = 0;
}

void main_control_sync_apply_push_result(openart_map_t *map, const main_control_context_t *ctx)
{
    main_control_map_pos_t old_pos;
    main_control_map_pos_t new_pos;

    if((0 == map) || (0 == ctx) || (!ctx->has_active_plan))
    {
        return;
    }

    old_pos = ctx->active_box_current;
    new_pos = ctx->active_box_end;
    if((!valid_pos(map, old_pos)) || (!valid_pos(map, new_pos)) || (!valid_pos(map, ctx->active_goal)))
    {
        return;
    }

    map->cells[map_index(map, old_pos)] = OPENART_CELL_BACKGROUND;
    map->cells[map_index(map, new_pos)] = same_pos(new_pos, ctx->active_goal) ?
                                          OPENART_CELL_BACKGROUND :
                                          OPENART_CELL_YELLOW_BOX;
    map->updated = 1;
    map->seq++;
}

const main_control_sync_status_t *main_control_sync_update(const openart_map_t *map)
{
    uint16 i;
    uint16 cell_count;
    uint8 old_cell;
    uint8 new_cell;
    uint8 removed_box;
    uint8 added_box;
    uint8 removed_goal;
    uint8 compare_last;

    sync_status.valid = 0;
    sync_status.map_changed = 0;
    sync_status.box_moved = 0;
    sync_status.box_completed = 0;
    sync_status.box_count = 0;
    sync_status.goal_count = 0;
    removed_box = 0;
    added_box = 0;
    removed_goal = 0;

    if(!valid_map(map, &cell_count))
    {
        return &sync_status;
    }

    sync_status.valid = 1;
    compare_last = (sync_status.initialized && (last_cols == map->cols) && (last_rows == map->rows));

    for(i = 0; i < cell_count; i++)
    {
        new_cell = map->cells[i];

        if(is_box(new_cell))
        {
            sync_status.box_count++;
        }
        else if(is_goal(new_cell))
        {
            sync_status.goal_count++;
        }

        if(!compare_last)
        {
            continue;
        }

        old_cell = (i < last_cell_count) ? last_cells[i] : OPENART_CELL_UNKNOWN;
        if(old_cell != new_cell)
        {
            sync_status.map_changed = 1;
        }
        if((!removed_box) && is_box(old_cell) && (!is_box(new_cell)))
        {
            removed_box = 1;
            sync_status.box_from = index_pos(map, i);
        }
        if((!added_box) && (!is_box(old_cell)) && is_box(new_cell))
        {
            added_box = 1;
            sync_status.box_to = index_pos(map, i);
        }
        if((!removed_goal) && is_goal(old_cell) && (!is_goal(new_cell)))
        {
            removed_goal = 1;
            sync_status.completed_goal = index_pos(map, i);
        }
    }

    if(!compare_last)
    {
        sync_status.initialized = 1;
        store_map(map, cell_count);
        return &sync_status;
    }

    if(removed_box && added_box)
    {
        sync_status.box_moved = 1;
        sync_status.move_count++;
    }
    else if(removed_box && removed_goal)
    {
        sync_status.box_completed = 1;
        sync_status.box_to = sync_status.completed_goal;
        sync_status.completed_count++;
    }

    store_map(map, cell_count);
    return &sync_status;
}

const main_control_sync_status_t *main_control_sync_get_status(void)
{
    return &sync_status;
}
