#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "car_control/path_follow_control_local.h"
#include "gyro_z_angle/gyro_z_angle.h"
#include "main_control/main_control.h"
#include "main_control/main_control_sync.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define MAIN_CAR_X_SPEED            (20)
#define MAIN_CAR_Y_SPEED            (20)
#define MAIN_CAR_ARRIVE_PERCENT     (30)
#define MAIN_CONTROL_UPDATE_MS      (20)
#define MAIN_TEST_MAP_COLS          (12)
#define MAIN_TEST_MAP_ROWS          (16)
#define MAIN_TEST_MAP_SRC_COLS      (16)
#define MAIN_TEST_MAP_SRC_ROWS      (12)

static void main_drive_path(main_control_context_t *ctx,
                            openart_pose_t *pose,
                            openart_map_t *map)
{
    path_follow_output_t follow = {0};

    if(MAIN_CONTROL_STATE_RUN_PATH == ctx->state[0])
    {
        follow = path_follow_update_local(pose,
                                          map,
                                          ctx->active_path,
                                          &ctx->active_path_count,
                                          MAIN_CAR_X_SPEED,
                                          MAIN_CAR_Y_SPEED,
                                          MAIN_CAR_ARRIVE_PERCENT);

        if(follow.valid && follow.finished)
        {
            car_stop();
            main_control_finish_path(ctx);
            openart_display_set_control_status((uint8)ctx->state[0], follow.valid, follow.x, follow.y);
            return;
        }
    }
    else
    {
        car_stop();
        openart_display_set_control_status((uint8)ctx->state[0], 0, 0, 0);
        return;
    }

    // 正常情况这里要跑。测试的时候临时用car_stop()
    if(follow.valid)
    {
        car_stop();
    }
    else
    {
        car_stop();
        main_control_add_task(ctx, MAIN_CONTROL_STATE_ERROR);
        main_control_shift_task(ctx);
    }

    openart_display_set_control_status((uint8)ctx->state[0], follow.valid, follow.x, follow.y);
}

static uint8 main_test_cell_from_char(char ch)
{
    switch(ch)
    {
        case '#':
        case 0:
            return OPENART_CELL_WALL;
        case '.':
            return OPENART_CELL_GOAL;
        case '$':
            return OPENART_CELL_YELLOW_BOX;
        case '-':
        default:
            return OPENART_CELL_BACKGROUND;
    }
}

static void main_test_scene_init(openart_pose_t *pose, openart_map_t *map)
{
    static const char *rows[MAIN_TEST_MAP_SRC_ROWS] =
    {
        "################",
        "#-#------------#",
        "#-.------#####-#",
        "##$###---#---#-#",
        "#----#---#.#-#-#",
        "#----#####.#-#-#",
        "#-------$--$-#-#",
        "#-----------##-#",
        "#--------------#",
        "#-----####-----#",
        "#--------------#",
        "################"
    };
    uint8 row;
    uint8 col;
    char ch;

    if((0 == pose) || (0 == map))
    {
        return;
    }

    pose->valid = 1;
    pose->updated = 1;
    pose->seq = 0;
    pose->x10 = 15;
    pose->y10 = 15;
    pose->angle10 = 0;

    map->valid = 1;
    map->updated = 1;
    map->seq = 0;
    map->cols = MAIN_TEST_MAP_COLS;
    map->rows = MAIN_TEST_MAP_ROWS;
    map->width10 = MAIN_TEST_MAP_COLS * 10;
    map->height10 = MAIN_TEST_MAP_ROWS * 10;

    for(row = 0; row < MAIN_TEST_MAP_ROWS; row++)
    {
        for(col = 0; col < MAIN_TEST_MAP_COLS; col++)
        {
            // 工程地图是 12 列 16 行，这里把 16 列 12 行的测试图转过来使用。
            if((col < MAIN_TEST_MAP_SRC_ROWS) && (row < MAIN_TEST_MAP_SRC_COLS))
            {
                ch = rows[col][row];
            }
            else
            {
                ch = '#';
            }
            map->cells[(uint16)row * map->cols + col] = main_test_cell_from_char(ch);
        }
    }
}

int main(void)
{
    static main_control_context_t main_control;
    openart_pose_t openart_pose = {0};
    openart_map_t openart_map = {0};

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    heading_sensor_init();
    gyro_z_angle_init();
    openart_uart_init();
    openart_display_init();
    main_control_init(&main_control, MAIN_CONTROL_UPDATE_MS);
    main_control_sync_reset();
    main_test_scene_init(&openart_pose, &openart_map);

    while(1)
    {
        gyro_z_angle_update(20);
        // 本地模拟时暂时不接收 OpenART 坐标和地图，直接使用上面的固定测试场景。
        // openart_uart_update(&openart_pose, &openart_map);
        if(openart_pose.valid && openart_map.valid)
        {
            main_control_sync_update(&openart_pose, &openart_map);
            main_control_update(&main_control, &openart_pose, &openart_map);
            main_drive_path(&main_control, &openart_pose, &openart_map);
        }
        else
        {
            car_stop();
            openart_display_set_control_status((uint8)main_control.state[0], 0, 0, 0);
        }
        openart_display_update(&openart_pose, &openart_map);
        system_delay_ms(MAIN_CONTROL_UPDATE_MS);
    }
}
