#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "car_control/path_follow_control.h"
#include "gyro_z_angle/gyro_z_angle.h"
#include "main_control/main_control.h"
#include "main_control/main_control_sync.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define MAIN_CAR_X_SPEED            (18)
#define MAIN_CAR_Y_SPEED            (10)
#define MAIN_CAR_ARRIVE_PERCENT     (60)
#define MAIN_CONTROL_UPDATE_MS      (20)
#define MAIN_START_STABLE_MS        (500)
#define MAIN_START_RIGHT_SPEED      (18)
#define MAIN_START_RIGHT_MS         (1000)
#define MAIN_RUN_TOTAL_COUNT        (2)
#define MAIN_MAP_REQUEST_MS         (100)

static void main_start_wait_stable(void)
{
    uint16 elapsed_ms;

    car_stop();
    for(elapsed_ms = 0; elapsed_ms < MAIN_START_STABLE_MS; elapsed_ms += MAIN_CONTROL_UPDATE_MS)
    {
        gyro_z_angle_update(MAIN_CONTROL_UPDATE_MS);
        system_delay_ms(MAIN_CONTROL_UPDATE_MS);
    }
    gyro_z_angle_reset();
}

static void main_start_move_right(void)
{
    uint16 elapsed_ms;

    for(elapsed_ms = 0; elapsed_ms < MAIN_START_RIGHT_MS; elapsed_ms += MAIN_CONTROL_UPDATE_MS)
    {
        gyro_z_angle_update(MAIN_CONTROL_UPDATE_MS);
        heading_sensor_update(MAIN_START_RIGHT_SPEED, 0, 0);
        system_delay_ms(MAIN_CONTROL_UPDATE_MS);
    }

    car_stop();
}

static void main_clear_openart_data(openart_pose_t *pose, openart_map_t *map)
{
    if(0 != pose)
    {
        pose->valid = 0;
        pose->updated = 0;
    }

    if(0 != map)
    {
        map->valid = 0;
        map->updated = 0;
    }
}

static void main_restart_run(main_control_context_t *ctx,
                             openart_pose_t *pose,
                             openart_map_t *map)
{
    main_start_wait_stable();
    main_start_move_right();

    main_control_init(ctx, MAIN_CONTROL_UPDATE_MS);
    main_control_sync_reset();
    main_clear_openart_data(pose, map);
    openart_uart_request_map();
}

static void main_request_map_until_valid(const openart_map_t *map,
                                         uint16 *elapsed_ms)
{
    if((0 == map) || (0 == elapsed_ms))
    {
        return;
    }

    if(map->valid)
    {
        *elapsed_ms = 0;
        return;
    }

    if(*elapsed_ms >= MAIN_MAP_REQUEST_MS)
    {
        openart_uart_request_map();
        *elapsed_ms = 0;
    }
    else
    {
        *elapsed_ms += MAIN_CONTROL_UPDATE_MS;
    }
}

static void main_handle_finished(main_control_context_t *ctx,
                                 openart_pose_t *pose,
                                 openart_map_t *map,
                                 uint8 *finished_count)
{
    if((0 == ctx) || (0 == finished_count) ||
       (MAIN_CONTROL_STATE_FINISHED != ctx->state[0]) ||
       (*finished_count >= MAIN_RUN_TOTAL_COUNT))
    {
        return;
    }

    car_stop();
    (*finished_count)++;
    if(*finished_count < MAIN_RUN_TOTAL_COUNT)
    {
        main_restart_run(ctx, pose, map);
    }
}

static void main_drive_path(main_control_context_t *ctx,
                            const openart_pose_t *pose,
                            const openart_map_t *map)
{
    path_follow_output_t follow = {0};

    if((0 == ctx) || (0 == pose) || (0 == map))
    {
        car_stop();
        return;
    }

    if(MAIN_CONTROL_STATE_RUN_PATH != ctx->state[0])
    {
        car_stop();
        openart_display_set_control_status((uint8)ctx->state[0], 0, 0, 0);
        return;
    }

    follow = path_follow_update(pose,
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

    if(follow.valid)
    {
        heading_sensor_update(follow.x, follow.y, 0);
    }
    else
    {
        car_stop();
        main_control_add_task(ctx, MAIN_CONTROL_STATE_ERROR);
        main_control_shift_task(ctx);
    }

    openart_display_set_control_status((uint8)ctx->state[0], follow.valid, follow.x, follow.y);
}

int main(void)
{
    static main_control_context_t main_control;
    openart_pose_t openart_pose = {0};
    openart_map_t openart_map = {0};
    uint8 finished_count = 0;
    uint16 map_request_elapsed_ms = 0;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    heading_sensor_init();
    gyro_z_angle_init();
    main_start_wait_stable();
    main_start_move_right();

    openart_uart_init();
    openart_display_init();
    main_control_init(&main_control, MAIN_CONTROL_UPDATE_MS);
    main_control_sync_reset();

    openart_uart_request_map();

    while(1)
    {
        gyro_z_angle_update(MAIN_CONTROL_UPDATE_MS);
        openart_uart_update(&openart_pose, &openart_map);
        main_request_map_until_valid(&openart_map, &map_request_elapsed_ms);

        if(openart_pose.valid && openart_map.valid)
        {
            main_control_sync_update(&openart_pose, &openart_map);
            main_control_update(&main_control, &openart_pose, &openart_map);
            main_drive_path(&main_control, &openart_pose, &openart_map);
            main_handle_finished(&main_control, &openart_pose, &openart_map, &finished_count);
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
