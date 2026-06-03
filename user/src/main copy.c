#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "car_control/path_follow_control.h"
#include "gyro_z_angle/gyro_z_angle.h"
#include "main_control/main_control.h"
#include "openart_uart/openart_uart.h"
#include "screen_print/openart_display.h"

#define MAIN_CAR_X_SPEED            (20)
#define MAIN_CAR_Y_SPEED            (20)
#define MAIN_CAR_ARRIVE_PERCENT     (30)
#define MAIN_CONTROL_UPDATE_MS      (20)
#define MAIN_MAP_REQUEST_MS         (500)

static void main_request_map_periodic(uint16 *elapsed_ms)
{
    if(0 == elapsed_ms)
    {
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
        openart_uart_request_map();
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
    uint16 map_request_elapsed_ms = MAIN_MAP_REQUEST_MS;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    heading_sensor_init();
    gyro_z_angle_init();
    openart_uart_init();
    openart_display_init();
    main_control_init(&main_control, MAIN_CONTROL_UPDATE_MS);

    openart_uart_request_map();
    map_request_elapsed_ms = 0;

    while(1)
    {
        gyro_z_angle_update(MAIN_CONTROL_UPDATE_MS);
        openart_uart_update(&openart_pose, &openart_map);
        main_request_map_periodic(&map_request_elapsed_ms);

        if(openart_pose.valid && openart_map.valid)
        {
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
