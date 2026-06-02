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

static void main_drive_path(main_control_context_t *ctx,
                            openart_pose_t *pose,
                            openart_map_t *map)
{
    path_follow_output_t follow = {0};

    if(MAIN_CONTROL_STATE_RUN_PATH == ctx->state)
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
            main_control_sync_apply_push_result(map, ctx);
            main_control_finish_path(ctx);
            openart_display_set_control_status((uint8)ctx->state, follow.valid, follow.x, follow.y);
            return;
        }
    }
    else
    {
        car_stop();
        openart_display_set_control_status((uint8)ctx->state, 0, 0, 0);
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
        ctx->state = MAIN_CONTROL_STATE_ERROR;
    }

    openart_display_set_control_status((uint8)ctx->state, follow.valid, follow.x, follow.y);
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
    main_control_init(&main_control);
    main_control_sync_reset();

    while(1)
    {
        gyro_z_angle_update(20);
        openart_uart_update(&openart_pose, &openart_map);
        if(openart_pose.valid && openart_map.valid)
        {
            main_control_sync_update(&openart_map);
            main_control_update(&main_control, &openart_pose, &openart_map);
            main_drive_path(&main_control, &openart_pose, &openart_map);
        }
        else
        {
            car_stop();
            openart_display_set_control_status((uint8)main_control.state, 0, 0, 0);
        }
        openart_display_update(&openart_pose, &openart_map);
        system_delay_ms(20);
    }
}
