#include "main_control.h"
#include "main_control_find_ids.h"
#include "main_control_plan.h"

static void main_control_clear_output(main_control_output_t *output)
{
    if(0 == output)
    {
        return;
    }

    output->state = MAIN_CONTROL_STATE_IDLE;
    output->valid = 0;
    output->plan_ready = 0;
}

void main_control_add_task(main_control_context_t *ctx,
                           main_control_state_t state)
{
    uint8 i;

    if(0 == ctx)
    {
        return;
    }

    for(i = 0; i < MAIN_CONTROL_TASK_MAX; i++)
    {
        if(MAIN_CONTROL_TASK_EMPTY == ctx->state[i])
        {
            ctx->state[i] = state;
            return;
        }
    }
}

void main_control_shift_task(main_control_context_t *ctx)
{
    uint8 i;

    if(0 == ctx)
    {
        return;
    }

    for(i = 1; i < MAIN_CONTROL_TASK_MAX; i++)
    {
        ctx->state[i - 1] = ctx->state[i];
    }

    ctx->state[MAIN_CONTROL_TASK_MAX - 1] = MAIN_CONTROL_TASK_EMPTY;
    if(MAIN_CONTROL_TASK_EMPTY == ctx->state[0])
    {
        ctx->state[0] = MAIN_CONTROL_STATE_IDLE;
    }
}

void main_control_init(main_control_context_t *ctx)
{
    uint8 i;

    if(0 == ctx)
    {
        return;
    }

    for(i = 0; i < MAIN_CONTROL_TASK_MAX; i++)
    {
        ctx->state[i] = MAIN_CONTROL_TASK_EMPTY;
    }
    ctx->state[0] = MAIN_CONTROL_STATE_IDLE;
    ctx->box_count = 0;
    ctx->goal_count = 0;
    ctx->plan_count = 0;
    ctx->best_plan_index = 0;
    ctx->active_path_count = 0;
    ctx->target_heading_angle = 0;
    ctx->turn_start_ms = MAIN_CONTROL_TURN_TIME_INVALID;
    ctx->active_box_start.x = 0;
    ctx->active_box_start.y = 0;
    ctx->active_box_current.x = 0;
    ctx->active_box_current.y = 0;
    ctx->active_box_end.x = 0;
    ctx->active_box_end.y = 0;
    ctx->active_goal.x = 0;
    ctx->active_goal.y = 0;
    ctx->replan_count = 0;
    ctx->has_active_plan = 0;
}

void main_control_finish_path(main_control_context_t *ctx)
{
    if((0 != ctx) && (MAIN_CONTROL_STATE_RUN_PATH == ctx->state[0]))
    {
        main_control_add_task(ctx, ctx->has_active_plan ? MAIN_CONTROL_STATE_PLAN : MAIN_CONTROL_STATE_FINISHED);
        main_control_shift_task(ctx);
        ctx->has_active_plan = 0;
        ctx->active_path_count = 0;
    }
}

main_control_output_t main_control_update(main_control_context_t *ctx,
                                          openart_pose_t *pose,
                                          openart_map_t *map)
{
    main_control_output_t output;

    main_control_clear_output(&output);
    if((0 == ctx) || (0 == pose) || (0 == map))
    {
        output.state = MAIN_CONTROL_STATE_ERROR;
        return output;
    }

    output.valid = 1;
    if(MAIN_CONTROL_TASK_EMPTY == ctx->state[0])
    {
        ctx->state[0] = MAIN_CONTROL_STATE_ERROR;
    }

    switch(ctx->state[0])
    {
        case MAIN_CONTROL_STATE_IDLE:
            main_control_add_task(ctx, MAIN_CONTROL_STATE_FIND_IDS);
            main_control_shift_task(ctx);
            break;

        case MAIN_CONTROL_STATE_FIND_IDS:
            if(main_control_find_ids_main(ctx, pose, map))
            {
                main_control_add_task(ctx, MAIN_CONTROL_STATE_PLAN);
                main_control_shift_task(ctx);
            }
            break;

        case MAIN_CONTROL_STATE_PLAN:
            if(main_control_build_best_plan(ctx, pose, map))
            {
                output.plan_ready = 1;
            }
            break;

        case MAIN_CONTROL_STATE_TURN:
            if(MAIN_CONTROL_TURN_TIME_INVALID == ctx->turn_start_ms)
            {
                ctx->target_heading_angle = MAIN_CONTROL_TURN_TARGET;
                ctx->turn_start_ms = timer_get(GPT_TIM_1);
            }
            if((uint32)(timer_get(GPT_TIM_1) - ctx->turn_start_ms) >= MAIN_CONTROL_TURN_STABLE_MS)
            {
                ctx->turn_start_ms = MAIN_CONTROL_TURN_TIME_INVALID;
                main_control_shift_task(ctx);
            }
            break;

        case MAIN_CONTROL_STATE_RUN_PATH:
            break;

        case MAIN_CONTROL_STATE_FINISHED:
        case MAIN_CONTROL_STATE_ERROR:
        default:
            break;
    }

    output.state = ctx->state[0];

    return output;
}
