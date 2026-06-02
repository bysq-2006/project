#include "main_control.h"
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

void main_control_init(main_control_context_t *ctx)
{
    if(0 == ctx)
    {
        return;
    }

    ctx->state = MAIN_CONTROL_STATE_PLAN;
    ctx->box_count = 0;
    ctx->goal_count = 0;
    ctx->plan_count = 0;
    ctx->best_plan_index = 0;
    ctx->active_path_count = 0;
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
    if((0 != ctx) && (MAIN_CONTROL_STATE_RUN_PATH == ctx->state))
    {
        ctx->state = ctx->has_active_plan ? MAIN_CONTROL_STATE_PLAN : MAIN_CONTROL_STATE_FINISHED;
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

    switch(ctx->state)
    {
        case MAIN_CONTROL_STATE_IDLE:
            ctx->state = MAIN_CONTROL_STATE_PLAN;
            break;

        case MAIN_CONTROL_STATE_PLAN:
            if(main_control_build_best_plan(ctx, pose, map))
            {
                ctx->state = MAIN_CONTROL_STATE_RUN_PATH;
                output.plan_ready = 1;
            }
            break;

        case MAIN_CONTROL_STATE_RUN_PATH:
            break;

        case MAIN_CONTROL_STATE_FINISHED:
        case MAIN_CONTROL_STATE_ERROR:
        default:
            break;
    }

    output.state = ctx->state;

    return output;
}
