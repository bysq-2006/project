#ifndef _MAIN_CONTROL_PLAN_H_
#define _MAIN_CONTROL_PLAN_H_

#include "main_control.h"

uint8 main_control_build_best_plan(main_control_context_t *ctx,
                                   const openart_pose_t *pose,
                                   const openart_map_t *map);

#endif
