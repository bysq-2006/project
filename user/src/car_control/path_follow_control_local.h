/*********************************************************************************************************************
* path_follow_control_local.h
*
* Local test version of path following.
* It keeps the same path planning logic, but updates the pose structure directly
* instead of calling the real car control interface.
*********************************************************************************************************************/

#ifndef _path_follow_control_local_h_
#define _path_follow_control_local_h_

#include "path_follow_control.h"
#include "../main_control/main_control.h"

#define MAIN_CONTROL_LOCAL_X_SPEED     (40)
#define MAIN_CONTROL_LOCAL_Y_SPEED     (40)
#define MAIN_CONTROL_LOCAL_ARRIVE_PERCENT (30)

typedef struct
{
    main_control_output_t control;
    path_follow_output_t follow;
    uint8 motion_finished;
} main_control_local_output_t;

/*
 * Local test helper.
 *
 * Behavior:
 * - Calls path_follow_update() to get the same steering output as the real version.
 * - Applies that output directly to pose->x10 / pose->y10.
 * - Does not modify the map state or box positions. Box synchronization is
 *   handled by main_control_sync.c in the normal runtime flow.
 *
 * This is intended for offline logic testing, not for real motion control.
 */
path_follow_output_t path_follow_update_local(openart_pose_t *pose,
                                              const openart_map_t *map,
                                              main_control_map_pos_t *path,
                                              uint16 *path_count,
                                              int8 x_speed,
                                              int8 y_speed,
                                              uint8 arrive_percent);

#endif
