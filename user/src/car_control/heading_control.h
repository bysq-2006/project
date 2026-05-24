/*********************************************************************************************************************
* 角度保持控制
*********************************************************************************************************************/

#ifndef _heading_control_h_
#define _heading_control_h_

#include "zf_common_headfile.h"

uint8 heading_control_init(void);
void heading_control_lock_current(void);
void heading_control_set_target(float yaw_deg);
void car_move_xy_heading(int8 x, int8 y);

#endif
