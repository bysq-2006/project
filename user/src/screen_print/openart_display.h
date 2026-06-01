#ifndef _openart_display_h_
#define _openart_display_h_

#include "zf_common_headfile.h"
#include "../openart_uart/openart_uart.h"


void openart_display_init(void);
void openart_display_set_control_status(uint8 state, uint8 follow_valid, int8 follow_x, int8 follow_y);
void openart_display_update(const openart_pose_t *pose, const openart_map_t *map);

#endif
