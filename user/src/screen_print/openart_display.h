#ifndef _openart_display_h_
#define _openart_display_h_

#include "zf_common_headfile.h"


void openart_display_init(void);
void openart_display_set_control_status(uint8 state, uint8 follow_valid, int8 follow_x, int8 follow_y);
void openart_display_update(void);

#endif
