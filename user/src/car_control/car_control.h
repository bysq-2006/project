/*********************************************************************************************************************
* RT1064DVL6A Open Source Library
* car_control.h
*********************************************************************************************************************/

#ifndef _car_control_h_
#define _car_control_h_

#include "zf_common_headfile.h"

void car_init(void);
void car_stop(void);
void car_move_xy(int8 x, int8 y);
void car_move_xyw(int8 x, int8 y, int8 w);

#endif
