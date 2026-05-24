/*********************************************************************************************************************
* IMU660RB 角度传感器基础框架
*********************************************************************************************************************/

#ifndef _heading_control_h_
#define _heading_control_h_

#include "zf_common_headfile.h"

uint8 heading_sensor_init(void);
void heading_sensor_update(int8 x, int8 y);

#endif
