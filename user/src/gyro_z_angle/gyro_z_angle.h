/*********************************************************************************************************************
* gyro_z 原始值积分与缩放输出
*********************************************************************************************************************/

#ifndef _gyro_z_angle_h_
#define _gyro_z_angle_h_

#include "zf_common_headfile.h"

void gyro_z_angle_init(void);
void gyro_z_angle_reset(void);
void gyro_z_angle_update(uint16 dt_ms);
void gyro_z_angle_set_w(int8 w);
int16 gyro_z_angle_get_raw(void);
int32 gyro_z_angle_get_integral(void);
float gyro_z_angle_get_output(void);

#endif
