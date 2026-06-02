/* gyro_z 角度积分与缩放输出接口 */

/* 防止头文件重复包含 */
#ifndef _gyro_z_angle_h_
/* 头文件保护宏开始 */
#define _gyro_z_angle_h_

/* 引入基础数据类型定义 */
#include "zf_common_headfile.h"

/* 初始化陀螺仪 Z 轴角度模块 */
void gyro_z_angle_init(void);
/* 重置陀螺仪 Z 轴角度模块状态 */
void gyro_z_angle_reset(void);
/* 更新陀螺仪 Z 轴角度积分结果 */
void gyro_z_angle_update(uint16 dt_ms);
/* 设置当前控制量 w */
void gyro_z_angle_set_w(int8 w);
/* 获取陀螺仪 Z 轴原始值 */
int16 gyro_z_angle_get_raw(void);
/* 获取陀螺仪 Z 轴积分值 */
int32 gyro_z_angle_get_integral(void);
/* 获取陀螺仪 Z 轴角度输出值 */
float gyro_z_angle_get_output(void);

/* 头文件保护宏结束 */
#endif
