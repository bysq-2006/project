#ifndef _CAR_PARAMS_H_
#define _CAR_PARAMS_H_

/*
 * car_control.c 参数
 * 这里统一调整小车底盘电机相关参数。
 */

/*
 * 电机最大输出占空比，单位：百分比。
 * 例如 90 表示最终输出不会超过 90% PWM。
 * 注意：四个轮子混控和增益计算后，如果任意一个轮子超过这个值，
 * 程序会把四个轮子的输出一起按比例缩小，保持四轮之间的比例不变。
 */
#define CAR_MAX_DUTY                        (90)

/*
 * 电机 PWM 频率，单位：Hz。
 * 这是 PWM 开关切换频率，一般不用调，不影响车子的运动方向和运动逻辑。
 */
#define CAR_MOTOR_PWM_FREQ_HZ               (17000)

/*
 * 四个电机的单独输出增益，单位：百分比。
 * 可以按“电压比例”理解：
 *     某个轮子的输出 = 输入电压 * 对应 MOTORx_GAIN_PERCENT / 100
 * 例如给 100 就是原来的 100%，给 120 就是这个轮子单独按 120% 输出。
 * 实际代码调的是 PWM 占空比；如果超过 CAR_MAX_DUTY，会被统一限幅。
 */
#define MOTOR1_GAIN_PERCENT                 (100)
#define MOTOR2_GAIN_PERCENT                 (100)
#define MOTOR3_GAIN_PERCENT                 (100)
#define MOTOR4_GAIN_PERCENT                 (100)

/*
 * heading_control.c 参数
 * 航向保持 P 系数。
 * 当前旋转输出 w = (当前 x 轴原始值 - 启动时 x 轴原始值) * HEADING_CONTROL_P。
 * 如果回正方向反了，把这个值改成负数。
 */
#define HEADING_CONTROL_P                   (0.01f)
#define HEADING_CONTROL_I                   (0.0001f)
#define HEADING_CONTROL_D                   (0.0f)
#define HEADING_CONTROL_I_LIMIT             (5000.0f)

#endif
