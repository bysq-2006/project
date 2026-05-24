#ifndef _CAR_PARAMS_H_
#define _CAR_PARAMS_H_

/*
 * car_control.c 参数
 * 这里调电机输出限幅、PWM 频率、每个电机的输出增益。
 */
#define CAR_MAX_DUTY                        (90)
#define CAR_MOTOR_PWM_FREQ_HZ               (17000)
#define MOTOR1_GAIN_PERCENT                 (100)
#define MOTOR2_GAIN_PERCENT                 (100)
#define MOTOR3_GAIN_PERCENT                 (100)
#define MOTOR4_GAIN_PERCENT                 (100)

/*
 * heading_control.c 参数
 * 这里调角度保持力度、死区、转向输出限幅、陀螺仪积分方向和校准次数。
 */
#define HEADING_KP_PERCENT                  (80)
#define HEADING_DEADBAND_DEG_X10            (10)
#define HEADING_TURN_MIN                    (0)
#define HEADING_TURN_MAX                    (35)
#define HEADING_UPDATE_PERIOD_S             (0.01f)
#define HEADING_GYRO_Z_SIGN                 (1.0f)
#define HEADING_GYRO_CALIBRATION_COUNT      (100)
#define HEADING_GYRO_CALIBRATION_DELAY_MS   (5)

#endif
