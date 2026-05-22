/*********************************************************************************************************************
* RT1064DVL6A 开源库
* 小车电机控制
*********************************************************************************************************************/

#include "car_control.h"

#define CAR_MAX_DUTY                (60)

// 电机输出增益百分比，100 表示不补偿。
// 用相同 PWM 测完编码器计数后，后续主要调这里。
#define MOTOR1_GAIN_PERCENT         (100)
#define MOTOR2_GAIN_PERCENT         (112)
#define MOTOR3_GAIN_PERCENT         (92)
#define MOTOR4_GAIN_PERCENT         (96)

// 电机实际安装位置：
//       前
// 左 MOTOR4    MOTOR1 右
//     ●          ●
//     │          │
//     │          │
//     ●          ●
// 左 MOTOR2    MOTOR3 右
//       后
// 前轮方向已在最终输出时取反，用来匹配后轮的前进方向。
// 左轮方向已在最终输出时取反，用来匹配右轮的前进方向。

#define MOTOR1_DIR                  (C9)
#define MOTOR1_PWM                  (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR                  (C7)
#define MOTOR2_PWM                  (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR                  (D2)
#define MOTOR3_PWM                  (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR                  (C10)
#define MOTOR4_PWM                  (PWM2_MODULE2_CHB_C11)

static int16 abs_int16(int16 value)
{
    return (value >= 0) ? value : -value;
}

static int8 limit_motor_duty(int16 duty)
{
    if(duty > CAR_MAX_DUTY)
    {
        duty = CAR_MAX_DUTY;
    }
    else if(duty < -CAR_MAX_DUTY)
    {
        duty = -CAR_MAX_DUTY;
    }

    return (int8)duty;
}

static int16 motor_apply_gain(int16 duty, uint16 gain_percent)
{
    return (int16)((int32)duty * gain_percent / 100);
}

static void motor_set_one(int8 duty, gpio_pin_enum dir_pin, pwm_channel_enum pwm_pin)
{
    if(duty <= 0)
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
        duty = -duty;
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_LOW);
    }

    pwm_set_duty(pwm_pin, duty * (PWM_DUTY_MAX / 100));
}

void car_init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, 17000, 0);

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, 17000, 0);

    car_stop();
}

void car_stop(void)
{
    motor_set_one(0, MOTOR1_DIR, MOTOR1_PWM);
    motor_set_one(0, MOTOR2_DIR, MOTOR2_PWM);
    motor_set_one(0, MOTOR3_DIR, MOTOR3_PWM);
    motor_set_one(0, MOTOR4_DIR, MOTOR4_PWM);
}

void car_move_xy(int8 x, int8 y)
{
    int16 motor1_duty;
    int16 motor2_duty;
    int16 motor3_duty;
    int16 motor4_duty;
    int16 max_duty;

    x = limit_motor_duty(x);
    y = limit_motor_duty(y);

    // x > 0 向右，x < 0 向左，y > 0 向前，y < 0 向后。
    // 向右时：MOTOR1/MOTOR3 反转，MOTOR2/MOTOR4 正转。
    motor1_duty = y - x;  // 右前轮
    motor2_duty = y + x;  // 左后轮
    motor3_duty = y - x;  // 右后轮
    motor4_duty = y + x;  // 左前轮

    motor1_duty = motor_apply_gain(motor1_duty, MOTOR1_GAIN_PERCENT);
    motor2_duty = motor_apply_gain(motor2_duty, MOTOR2_GAIN_PERCENT);
    motor3_duty = motor_apply_gain(motor3_duty, MOTOR3_GAIN_PERCENT);
    motor4_duty = motor_apply_gain(motor4_duty, MOTOR4_GAIN_PERCENT);

    max_duty = abs_int16(motor1_duty);
    if(max_duty < abs_int16(motor2_duty)) max_duty = abs_int16(motor2_duty);
    if(max_duty < abs_int16(motor3_duty)) max_duty = abs_int16(motor3_duty);
    if(max_duty < abs_int16(motor4_duty)) max_duty = abs_int16(motor4_duty);

    if(max_duty > CAR_MAX_DUTY)
    {
        motor1_duty = motor1_duty * CAR_MAX_DUTY / max_duty;
        motor2_duty = motor2_duty * CAR_MAX_DUTY / max_duty;
        motor3_duty = motor3_duty * CAR_MAX_DUTY / max_duty;
        motor4_duty = motor4_duty * CAR_MAX_DUTY / max_duty;
    }

    motor_set_one((int8)motor1_duty, MOTOR1_DIR, MOTOR1_PWM);
    motor_set_one((int8)motor2_duty, MOTOR2_DIR, MOTOR2_PWM);
    motor_set_one((int8)motor3_duty, MOTOR3_DIR, MOTOR3_PWM);
    motor_set_one((int8)motor4_duty, MOTOR4_DIR, MOTOR4_PWM);
}
