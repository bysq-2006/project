/*********************************************************************************************************************
* RT1064DVL6A 开源库
* 小车电机控制
*********************************************************************************************************************/

#include "car_control.h"
#include "../car_params.h"

// 可调参数在 car_params.h：
// CAR_MAX_DUTY, CAR_MOTOR_PWM_FREQ_HZ,
// MOTOR1_GAIN_PERCENT, MOTOR2_GAIN_PERCENT, MOTOR3_GAIN_PERCENT, MOTOR4_GAIN_PERCENT。

// 电机输出增益百分比，100 表示不补偿，200表示两倍电压。
// 用相同 PWM 测完编码器计数后，后续主要调这里。

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
        gpio_set_level(dir_pin, GPIO_LOW);
        duty = -duty;
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
    }

    pwm_set_duty(pwm_pin, duty * (PWM_DUTY_MAX / 100));
}

void car_init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, CAR_MOTOR_PWM_FREQ_HZ, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, CAR_MOTOR_PWM_FREQ_HZ, 0);

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, CAR_MOTOR_PWM_FREQ_HZ, 0);

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, CAR_MOTOR_PWM_FREQ_HZ, 0);

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
    car_move_xyw(x, y, 0);
}

void car_move_xyw(int8 x, int8 y, int8 w)
{
    int16 motor1_duty;
    int16 motor2_duty;
    int16 motor3_duty;
    int16 motor4_duty;
    int16 max_duty;

    // 混合 x/y 平移和自转之前，先限制输入速度范围。
    x = limit_motor_duty(x);
    y = limit_motor_duty(y);
    w = limit_motor_duty(w);

    // x > 0 向右，x < 0 向左，y > 0 向前，y < 0 向后。
    // 向右时：MOTOR1/MOTOR3 反转，MOTOR2/MOTOR4 正转。
    // 电机实际安装位置：
    //       前
    // 左 MOTOR1    MOTOR2 右
    //     ●          ●
    //     │          │
    //     │          │
    //     ●          ●
    // 左 MOTOR3    MOTOR4 右
    //       后
    // 麦克纳姆/全向轮混控：x、y、w 分别保持独立控制轴。
    motor1_duty = y + x - w;
    motor2_duty = y - x + w;
    motor3_duty = y + x + w;
    motor4_duty = y - x - w;

    motor1_duty = motor_apply_gain(motor1_duty, MOTOR1_GAIN_PERCENT);
    motor2_duty = motor_apply_gain(motor2_duty, MOTOR2_GAIN_PERCENT);
    motor3_duty = motor_apply_gain(motor3_duty, MOTOR3_GAIN_PERCENT);
    motor4_duty = motor_apply_gain(motor4_duty, MOTOR4_GAIN_PERCENT);

    // 找到绝对值最大的电机占空比，若超过最大值则按比例缩放所有电机占空比。
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
    motor_set_one((int8)-motor3_duty, MOTOR3_DIR, MOTOR3_PWM);
    motor_set_one((int8)-motor4_duty, MOTOR4_DIR, MOTOR4_PWM);
}
