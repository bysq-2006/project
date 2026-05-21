/*********************************************************************************************************************
* RT1064DVL6A Open Source Library
* car_control.c
*********************************************************************************************************************/

#include "car_control.h"

#define CAR_MAX_DUTY                (20)

// Motor position mapping measured on the car:
// MOTOR1 = front right
// MOTOR2 = rear left
// MOTOR3 = rear right
// MOTOR4 = front left
// Front motors are reversed to match the rear wheel forward direction.
// Left motors are reversed to match the right wheel forward direction.

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

void car_motor_test(uint8 motor_index, int8 duty)
{
    car_stop();
    duty = limit_motor_duty(duty);

    switch(motor_index)
    {
        case 1:
            motor_set_one(-duty, MOTOR1_DIR, MOTOR1_PWM);
            break;

        case 2:
            motor_set_one(-duty, MOTOR2_DIR, MOTOR2_PWM);
            break;

        case 3:
            motor_set_one(duty, MOTOR3_DIR, MOTOR3_PWM);
            break;

        case 4:
            motor_set_one(duty, MOTOR4_DIR, MOTOR4_PWM);
            break;

        default:
            break;
    }
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

    // x > 0: right, x < 0: left, y > 0: forward, y < 0: backward.
    motor1_duty = y - x;  // front right
    motor2_duty = y + x;  // rear left
    motor3_duty = y - x;  // rear right
    motor4_duty = y + x;  // front left

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

    motor_set_one((int8)-motor1_duty, MOTOR1_DIR, MOTOR1_PWM);
    motor_set_one((int8)-motor2_duty, MOTOR2_DIR, MOTOR2_PWM);
    motor_set_one((int8) motor3_duty, MOTOR3_DIR, MOTOR3_PWM);
    motor_set_one((int8) motor4_duty, MOTOR4_DIR, MOTOR4_PWM);
}
