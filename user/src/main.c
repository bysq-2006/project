#include "zf_common_headfile.h"
#include "car_control.h"
#include "screen_print/screen_print.h"

#define DEFAULT_MOTOR_DUTY          (20)
#define DUTY_STEP                   (5)
#define DUTY_MAX                    (90)
#define DUTY_MIN                    (0)

// S5-1 控制车轮是否输出。
#define SWITCH_RUN                  (C26)
// S5-2 控制当前调参的轮胎组。
#define SWITCH_GROUP                (C27)

// C12/C13 调当前组右侧轮胎。
#define KEY_MOTOR_UP                (C12)
#define KEY_MOTOR_DOWN              (C13)
// C14/C15 调当前组左侧轮胎。
#define KEY_PAIR_UP                 (C14)
#define KEY_PAIR_DOWN               (C15)

#define MOTOR1_DIR                  (C9)
#define MOTOR1_PWM                  (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR                  (C7)
#define MOTOR2_PWM                  (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR                  (D2)
#define MOTOR3_PWM                  (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR                  (C10)
#define MOTOR4_PWM                  (PWM2_MODULE2_CHB_C11)

static int8 motor_duty[4] =
{
    DEFAULT_MOTOR_DUTY,
    DEFAULT_MOTOR_DUTY,
    DEFAULT_MOTOR_DUTY,
    DEFAULT_MOTOR_DUTY,
};

static uint8 key_last_level[4] = {GPIO_HIGH, GPIO_HIGH, GPIO_HIGH, GPIO_HIGH};
static const gpio_pin_enum key_pin[4] =
{
    KEY_MOTOR_UP,
    KEY_MOTOR_DOWN,
    KEY_PAIR_UP,
    KEY_PAIR_DOWN,
};

static uint8 switch_is_on(gpio_pin_enum pin)
{
    // 拨码开关使用上拉输入。
    // 拨到 ON 时读到低电平。
    return (GPIO_LOW == gpio_get_level(pin));
}

static void adjust_motor_duty(uint8 motor_index, int8 delta)
{
    int16 duty;

    if((1 <= motor_index) && (4 >= motor_index))
    {
        duty = motor_duty[motor_index - 1] + delta;

        if(duty > DUTY_MAX)
        {
            duty = DUTY_MAX;
        }
        else if(duty < DUTY_MIN)
        {
            duty = DUTY_MIN;
        }

        motor_duty[motor_index - 1] = (int8)duty;
    }
}

static uint8 key_pressed_once(uint8 key_index)
{
    uint8 now_level = gpio_get_level(key_pin[key_index]);
    uint8 pressed = ((GPIO_LOW == now_level) && (GPIO_HIGH == key_last_level[key_index]));

    // 只在刚按下的瞬间触发一次。
    // 这样可以避免按住时连续加减。
    key_last_level[key_index] = now_level;
    return pressed;
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

static void motor_output_all(uint8 run_enable)
{
    // 向左平移逻辑：M1 反转，M2 正转，M3 正转，M4 反转。
    int8 motor1_command = (int8)-motor_duty[0];
    int8 motor2_command = motor_duty[1];
    int8 motor3_command = motor_duty[2];
    int8 motor4_command = (int8)-motor_duty[3];

    int8 motor1_output = run_enable ? motor1_command : 0;
    int8 motor2_output = run_enable ? motor2_command : 0;
    int8 motor3_output = run_enable ? motor3_command : 0;
    int8 motor4_output = run_enable ? motor4_command : 0;

    motor_set_one(motor1_output, MOTOR1_DIR, MOTOR1_PWM);
    motor_set_one(motor2_output, MOTOR2_DIR, MOTOR2_PWM);
    motor_set_one(motor3_output, MOTOR3_DIR, MOTOR3_PWM);
    motor_set_one(motor4_output, MOTOR4_DIR, MOTOR4_PWM);

    // 屏幕始终显示当前设定值。
    // 所以车停着也能看到调参结果。
    screen_print_motor_duty(motor1_command, motor2_command, motor3_command, motor4_command);
}

static void tune_motor_duty(void)
{
    // S5-2 关闭时调 M4/M3。
    // S5-2 打开时调 M2/M1。
    uint8 selected_motor = switch_is_on(SWITCH_GROUP) ? 2 : 4;
    uint8 paired_motor = switch_is_on(SWITCH_GROUP) ? 1 : 3;

    if(key_pressed_once(0))
    {
        adjust_motor_duty(selected_motor, DUTY_STEP);
    }
    if(key_pressed_once(1))
    {
        adjust_motor_duty(selected_motor, -DUTY_STEP);
    }
    if(key_pressed_once(2))
    {
        adjust_motor_duty(paired_motor, DUTY_STEP);
    }
    if(key_pressed_once(3))
    {
        adjust_motor_duty(paired_motor, -DUTY_STEP);
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(300);

    car_init();
    system_delay_ms(100);

    gpio_init(SWITCH_RUN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH_GROUP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_MOTOR_UP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_MOTOR_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_PAIR_UP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_PAIR_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);

    while(1)
    {
        tune_motor_duty();
        motor_output_all(switch_is_on(SWITCH_RUN));
        system_delay_ms(50);
    }
}
