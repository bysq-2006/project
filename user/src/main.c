#include "zf_common_headfile.h"
#include "car_control/car_control.h"
#include "car_control/heading_control.h"
#include "gyro_z_angle/gyro_z_angle.h"
#include "screen_print/screen_print.h"
#include <stdio.h>

#define MAIN_CONTROL_UPDATE_MS      (20)
#define MAIN_HEADING_HOLD_ANGLE     (90)

static void main_format_fixed3(char *buffer, int32 value1000)
{
    int32 abs_value;

    abs_value = value1000;
    if(value1000 < 0)
    {
        abs_value = -value1000;
        sprintf(buffer, "-%ld.%03ld", (long)(abs_value / 1000), (long)(abs_value % 1000));
    }
    else
    {
        sprintf(buffer, "%ld.%03ld", (long)(abs_value / 1000), (long)(abs_value % 1000));
    }
}

static void main_display_heading_hold(void)
{
    char line[64];
    char angle_text[20];
    char error_text[20];
    int32 angle1000;
    int32 error1000;

    angle1000 = (int32)(gyro_z_angle_get_output() * 1000.0f);
    error1000 = angle1000 - ((int32)MAIN_HEADING_HOLD_ANGLE * 1000);

    main_format_fixed3(angle_text, angle1000);
    main_format_fixed3(error_text, error1000);

    screen_print_line(0, "HEADING HOLD");
    sprintf(line, "Target:%d deg", MAIN_HEADING_HOLD_ANGLE);
    screen_print_line(1, line);
    sprintf(line, "Angle:%s", angle_text);
    screen_print_line(2, line);
    sprintf(line, "Error:%s", error_text);
    screen_print_line(3, line);
    sprintf(line, "RAW:%d", gyro_z_angle_get_raw());
    screen_print_line(4, line);
    sprintf(line, "INT:%ld", (long)gyro_z_angle_get_integral());
    screen_print_line(5, line);
    screen_print_line(6, "X=0 Y=0");
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    car_init();
    heading_sensor_init();
    gyro_z_angle_init();
    screen_print_init();

    gyro_z_angle_reset();

    while(1)
    {
        gyro_z_angle_update(MAIN_CONTROL_UPDATE_MS);
        heading_sensor_update(0, 0, MAIN_HEADING_HOLD_ANGLE);
        main_display_heading_hold();
        system_delay_ms(MAIN_CONTROL_UPDATE_MS);
    }
}
