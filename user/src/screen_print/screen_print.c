/*********************************************************************************************************************
* Screen print helper
*********************************************************************************************************************/

#include "screen_print.h"
#include <stdio.h>

#define SCREEN_PRINT_LINE_COUNT     (7)
#define SCREEN_PRINT_LINE_HEIGHT    (24)
#define SCREEN_PRINT_LINE_WIDTH     (240)
#define SCREEN_PRINT_BUFFER_SIZE    (40)

static uint16 screen_print_page_id;
static uint16 screen_print_label_id[SCREEN_PRINT_LINE_COUNT];
static uint8 screen_print_ready;

void screen_print_init(void)
{
    uint8 i;

    screen_print_page_id = ips200pro_init("Debug", IPS200PRO_TITLE_BOTTOM, 30);
    ips200pro_set_default_font(FONT_SIZE_16);
    ips200pro_set_direction(IPS200PRO_PORTRAIT);
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);
    ips200pro_page_switch(screen_print_page_id, PAGE_ANIM_OFF);

    for(i = 0; i < SCREEN_PRINT_LINE_COUNT; i ++)
    {
        screen_print_label_id[i] = ips200pro_label_create(0, i * SCREEN_PRINT_LINE_HEIGHT, SCREEN_PRINT_LINE_WIDTH, SCREEN_PRINT_LINE_HEIGHT);
        ips200pro_set_color(screen_print_label_id[i], COLOR_FOREGROUND, RGB565_WHITE);
        ips200pro_set_color(screen_print_label_id[i], COLOR_BACKGROUND, RGB565_BLACK);
    }

    ips200pro_set_backlight(255);
    screen_print_ready = 1;
}

void screen_print_string(const char *str)
{
    screen_print_line(0, str);
}

void screen_print_line(uint8 line, const char *str)
{
    if((screen_print_ready) && (line < SCREEN_PRINT_LINE_COUNT))
    {
        ips200pro_label_show_string(screen_print_label_id[line], (str == NULL) ? "" : str);
    }
}

void screen_print_motor_duty(int16 motor1_duty, int16 motor2_duty, int16 motor3_duty, int16 motor4_duty)
{
    char buffer[SCREEN_PRINT_BUFFER_SIZE];

    sprintf(buffer, "M1:%4d%% M2:%4d%%", motor1_duty, motor2_duty);
    screen_print_line(0, buffer);

    sprintf(buffer, "M3:%4d%% M4:%4d%%", motor3_duty, motor4_duty);
    screen_print_line(1, buffer);

    screen_print_line(2, "      Front");
    screen_print_line(3, "Left M1    M2 Right");
    screen_print_line(4, "     |      |");
    screen_print_line(5, "Left M3    M4 Right");
    screen_print_line(6, "       Rear");
}
