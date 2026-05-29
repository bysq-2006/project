#include "openart_display.h"
#include "../openart_uart/openart_uart.h"
#include "screen_print.h"
#include <stdio.h>


#define OPENART_DISPLAY_MAP_LINES       (4)
#define OPENART_DISPLAY_LOOP_DELAY_MS   (50)
#define OPENART_DISPLAY_PAGE_TICKS      (20)
#define OPENART_DISPLAY_BUFFER_SIZE     (40)


static uint16 display_tick;
static uint8 map_page;


static void format_fixed10(char *buffer, int16 value10)
{
    int16 abs_value = value10;

    if(value10 < 0)
    {
        abs_value = (int16)-value10;
        sprintf(buffer, "-%d.%d", abs_value / 10, abs_value % 10);
    }
    else
    {
        sprintf(buffer, "%d.%d", abs_value / 10, abs_value % 10);
    }
}


static char cell_to_char(uint8 cell)
{
    switch(cell)
    {
        case OPENART_CELL_BACKGROUND:
            return '-';
        case OPENART_CELL_WALL:
            return '#';
        case OPENART_CELL_GOAL:
            return '+';
        case OPENART_CELL_YELLOW_BOX:
            return '$';
        default:
            return '?';
    }
}


static void display_pose(void)
{
    char x_text[12];
    char y_text[12];
    char angle_text[12];
    char buffer[OPENART_DISPLAY_BUFFER_SIZE];

    if(openart_pose.valid)
    {
        format_fixed10(x_text, openart_pose.x10);
        format_fixed10(y_text, openart_pose.y10);
        format_fixed10(angle_text, (int16)openart_pose.angle10);

        sprintf(buffer, "Car X:%s Y:%s", x_text, y_text);
        screen_print_line(0, buffer);

        sprintf(buffer, "Ang:%s Seq:%d", angle_text, openart_pose.seq);
        screen_print_line(1, buffer);
    }
    else
    {
        screen_print_line(0, "Car invalid");
        sprintf(buffer, "Pose Seq:%d", openart_pose.seq);
        screen_print_line(1, buffer);
    }
}


static void display_map_header(void)
{
    char width_text[12];
    char height_text[12];
    char buffer[OPENART_DISPLAY_BUFFER_SIZE];

    if(openart_map.valid)
    {
        format_fixed10(width_text, (int16)openart_map.width10);
        format_fixed10(height_text, (int16)openart_map.height10);
        sprintf(buffer, "Map %dx%d W:%s", openart_map.cols, openart_map.rows, width_text);
        screen_print_line(2, buffer);
        sprintf(buffer, "H:%s Seq:%d P:%d", height_text, openart_map.seq, map_page);
        screen_print_line(3, buffer);
    }
    else
    {
        screen_print_line(2, "Map invalid");
        sprintf(buffer, "Map Seq:%d", openart_map.seq);
        screen_print_line(3, buffer);
    }
}


static void display_map_rows(void)
{
    uint8 line;
    uint8 col;
    uint8 row;
    uint8 start_row;
    uint16 cell_index;
    char buffer[OPENART_DISPLAY_BUFFER_SIZE];

    if(!openart_map.valid)
    {
        screen_print_line(4, "");
        screen_print_line(5, "");
        screen_print_line(6, "");
        return;
    }

    start_row = (uint8)(map_page * 3);
    for(line = 0; line < 3; line++)
    {
        row = (uint8)(start_row + line);
        if(row >= openart_map.rows)
        {
            screen_print_line((uint8)(4 + line), "");
            continue;
        }

        sprintf(buffer, "%02d:", row);
        for(col = 0; (col < openart_map.cols) && (col < OPENART_MAP_COLS_MAX); col++)
        {
            cell_index = (uint16)row * openart_map.cols + col;
            buffer[3 + col] = cell_to_char(openart_map.cells[cell_index]);
        }
        buffer[3 + col] = '\0';
        screen_print_line((uint8)(4 + line), buffer);
    }
}


void openart_display_init(void)
{
    display_tick = 0;
    map_page = 0;
    screen_print_init();
    screen_print_line(0, "Wait OpenART...");
    screen_print_line(1, "");
    screen_print_line(2, "");
    screen_print_line(3, "");
    screen_print_line(4, "");
    screen_print_line(5, "");
    screen_print_line(6, "");
}


void openart_display_update(void)
{
    uint8 page_count;

    if(openart_map.valid && openart_map.rows > 0)
    {
        page_count = (uint8)((openart_map.rows + 2) / 3);
        if(page_count == 0)
        {
            page_count = 1;
        }
        display_tick++;
        if(display_tick >= OPENART_DISPLAY_PAGE_TICKS)
        {
            display_tick = 0;
            map_page++;
            if(map_page >= page_count)
            {
                map_page = 0;
            }
        }
    }
    else
    {
        display_tick = 0;
        map_page = 0;
    }

    display_pose();
    display_map_header();
    display_map_rows();
}
