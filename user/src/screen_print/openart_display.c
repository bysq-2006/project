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


static char angle_to_car_char(uint16 angle10)
{
    static const char direction_chars[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};
    uint16 sector;

    sector = (uint16)(((angle10 % 3600) + 225) / 450);
    if(sector >= 8)
    {
        sector = 0;
    }

    return direction_chars[sector];
}


static uint8 get_car_grid_position(uint8 *car_col, uint8 *car_row)
{
    int32 col;
    int32 row;

    if((!openart_pose.valid) || (!openart_map.valid) ||
       (openart_map.cols == 0) || (openart_map.rows == 0) ||
       (openart_map.width10 == 0) || (openart_map.height10 == 0))
    {
        return 0;
    }

    col = ((int32)openart_pose.x10 * openart_map.cols) / openart_map.width10;
    row = ((int32)openart_pose.y10 * openart_map.rows) / openart_map.height10;

    if(col < 0)
    {
        col = 0;
    }
    else if(col >= openart_map.cols)
    {
        col = openart_map.cols - 1;
    }

    if(row < 0)
    {
        row = 0;
    }
    else if(row >= openart_map.rows)
    {
        row = openart_map.rows - 1;
    }

    *car_col = (uint8)col;
    *car_row = (uint8)row;
    return 1;
}


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


static void display_uart_status_rows(void)
{
    char buffer[OPENART_DISPLAY_BUFFER_SIZE];

    sprintf(buffer, "RX:%u PK:%u",
            openart_uart_status.rx_bytes,
            openart_uart_status.packet_count);
    screen_print_line(4, buffer);

    sprintf(buffer, "PO:%u MP:%u",
            openart_uart_status.pose_packets,
            openart_uart_status.map_packets);
    screen_print_line(5, buffer);

    sprintf(buffer, "CK:%u FM:%u OV:%u",
            openart_uart_status.checksum_errors,
            openart_uart_status.format_errors,
            openart_uart_status.rx_overflows);
    screen_print_line(6, buffer);
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
    uint8 car_col;
    uint8 car_row;

    if(openart_map.valid)
    {
        format_fixed10(width_text, (int16)openart_map.width10);
        format_fixed10(height_text, (int16)openart_map.height10);
        sprintf(buffer, "Map %dx%d W:%s", openart_map.cols, openart_map.rows, width_text);
        screen_print_line(2, buffer);
        if(get_car_grid_position(&car_col, &car_row))
        {
            sprintf(buffer, "H:%s C:%02d,%02d P:%d", height_text, car_col, car_row, map_page);
        }
        else
        {
            sprintf(buffer, "H:%s Seq:%d P:%d", height_text, openart_map.seq, map_page);
        }
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
    uint8 car_col;
    uint8 car_row;
    uint8 has_car;
    uint16 cell_index;
    char buffer[OPENART_DISPLAY_BUFFER_SIZE];

    if(!openart_map.valid)
    {
        display_uart_status_rows();
        return;
    }

    has_car = get_car_grid_position(&car_col, &car_row);
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
        if((has_car) && (row == car_row) && (car_col < col))
        {
            buffer[3 + car_col] = angle_to_car_char(openart_pose.angle10);
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
    uint8 car_col;
    uint8 car_row;

    if(openart_map.valid && openart_map.rows > 0)
    {
        page_count = (uint8)((openart_map.rows + 2) / 3);
        if(page_count == 0)
        {
            page_count = 1;
        }

        if(get_car_grid_position(&car_col, &car_row))
        {
            display_tick = 0;
            map_page = (uint8)(car_row / 3);
            if(map_page >= page_count)
            {
                map_page = (uint8)(page_count - 1);
            }
        }
        else
        {
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
