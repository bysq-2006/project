#include "openart_display.h"
#include "../openart_uart/openart_uart.h"
#include "../main_control/main_control.h"
#include <math.h>
#include <stdio.h>


#define RGB565(r, g, b) ((uint16)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) >> 3) & 0x1F)))

#define MAP_CANVAS_W                (240)
#define MAP_CANVAS_H                (240)
#define STATUS_CANVAS_W             (240)
#define STATUS_CANVAS_H             (50)
#define MAP_COLS_VIEW               (12)
#define MAP_ROWS_VIEW               (16)
#define MAP_CELL_W                  (MAP_CANVAS_W / MAP_COLS_VIEW)
#define MAP_CELL_H                  (MAP_CANVAS_H / MAP_ROWS_VIEW)

#define SPEED_BAR_X                 (10)
#define SPEED_BAR_Y                 (8)
#define SPEED_BAR_W                 (150)
#define SPEED_BAR_H                 (12)

#define INFO_BOX_X                  (170)
#define INFO_BOX_Y                  (8)
#define INFO_BOX_W                  (60)
#define INFO_BOX_H                  (34)

#define SPEED_MAX_PX_S              (240U)
#define CAR_HEAD_LEN                (7.0f)
#define CAR_HEAD_ANGLE              (0.55f)

#define MAP_BG_COLOR                RGB565(18, 18, 18)
#define MAP_GRID_COLOR              RGB565(36, 36, 36)
#define MAP_WALL_COLOR              RGB565(240, 240, 240)
#define MAP_GOAL_COLOR              RGB565(180, 60, 255)
#define MAP_BOX_COLOR               RGB565(240, 180, 0)
#define MAP_UNKNOWN_COLOR           RGB565(96, 96, 192)
#define MAP_CAR_COLOR               RGB565(255, 64, 64)
#define MAP_CAR_HEAD_COLOR          RGB565(0, 255, 255)
#define MAP_INVALID_BORDER          RGB565(255, 64, 64)

#define STATUS_BG_COLOR             RGB565(0, 0, 0)
#define STATUS_BORDER_COLOR         RGB565(80, 80, 80)
#define SPEED_BAR_BG_COLOR          RGB565(32, 32, 32)
#define SPEED_BAR_LOW_COLOR         RGB565(0, 180, 255)
#define SPEED_BAR_HIGH_COLOR        RGB565(255, 80, 0)
#define STATUS_VALID_COLOR          RGB565(0, 180, 255)
#define STATUS_INVALID_COLOR        RGB565(255, 64, 64)
#define STATUS_DEBUG_LINE_COUNT     (2)
#define STATUS_DEBUG_LINE_HEIGHT    (25)
#define STATUS_DEBUG_BUFFER_SIZE    (64)


static uint16 map_image_id;
static uint16 status_image_id;
static uint16 status_debug_label_id[STATUS_DEBUG_LINE_COUNT];
static uint16 map_canvas[MAP_CANVAS_W * MAP_CANVAS_H];
static uint16 status_canvas[STATUS_CANVAS_W * STATUS_CANVAS_H];
static uint32 speed_last_ms;
static int32 speed_last_x;
static int32 speed_last_y;
static uint8 speed_last_valid;
static uint16 speed_px_s;


static int16 abs16(int16 value)
{
    return (value < 0) ? (int16)-value : value;
}


static uint16 map_cell_color(uint8 cell)
{
    switch(cell)
    {
        case OPENART_CELL_BACKGROUND:
            return MAP_BG_COLOR;
        case OPENART_CELL_WALL:
            return MAP_WALL_COLOR;
        case OPENART_CELL_GOAL:
            return MAP_GOAL_COLOR;
        case OPENART_CELL_YELLOW_BOX:
            return MAP_BOX_COLOR;
        default:
            return MAP_UNKNOWN_COLOR;
    }
}


static void canvas_fill(uint16 *canvas, uint16 width, uint16 height, uint16 color)
{
    uint32 total;
    uint32 i;

    total = (uint32)width * height;
    for(i = 0; i < total; i++)
    {
        canvas[i] = color;
    }
}


static void canvas_set_pixel(uint16 *canvas, uint16 width, uint16 height, int16 x, int16 y, uint16 color)
{
    if((x < 0) || (y < 0) || (x >= (int16)width) || (y >= (int16)height))
    {
        return;
    }

    canvas[(uint32)y * width + (uint16)x] = color;
}


static void canvas_fill_rect(uint16 *canvas, uint16 width, uint16 height,
                             int16 x, int16 y, uint16 rect_w, uint16 rect_h, uint16 color)
{
    int16 x0;
    int16 y0;
    int16 x1;
    int16 y1;
    int16 row;
    int16 col;

    if((rect_w == 0U) || (rect_h == 0U))
    {
        return;
    }

    x0 = x;
    y0 = y;
    x1 = (int16)(x + (int16)rect_w - 1);
    y1 = (int16)(y + (int16)rect_h - 1);

    if(x0 < 0)
    {
        x0 = 0;
    }
    if(y0 < 0)
    {
        y0 = 0;
    }
    if(x1 >= (int16)width)
    {
        x1 = (int16)width - 1;
    }
    if(y1 >= (int16)height)
    {
        y1 = (int16)height - 1;
    }

    for(row = y0; row <= y1; row++)
    {
        for(col = x0; col <= x1; col++)
        {
            canvas[(uint32)row * width + (uint16)col] = color;
        }
    }
}


static void canvas_draw_rect(uint16 *canvas, uint16 width, uint16 height,
                             int16 x, int16 y, uint16 rect_w, uint16 rect_h, uint16 color)
{
    int16 right;
    int16 bottom;
    int16 i;

    if((rect_w == 0U) || (rect_h == 0U))
    {
        return;
    }

    right = (int16)(x + (int16)rect_w - 1);
    bottom = (int16)(y + (int16)rect_h - 1);

    for(i = x; i <= right; i++)
    {
        canvas_set_pixel(canvas, width, height, i, y, color);
        canvas_set_pixel(canvas, width, height, i, bottom, color);
    }
    for(i = y; i <= bottom; i++)
    {
        canvas_set_pixel(canvas, width, height, x, i, color);
        canvas_set_pixel(canvas, width, height, right, i, color);
    }
}


static void canvas_draw_line(uint16 *canvas, uint16 width, uint16 height,
                             int16 x0, int16 y0, int16 x1, int16 y1, uint16 color)
{
    int16 dx;
    int16 sx;
    int16 dy;
    int16 sy;
    int16 err;
    int16 e2;

    dx = abs16((int16)(x1 - x0));
    sx = (x0 < x1) ? 1 : -1;
    dy = (int16)-abs16((int16)(y1 - y0));
    sy = (y0 < y1) ? 1 : -1;
    err = (int16)(dx + dy);

    while(1)
    {
        canvas_set_pixel(canvas, width, height, x0, y0, color);
        if((x0 == x1) && (y0 == y1))
        {
            break;
        }

        e2 = (int16)(2 * err);
        if(e2 >= dy)
        {
            err = (int16)(err + dy);
            x0 = (int16)(x0 + sx);
        }
        if(e2 <= dx)
        {
            err = (int16)(err + dx);
            y0 = (int16)(y0 + sy);
        }
    }
}


static uint8 get_car_canvas_position(int16 *car_x, int16 *car_y)
{
    if((!openart_pose.valid) || (!openart_map.valid) ||
       (openart_map.width10 == 0) || (openart_map.height10 == 0))
    {
        return 0;
    }

    *car_x = (int16)(((int32)openart_pose.x10 * MAP_CANVAS_W) / openart_map.width10);
    *car_y = (int16)(((int32)openart_pose.y10 * MAP_CANVAS_H) / openart_map.height10);

    if(*car_x < 0)
    {
        *car_x = 0;
    }
    else if(*car_x >= MAP_CANVAS_W)
    {
        *car_x = MAP_CANVAS_W - 1;
    }

    if(*car_y < 0)
    {
        *car_y = 0;
    }
    else if(*car_y >= MAP_CANVAS_H)
    {
        *car_y = MAP_CANVAS_H - 1;
    }

    return 1;
}


static uint8 get_car_cell_position(uint8 *car_col, uint8 *car_row)
{
    int16 car_x;
    int16 car_y;

    if(!get_car_canvas_position(&car_x, &car_y))
    {
        return 0;
    }

    *car_col = (uint8)(car_x / MAP_CELL_W);
    *car_row = (uint8)(car_y / MAP_CELL_H);

    if(*car_col >= MAP_COLS_VIEW)
    {
        *car_col = MAP_COLS_VIEW - 1;
    }
    if(*car_row >= MAP_ROWS_VIEW)
    {
        *car_row = MAP_ROWS_VIEW - 1;
    }

    return 1;
}


static uint16 estimate_speed_px_s(int16 car_x, int16 car_y)
{
    uint32 now_ms;
    uint32 dt_ms;
    uint32 dist_px;
    int32 dx;
    int32 dy;
    uint32 speed;

    now_ms = OSA_TimeGetMsec();
    if(speed_last_valid && (now_ms > speed_last_ms))
    {
        dt_ms = now_ms - speed_last_ms;
        dx = car_x - speed_last_x;
        dy = car_y - speed_last_y;
        dist_px = (uint32)abs16((int16)dx) + (uint32)abs16((int16)dy);
        speed = (dist_px * 1000U) / dt_ms;
        if(speed > 0xFFFFU)
        {
            speed = 0xFFFFU;
        }
        speed_px_s = (uint16)speed;
    }

    speed_last_ms = now_ms;
    speed_last_x = car_x;
    speed_last_y = car_y;
    speed_last_valid = 1;
    return speed_px_s;
}


static void draw_speed_bar(uint16 *canvas, uint16 width, uint16 height, uint16 speed)
{
    uint16 fill_w;
    uint16 color;

    canvas_draw_rect(canvas, width, height, SPEED_BAR_X, SPEED_BAR_Y, SPEED_BAR_W, SPEED_BAR_H, STATUS_BORDER_COLOR);
    canvas_fill_rect(canvas, width, height,
                     SPEED_BAR_X + 1, SPEED_BAR_Y + 1,
                     SPEED_BAR_W - 2, SPEED_BAR_H - 2, SPEED_BAR_BG_COLOR);

    if(speed >= SPEED_MAX_PX_S)
    {
        fill_w = SPEED_BAR_W - 2;
        color = SPEED_BAR_HIGH_COLOR;
    }
    else
    {
        fill_w = (uint16)(((uint32)(SPEED_BAR_W - 2) * speed) / SPEED_MAX_PX_S);
        color = (speed > (SPEED_MAX_PX_S / 2U)) ? SPEED_BAR_HIGH_COLOR : SPEED_BAR_LOW_COLOR;
    }

    if(fill_w > 0U)
    {
        canvas_fill_rect(canvas, width, height,
                         SPEED_BAR_X + 1, SPEED_BAR_Y + 1,
                         fill_w, SPEED_BAR_H - 2, color);
    }
}


static void draw_position_panel(uint16 *canvas, uint16 width, uint16 height)
{
    int16 panel_x;
    int16 panel_y;
    int16 box_x;
    int16 box_y;
    int16 box_w;
    int16 box_h;
    int16 car_x;
    int16 car_y;
    int16 dot_x;
    int16 dot_y;
    float angle_rad;
    int16 tip_x;
    int16 tip_y;
    int16 head1_x;
    int16 head1_y;
    int16 head2_x;
    int16 head2_y;
    uint16 border_color;

    panel_x = INFO_BOX_X;
    panel_y = INFO_BOX_Y;
    box_x = panel_x + 4;
    box_y = panel_y + 4;
    box_w = INFO_BOX_W - 8;
    box_h = INFO_BOX_H - 8;

    if(get_car_canvas_position(&car_x, &car_y))
    {
        border_color = STATUS_VALID_COLOR;
        canvas_draw_rect(canvas, width, height, panel_x, panel_y, INFO_BOX_W, INFO_BOX_H, border_color);
        canvas_fill_rect(canvas, width, height, box_x, box_y, (uint16)box_w, (uint16)box_h, STATUS_BG_COLOR);

        dot_x = (int16)(box_x + (int16)(((int32)car_x * (box_w - 2)) / (MAP_CANVAS_W - 1)) + 1);
        dot_y = (int16)(box_y + (int16)(((int32)car_y * (box_h - 2)) / (MAP_CANVAS_H - 1)) + 1);

        canvas_fill_rect(canvas, width, height, dot_x - 2, dot_y - 2, 5, 5, MAP_CAR_COLOR);

        angle_rad = ((float)openart_pose.angle10 / 10.0f) * 0.01745329252f;
        tip_x = (int16)(dot_x + (int16)(cosf(angle_rad) * CAR_HEAD_LEN));
        tip_y = (int16)(dot_y - (int16)(sinf(angle_rad) * CAR_HEAD_LEN));
        head1_x = (int16)(tip_x + (int16)(cosf(angle_rad + CAR_HEAD_ANGLE) * 4.0f));
        head1_y = (int16)(tip_y - (int16)(sinf(angle_rad + CAR_HEAD_ANGLE) * 4.0f));
        head2_x = (int16)(tip_x + (int16)(cosf(angle_rad - CAR_HEAD_ANGLE) * 4.0f));
        head2_y = (int16)(tip_y - (int16)(sinf(angle_rad - CAR_HEAD_ANGLE) * 4.0f));

        canvas_draw_line(canvas, width, height, dot_x, dot_y, tip_x, tip_y, MAP_CAR_HEAD_COLOR);
        canvas_draw_line(canvas, width, height, tip_x, tip_y, head1_x, head1_y, MAP_CAR_HEAD_COLOR);
        canvas_draw_line(canvas, width, height, tip_x, tip_y, head2_x, head2_y, MAP_CAR_HEAD_COLOR);
    }
    else
    {
        border_color = STATUS_INVALID_COLOR;
        canvas_draw_rect(canvas, width, height, panel_x, panel_y, INFO_BOX_W, INFO_BOX_H, border_color);
        canvas_fill_rect(canvas, width, height, box_x, box_y, (uint16)box_w, (uint16)box_h, STATUS_BG_COLOR);
    }
}


static void render_map_canvas(void)
{
    uint8 row;
    uint8 col;
    uint8 cell;
    uint8 cols;
    uint8 rows;
    uint8 has_car;
    uint8 car_col;
    uint8 car_row;
    uint16 x;
    uint16 y;
    uint16 color;
    uint16 index;
    int16 car_x;
    int16 car_y;

    canvas_fill(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, MAP_BG_COLOR);
    canvas_draw_rect(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, 0, 0, MAP_CANVAS_W, MAP_CANVAS_H,
                     openart_map.valid ? MAP_GRID_COLOR : MAP_INVALID_BORDER);

    cols = openart_map.valid ? openart_map.cols : 0;
    rows = openart_map.valid ? openart_map.rows : 0;
    if(cols > MAP_COLS_VIEW)
    {
        cols = MAP_COLS_VIEW;
    }
    if(rows > MAP_ROWS_VIEW)
    {
        rows = MAP_ROWS_VIEW;
    }

    for(row = 0; row < MAP_ROWS_VIEW; row++)
    {
        for(col = 0; col < MAP_COLS_VIEW; col++)
        {
            x = (uint16)col * MAP_CELL_W;
            y = (uint16)row * MAP_CELL_H;
            cell = OPENART_CELL_BACKGROUND;
            if((openart_map.valid) && (row < rows) && (col < cols))
            {
                index = (uint16)row * openart_map.cols + col;
                if(index < OPENART_MAP_CELL_MAX)
                {
                    cell = openart_map.cells[index];
                }
            }

            canvas_fill_rect(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H,
                             (int16)x + 1, (int16)y + 1,
                             MAP_CELL_W - 2, MAP_CELL_H - 2, map_cell_color(cell));
            canvas_draw_rect(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H,
                             (int16)x, (int16)y, MAP_CELL_W, MAP_CELL_H, MAP_GRID_COLOR);
        }
    }

    has_car = get_car_cell_position(&car_col, &car_row);
    if(has_car)
    {
        x = (uint16)car_col * MAP_CELL_W;
        y = (uint16)car_row * MAP_CELL_H;
        canvas_draw_rect(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H,
                         (int16)x, (int16)y, MAP_CELL_W, MAP_CELL_H, MAP_CAR_COLOR);

        if(get_car_canvas_position(&car_x, &car_y))
        {
            canvas_fill_rect(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, car_x - 2, car_y - 2, 5, 5, MAP_CAR_COLOR);

            if(openart_pose.valid)
            {
                float angle_rad = ((float)openart_pose.angle10 / 10.0f) * 0.01745329252f;
                int16 tip_x = (int16)(car_x + (int16)(cosf(angle_rad) * CAR_HEAD_LEN));
                int16 tip_y = (int16)(car_y - (int16)(sinf(angle_rad) * CAR_HEAD_LEN));
                int16 head1_x = (int16)(tip_x + (int16)(cosf(angle_rad + CAR_HEAD_ANGLE) * 4.0f));
                int16 head1_y = (int16)(tip_y - (int16)(sinf(angle_rad + CAR_HEAD_ANGLE) * 4.0f));
                int16 head2_x = (int16)(tip_x + (int16)(cosf(angle_rad - CAR_HEAD_ANGLE) * 4.0f));
                int16 head2_y = (int16)(tip_y - (int16)(sinf(angle_rad - CAR_HEAD_ANGLE) * 4.0f));

                canvas_draw_line(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, car_x, car_y, tip_x, tip_y, MAP_CAR_HEAD_COLOR);
                canvas_draw_line(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, tip_x, tip_y, head1_x, head1_y, MAP_CAR_HEAD_COLOR);
                canvas_draw_line(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, tip_x, tip_y, head2_x, head2_y, MAP_CAR_HEAD_COLOR);
            }
        }
    }
}


static void render_status_canvas(void)
{
    int16 car_x;
    int16 car_y;
    uint16 speed;

    canvas_fill(status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H, STATUS_BG_COLOR);
    canvas_draw_rect(status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H, 0, 0, STATUS_CANVAS_W, STATUS_CANVAS_H, STATUS_BORDER_COLOR);

    speed = 0;
    if(get_car_canvas_position(&car_x, &car_y))
    {
        speed = estimate_speed_px_s(car_x, car_y);
    }
    draw_speed_bar(status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H, speed);
    draw_position_panel(status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H);
}


static void render_status_debug_text(void)
{
    static main_control_map_pos_t boxes[OPENART_MAP_CELL_MAX];
    static main_control_map_pos_t goals[OPENART_MAP_CELL_MAX];
    main_control_map_pos_t car_pos;
    char line[STATUS_DEBUG_BUFFER_SIZE];
    uint16 box_count;
    uint16 goal_count;

    box_count = main_control_find_boxes(&openart_map, boxes, OPENART_MAP_CELL_MAX);
    goal_count = main_control_find_goals(&openart_map, goals, OPENART_MAP_CELL_MAX);

    if(main_control_get_car_map_pos(&openart_pose, &openart_map, &car_pos))
    {
        sprintf(line, "Car:(%u,%u)", car_pos.x, car_pos.y);
    }
    else
    {
        sprintf(line, "Car:invalid");
    }
    ips200pro_label_show_string(status_debug_label_id[0], line);

    sprintf(line, "Box:%u Goal:%u", box_count, goal_count);
    ips200pro_label_show_string(status_debug_label_id[1], line);
}


void openart_display_init(void)
{
    speed_last_ms = 0;
    speed_last_x = 0;
    speed_last_y = 0;
    speed_last_valid = 0;
    speed_px_s = 0;

    ips200pro_page_switch(ips200pro_init("", IPS200PRO_TITLE_BOTTOM, 30), PAGE_ANIM_OFF);
    ips200pro_set_default_font(FONT_SIZE_16);
    ips200pro_set_direction(IPS200PRO_PORTRAIT);
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);
    ips200pro_set_backlight(255);

    map_image_id = ips200pro_image_create(0, 0, MAP_CANVAS_W, MAP_CANVAS_H);
    status_image_id = ips200pro_image_create(0, MAP_CANVAS_H, STATUS_CANVAS_W, STATUS_CANVAS_H);

    canvas_fill(map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, MAP_BG_COLOR);
    canvas_fill(status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H, STATUS_BG_COLOR);

    if(map_image_id != 0U)
    {
        ips200pro_image_display(map_image_id, map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, IMAGE_RGB565, 0);
    }
    if(status_image_id != 0U)
    {
        ips200pro_image_display(status_image_id, status_canvas, STATUS_CANVAS_W, STATUS_CANVAS_H, IMAGE_RGB565, 0);
    }

    status_debug_label_id[0] = ips200pro_label_create(0, MAP_CANVAS_H,
                                                      STATUS_CANVAS_W, STATUS_DEBUG_LINE_HEIGHT);
    status_debug_label_id[1] = ips200pro_label_create(0, MAP_CANVAS_H + STATUS_DEBUG_LINE_HEIGHT,
                                                      STATUS_CANVAS_W, STATUS_DEBUG_LINE_HEIGHT);

    ips200pro_set_color(status_debug_label_id[0], COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(status_debug_label_id[0], COLOR_BACKGROUND, RGB565_BLACK);
    ips200pro_set_color(status_debug_label_id[1], COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(status_debug_label_id[1], COLOR_BACKGROUND, RGB565_BLACK);
}


void openart_display_update(void)
{
    render_map_canvas();

    if(map_image_id != 0U)
    {
        ips200pro_image_display(map_image_id, map_canvas, MAP_CANVAS_W, MAP_CANVAS_H, IMAGE_RGB565, 0);
    }

    render_status_debug_text();
}
