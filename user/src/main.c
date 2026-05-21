#include "zf_common_headfile.h"

#define OPENART_UART                (UART_1)
#define OPENART_UART_BAUD           (115200)
#define OPENART_UART_TX             (UART1_TX_B12)
#define OPENART_UART_RX             (UART1_RX_B13)

#define OPENART_FRAME_WIDTH         (80)
#define OPENART_FRAME_HEIGHT        (60)
#define OPENART_FRAME_SIZE          (OPENART_FRAME_WIDTH * OPENART_FRAME_HEIGHT)

#define OPENART_DISPLAY_X           (40)
#define OPENART_DISPLAY_Y           (20)
#define OPENART_DISPLAY_WIDTH       (160)
#define OPENART_DISPLAY_HEIGHT      (120)

static uint8 openart_frame[OPENART_FRAME_SIZE];
static uint32 openart_frame_count = 0;
static uint32 openart_bad_frame_count = 0;

static uint16 page_id;
static uint16 image_id;
static uint16 status_label_id;

static uint8 openart_try_receive_frame(void)
{
    static const uint8 header[4] = {0xAA, 0x55, 'I', 'M'};
    static uint8 state = 0;
    static uint8 header_index = 0;
    static uint16 frame_index = 0;
    static uint16 checksum = 0;
    static uint16 checksum_calc = 0;
    static uint8 width = 0;
    static uint8 height = 0;
    static uint8 checksum_l = 0;
    uint8 data = 0;

    while(uart_query_byte(OPENART_UART, &data))
    {
        switch(state)
        {
            case 0:
                if(header[0] == data)
                {
                    header_index = 1;
                    state = 1;
                }
                break;

            case 1:
                if(header[header_index] == data)
                {
                    header_index++;
                    if(4 <= header_index)
                    {
                        state = 2;
                    }
                }
                else
                {
                    header_index = (header[0] == data) ? 1 : 0;
                    state = header_index ? 1 : 0;
                }
                break;

            case 2:
                width = data;
                state = 3;
                break;

            case 3:
                height = data;
                state = 4;
                break;

            case 4:
                state = 5;
                break;

            case 5:
                state = 6;
                break;

            case 6:
                checksum_l = data;
                state = 7;
                break;

            case 7:
                checksum = (uint16)checksum_l | ((uint16)data << 8);
                frame_index = 0;
                checksum_calc = 0;
                if((OPENART_FRAME_WIDTH == width) && (OPENART_FRAME_HEIGHT == height))
                {
                    state = 8;
                }
                else
                {
                    state = 0;
                    openart_bad_frame_count++;
                }
                break;

            case 8:
                openart_frame[frame_index++] = data;
                checksum_calc = (uint16)(checksum_calc + data);
                if(OPENART_FRAME_SIZE <= frame_index)
                {
                    state = 0;
                    if(checksum_calc == checksum)
                    {
                        return 1;
                    }
                    openart_bad_frame_count++;
                }
                break;

            default:
                state = 0;
                break;
        }
    }

    return 0;
}

static void ips200pro_openart_init(void)
{
    page_id = ips200pro_init("OpenART", IPS200PRO_TITLE_BOTTOM, 30);
    ips200pro_set_default_font(FONT_SIZE_16);
    ips200pro_set_direction(IPS200PRO_PORTRAIT);
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);
    ips200pro_set_color(page_id, COLOR_BACKGROUND, IPS200PRO_RGB888_TO_RGB565(0x20, 0x24, 0x2C));
    ips200pro_set_color(page_id, COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(page_id, COLOR_PAGE_SELECTED_TEXT, RGB565_WHITE);
    ips200pro_set_color(page_id, COLOR_PAGE_SELECTED_BG, IPS200PRO_RGB888_TO_RGB565(0x36, 0x73, 0xD9));
    ips200pro_page_switch(page_id, PAGE_ANIM_ON);
    ips200pro_set_backlight(255);

    image_id = ips200pro_image_create(OPENART_DISPLAY_X, OPENART_DISPLAY_Y, OPENART_DISPLAY_WIDTH, OPENART_DISPLAY_HEIGHT);
    status_label_id = ips200pro_label_create(8, 155, 220, 48);
    ips200pro_set_color(status_label_id, COLOR_FOREGROUND, RGB565_WHITE);
    ips200pro_set_color(status_label_id, COLOR_BACKGROUND, IPS200PRO_RGB888_TO_RGB565(0x20, 0x24, 0x2C));
    ips200pro_label_show_string(status_label_id, "wait OpenART...");
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(300);

    ips200pro_openart_init();

    uart_init(OPENART_UART, OPENART_UART_BAUD, OPENART_UART_TX, OPENART_UART_RX);
    interrupt_global_enable(0);

    while(1)
    {
        if(openart_try_receive_frame())
        {
            openart_frame_count++;
            ips200pro_label_printf(status_label_id, "frame:%d err:%d", openart_frame_count, openart_bad_frame_count);
            ips200pro_image_display(image_id, openart_frame, OPENART_FRAME_WIDTH, OPENART_FRAME_HEIGHT, IMAGE_GRAYSCALE, 0);
        }
    }
}
