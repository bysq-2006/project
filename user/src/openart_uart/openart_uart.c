#include "openart_uart.h"


#define OPENART_PACKET_MAX          (256)
#define OPENART_PACKET_HEADER_0     (0xAA)
#define OPENART_PACKET_HEADER_1     (0x55)
#define OPENART_PACKET_POSE_0       ('P')
#define OPENART_PACKET_POSE_1       ('O')
#define OPENART_PACKET_MAP_0        ('M')
#define OPENART_PACKET_MAP_1        ('P')
#define OPENART_POSE_PAYLOAD_LEN    (8)
#define OPENART_MAP_HEADER_LEN      (8)
#define OPENART_RX_BUFFER_SIZE      (512)
#define OPENART_RX_BUFFER_MASK      (OPENART_RX_BUFFER_SIZE - 1)


openart_uart_status_t openart_uart_status;

static uint8 packet_type0;
static uint8 packet_type1;
static uint8 packet_payload[OPENART_PACKET_MAX];
static uint16 packet_len;
static volatile uint8 rx_buffer[OPENART_RX_BUFFER_SIZE];
static volatile uint16 rx_head;
static volatile uint16 rx_tail;


static uint16 openart_get_u16(const uint8 *data)
{
    return (uint16)data[0] | ((uint16)data[1] << 8);
}


static int16 openart_get_i16(const uint8 *data)
{
    return (int16)openart_get_u16(data);
}


static void openart_rx_buffer_clear(void)
{
    rx_head = 0;
    rx_tail = 0;
}


static void openart_rx_buffer_push(uint8 data)
{
    uint16 next_head;

    next_head = (uint16)((rx_head + 1) & OPENART_RX_BUFFER_MASK);
    if(next_head == rx_tail)
    {
        openart_uart_status.rx_overflows++;
        return;
    }

    rx_buffer[rx_head] = data;
    rx_head = next_head;
    openart_uart_status.rx_bytes++;
}


static uint8 openart_rx_buffer_pop(uint8 *data)
{
    if(rx_tail == rx_head)
    {
        return 0;
    }

    *data = rx_buffer[rx_tail];
    rx_tail = (uint16)((rx_tail + 1) & OPENART_RX_BUFFER_MASK);
    return 1;
}


void openart_uart_init(void)
{
    openart_uart_status.rx_bytes = 0;
    openart_uart_status.packet_count = 0;
    openart_uart_status.pose_packets = 0;
    openart_uart_status.map_packets = 0;
    openart_uart_status.checksum_errors = 0;
    openart_uart_status.format_errors = 0;
    openart_uart_status.rx_overflows = 0;
    openart_rx_buffer_clear();

    uart_init(OPENART_UART_INDEX, OPENART_UART_BAUD,
              OPENART_UART_TX_PIN, OPENART_UART_RX_PIN);
    uart_rx_interrupt(OPENART_UART_INDEX, 1);
}


void openart_uart_clear_updated(openart_pose_t *pose, openart_map_t *map)
{
    pose->updated = 0;
    map->updated = 0;
}


static uint8 openart_try_receive_packet(void)
{
    static uint8 state = 0;
    static uint16 index = 0;
    static uint16 checksum_calc = 0;
    static uint16 checksum_recv = 0;
    static uint8 checksum_l = 0;
    uint8 data = 0;

    openart_uart_interrupt_handler();

    while(openart_rx_buffer_pop(&data))
    {
        switch(state)
        {
            case 0:
                if(OPENART_PACKET_HEADER_0 == data)
                {
                    state = 1;
                }
                break;

            case 1:
                if(OPENART_PACKET_HEADER_1 == data)
                {
                    state = 2;
                }
                else
                {
                    state = (OPENART_PACKET_HEADER_0 == data) ? 1 : 0;
                }
                break;

            case 2:
                packet_type0 = data;
                checksum_calc = data;
                state = 3;
                break;

            case 3:
                packet_type1 = data;
                checksum_calc = (uint16)(checksum_calc + data);
                state = 4;
                break;

            case 4:
                packet_len = data;
                checksum_calc = (uint16)(checksum_calc + data);
                state = 5;
                break;

            case 5:
                packet_len |= ((uint16)data << 8);
                checksum_calc = (uint16)(checksum_calc + data);
                if(packet_len > OPENART_PACKET_MAX)
                {
                    openart_uart_status.format_errors++;
                    state = 0;
                }
                else
                {
                    index = 0;
                    state = (packet_len > 0) ? 6 : 7;
                }
                break;

            case 6:
                packet_payload[index++] = data;
                checksum_calc = (uint16)(checksum_calc + data);
                if(index >= packet_len)
                {
                    state = 7;
                }
                break;

            case 7:
                checksum_l = data;
                state = 8;
                break;

            case 8:
                checksum_recv = (uint16)checksum_l | ((uint16)data << 8);
                state = 0;
                if(checksum_recv == checksum_calc)
                {
                    openart_uart_status.packet_count++;
                    return 1;
                }
                openart_uart_status.checksum_errors++;
                break;

            default:
                state = 0;
                break;
        }
    }

    return 0;
}


void openart_uart_interrupt_handler(void)
{
    uint8 data = 0;

    while(uart_query_byte(OPENART_UART_INDEX, &data))
    {
        openart_rx_buffer_push(data);
    }
}


static uint8 openart_parse_pose_packet(openart_pose_t *pose)
{
    if(OPENART_POSE_PAYLOAD_LEN != packet_len)
    {
        return 0;
    }

    pose->seq = packet_payload[0];
    pose->valid = packet_payload[1] & 0x01;
    pose->x10 = openart_get_i16(&packet_payload[2]);
    pose->y10 = openart_get_i16(&packet_payload[4]);
    pose->angle10 = openart_get_u16(&packet_payload[6]);
    pose->updated = 1;

    return 1;
}


static uint8 openart_parse_map_packet(openart_map_t *map)
{
    uint16 cell_count;
    uint16 i;

    if(packet_len < OPENART_MAP_HEADER_LEN)
    {
        return 0;
    }

    map->seq = packet_payload[0];
    map->valid = packet_payload[1] & 0x01;
    map->cols = packet_payload[2];
    map->rows = packet_payload[3];
    map->width10 = openart_get_u16(&packet_payload[4]);
    map->height10 = openart_get_u16(&packet_payload[6]);

    if(!map->valid)
    {
        map->updated = 1;
        return 1;
    }

    if((0 == map->cols) || (0 == map->rows) ||
       (OPENART_MAP_COLS_MAX < map->cols) ||
       (OPENART_MAP_ROWS_MAX < map->rows))
    {
        map->valid = 0;
        return 0;
    }

    cell_count = (uint16)map->cols * map->rows;
    if((OPENART_MAP_CELL_MAX < cell_count) ||
       (packet_len != (uint16)(OPENART_MAP_HEADER_LEN + cell_count)))
    {
        map->valid = 0;
        return 0;
    }

    for(i = 0; i < cell_count; i++)
    {
        map->cells[i] = packet_payload[OPENART_MAP_HEADER_LEN + i];
    }

    map->updated = 1;
    return 1;
}


static void openart_handle_packet(openart_pose_t *pose, openart_map_t *map)
{
    if((OPENART_PACKET_POSE_0 == packet_type0) &&
       (OPENART_PACKET_POSE_1 == packet_type1))
    {
        if(openart_parse_pose_packet(pose))
        {
            openart_uart_status.pose_packets++;
        }
        else
        {
            openart_uart_status.format_errors++;
        }
    }
    else if((OPENART_PACKET_MAP_0 == packet_type0) &&
            (OPENART_PACKET_MAP_1 == packet_type1))
    {
        if(openart_parse_map_packet(map))
        {
            openart_uart_status.map_packets++;
        }
        else
        {
            openart_uart_status.format_errors++;
        }
    }
    else
    {
        openart_uart_status.format_errors++;
    }
}


void openart_uart_update(openart_pose_t *pose, openart_map_t *map)
{
    while(openart_try_receive_packet())
    {
        openart_handle_packet(pose, map);
    }
}
