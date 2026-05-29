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


openart_pose_t openart_pose;
openart_map_t openart_map;

static uint8 packet_type0;
static uint8 packet_type1;
static uint8 packet_payload[OPENART_PACKET_MAX];
static uint16 packet_len;


static uint16 openart_get_u16(const uint8 *data)
{
    return (uint16)data[0] | ((uint16)data[1] << 8);
}


static int16 openart_get_i16(const uint8 *data)
{
    return (int16)openart_get_u16(data);
}


void openart_uart_init(void)
{
    openart_pose.valid = 0;
    openart_pose.updated = 0;
    openart_pose.seq = 0;
    openart_pose.x10 = 0;
    openart_pose.y10 = 0;
    openart_pose.angle10 = 0;

    openart_map.valid = 0;
    openart_map.updated = 0;
    openart_map.seq = 0;
    openart_map.cols = 0;
    openart_map.rows = 0;
    openart_map.width10 = 0;
    openart_map.height10 = 0;

    uart_init(OPENART_UART_INDEX, OPENART_UART_BAUD,
              OPENART_UART_TX_PIN, OPENART_UART_RX_PIN);
}


void openart_uart_clear_updated(void)
{
    openart_pose.updated = 0;
    openart_map.updated = 0;
}


static uint8 openart_try_receive_packet(void)
{
    static uint8 state = 0;
    static uint16 index = 0;
    static uint16 checksum_calc = 0;
    static uint16 checksum_recv = 0;
    static uint8 checksum_l = 0;
    uint8 data = 0;

    while(uart_query_byte(OPENART_UART_INDEX, &data))
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
                    return 1;
                }
                break;

            default:
                state = 0;
                break;
        }
    }

    return 0;
}


static uint8 openart_parse_pose_packet(void)
{
    if(OPENART_POSE_PAYLOAD_LEN != packet_len)
    {
        return 0;
    }

    openart_pose.seq = packet_payload[0];
    openart_pose.valid = packet_payload[1] & 0x01;
    openart_pose.x10 = openart_get_i16(&packet_payload[2]);
    openart_pose.y10 = openart_get_i16(&packet_payload[4]);
    openart_pose.angle10 = openart_get_u16(&packet_payload[6]);
    openart_pose.updated = 1;

    return 1;
}


static uint8 openart_parse_map_packet(void)
{
    uint16 cell_count;
    uint16 i;

    if(packet_len < OPENART_MAP_HEADER_LEN)
    {
        return 0;
    }

    openart_map.seq = packet_payload[0];
    openart_map.valid = packet_payload[1] & 0x01;
    openart_map.cols = packet_payload[2];
    openart_map.rows = packet_payload[3];
    openart_map.width10 = openart_get_u16(&packet_payload[4]);
    openart_map.height10 = openart_get_u16(&packet_payload[6]);

    if(!openart_map.valid)
    {
        openart_map.updated = 1;
        return 1;
    }

    if((0 == openart_map.cols) || (0 == openart_map.rows) ||
       (OPENART_MAP_COLS_MAX < openart_map.cols) ||
       (OPENART_MAP_ROWS_MAX < openart_map.rows))
    {
        openart_map.valid = 0;
        return 0;
    }

    cell_count = (uint16)openart_map.cols * openart_map.rows;
    if((OPENART_MAP_CELL_MAX < cell_count) ||
       (packet_len != (uint16)(OPENART_MAP_HEADER_LEN + cell_count)))
    {
        openart_map.valid = 0;
        return 0;
    }

    for(i = 0; i < cell_count; i++)
    {
        openart_map.cells[i] = packet_payload[OPENART_MAP_HEADER_LEN + i];
    }

    openart_map.updated = 1;
    return 1;
}


static void openart_handle_packet(void)
{
    if((OPENART_PACKET_POSE_0 == packet_type0) &&
       (OPENART_PACKET_POSE_1 == packet_type1))
    {
        openart_parse_pose_packet();
    }
    else if((OPENART_PACKET_MAP_0 == packet_type0) &&
            (OPENART_PACKET_MAP_1 == packet_type1))
    {
        openart_parse_map_packet();
    }
}


void openart_uart_update(void)
{
    while(openart_try_receive_packet())
    {
        openart_handle_packet();
    }
}
