#include "zf_common_headfile.h"
#include "openart_uart/openart_uart.h"

#define TEST_PACKET_HEADER_0        (0xAA)
#define TEST_PACKET_HEADER_1        (0x55)
#define TEST_PACKET_TYPE_0          ('C')
#define TEST_PACKET_TYPE_1          ('T')
#define TEST_PACKET_PAYLOAD_LEN     (1)

static void test_send_packet(uint8 seq)
{
    uint8 packet[9];
    uint16 checksum = 0;
    uint16 index;

    packet[0] = TEST_PACKET_HEADER_0;
    packet[1] = TEST_PACKET_HEADER_1;
    packet[2] = TEST_PACKET_TYPE_0;
    packet[3] = TEST_PACKET_TYPE_1;
    packet[4] = TEST_PACKET_PAYLOAD_LEN & 0xFF;
    packet[5] = (TEST_PACKET_PAYLOAD_LEN >> 8) & 0xFF;
    packet[6] = seq;

    for(index = 2; index <= 6; index++)
    {
        checksum = (uint16)(checksum + packet[index]);
    }

    packet[7] = checksum & 0xFF;
    packet[8] = (checksum >> 8) & 0xFF;

    uart_write_buffer(OPENART_UART_INDEX, packet, sizeof(packet));
}

int main(void)
{
    uint8 seq = 0;

    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(100);

    openart_uart_init();

    while(1)
    {
        test_send_packet(seq++);
        system_delay_ms(1000);
    }
}
