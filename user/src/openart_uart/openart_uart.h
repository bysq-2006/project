#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_headfile.h"


// UART used by the mainboard to receive OpenART packets.
#define OPENART_UART_INDEX          (UART_1)
// UART baud rate. This must match the OpenART side.
#define OPENART_UART_BAUD           (115200)
// Mainboard UART TX pin. It is optional while the mainboard only receives data.
#define OPENART_UART_TX_PIN         (UART1_TX_B12)
// Mainboard UART RX pin. Connect this pin to OpenART TX.
#define OPENART_UART_RX_PIN         (UART1_RX_B13)

// Maximum map column count. It should match OpenART GRID_CONFIG["cols"].
#define OPENART_MAP_COLS_MAX        (12)
// Maximum map row count. It should match OpenART GRID_CONFIG["rows"].
#define OPENART_MAP_ROWS_MAX        (16)
// Maximum number of map cells stored in the receive buffer.
#define OPENART_MAP_CELL_MAX        (OPENART_MAP_COLS_MAX * OPENART_MAP_ROWS_MAX)

// Map cell value: background or empty area.
#define OPENART_CELL_BACKGROUND     (0)
// Map cell value: wall.
#define OPENART_CELL_WALL           (1)
// Map cell value: goal.
#define OPENART_CELL_GOAL           (2)
// Map cell value: yellow box.
#define OPENART_CELL_YELLOW_BOX     (3)
// Map cell value: unknown or unmatched result.
#define OPENART_CELL_UNKNOWN        (255)


typedef struct
{
    // 1 means the latest car pose is valid. 0 means OpenART did not find a complete car pose.
    uint8 valid;
    // 1 means a new car pose packet was received. User code can clear it after reading.
    uint8 updated;
    // Packet sequence number from OpenART, 0-255 wraparound.
    uint8 seq;
    // Car x coordinate in map coordinates, multiplied by 10.
    int16 x10;
    // Car y coordinate in map coordinates, multiplied by 10.
    int16 y10;
    // Car angle in degrees, multiplied by 10.
    uint16 angle10;
} openart_pose_t;


typedef struct
{
    // 1 means the latest map data is valid. 0 means no valid map is available.
    uint8 valid;
    // 1 means a new map packet was received. User code can clear it after reading.
    uint8 updated;
    // Packet sequence number from OpenART, 0-255 wraparound.
    uint8 seq;
    // Number of map columns.
    uint8 cols;
    // Number of map rows.
    uint8 rows;
    // Map ROI width in pixels, multiplied by 10.
    uint16 width10;
    // Map ROI height in pixels, multiplied by 10.
    uint16 height10;
    // Row-major map cells: cells[row * cols + col].
    uint8 cells[OPENART_MAP_CELL_MAX];
} openart_map_t;


typedef struct
{
    // Total UART bytes received from OpenART.
    uint32 rx_bytes;
    // Packets with a valid header and checksum.
    uint16 packet_count;
    // Successfully parsed car pose packets.
    uint16 pose_packets;
    // Successfully parsed map packets.
    uint16 map_packets;
    // Packets rejected because checksum did not match.
    uint16 checksum_errors;
    // Packets rejected because type, length, or map size was invalid.
    uint16 format_errors;
    // Bytes dropped because the receive buffer was full.
    uint16 rx_overflows;
} openart_uart_status_t;


// Latest parsed car pose data.
extern openart_pose_t openart_pose;
// Latest parsed map data.
extern openart_map_t openart_map;
// UART receive and parse diagnostic counters.
extern openart_uart_status_t openart_uart_status;

// Initialize OpenART UART and clear receive results.
void openart_uart_init(void);
// Non-blocking UART parser. Call this repeatedly in the main loop.
void openart_uart_update(void);
// Pull pending UART bytes into the software receive buffer.
void openart_uart_interrupt_handler(void);
// Clear both updated flags without clearing the received data itself.
void openart_uart_clear_updated(void);

#endif
