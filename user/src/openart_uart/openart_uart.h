#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_headfile.h"

#define OPENART_UART_INDEX          (UART_1)
#define OPENART_UART_BAUD           (115200)
#define OPENART_UART_TX_PIN         (UART1_TX_B12)
#define OPENART_UART_RX_PIN         (UART1_RX_B13)

#define OPENART_MAP_COLS_MAX        (12)
#define OPENART_MAP_ROWS_MAX        (16)
#define OPENART_MAP_CELL_MAX        (OPENART_MAP_COLS_MAX * OPENART_MAP_ROWS_MAX)
#define OPENART_BOX_COUNT_MAX       (10)

#define OPENART_CELL_BACKGROUND     (0)
#define OPENART_CELL_WALL           (1)
#define OPENART_CELL_GOAL           (2)
#define OPENART_CELL_YELLOW_BOX     (3)
#define OPENART_CELL_GOAL_ID_BASE   (20)
#define OPENART_CELL_GOAL_ID_MAX    (29)
#define OPENART_CELL_BOX_ID_BASE    (30)
#define OPENART_CELL_BOX_ID_MAX     (39)
#define OPENART_CELL_UNKNOWN        (255)

typedef struct
{
    uint8 valid;
    int16 x10;
    int16 y10;
} openart_box_t;

typedef struct
{
    uint8 valid;
    uint8 updated;
    uint8 seq;
    int16 x10;
    int16 y10;
    uint16 angle10;
    uint8 box_count;
    openart_box_t boxes[OPENART_BOX_COUNT_MAX];
} openart_pose_t;

typedef struct
{
    uint8 valid;
    uint8 updated;
    uint8 seq;
    uint8 cols;
    uint8 rows;
    uint16 width10;
    uint16 height10;
    uint8 cells[OPENART_MAP_CELL_MAX];
} openart_map_t;

typedef struct
{
    uint32 rx_bytes;
    uint16 packet_count;
    uint16 pose_packets;
    uint16 map_packets;
    uint16 checksum_errors;
    uint16 format_errors;
    uint16 rx_overflows;
} openart_uart_status_t;

extern openart_uart_status_t openart_uart_status;

void openart_uart_init(void);
void openart_uart_request_map(void);
void openart_uart_update(openart_pose_t *pose, openart_map_t *map);
void openart_uart_interrupt_handler(void);
void openart_uart_clear_updated(openart_pose_t *pose, openart_map_t *map);

#endif
