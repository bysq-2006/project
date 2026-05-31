#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_headfile.h"


// 主控板接收 OpenART 数据包使用的 UART。
#define OPENART_UART_INDEX          (UART_1)
// UART 波特率，必须和 OpenART 端一致。
#define OPENART_UART_BAUD           (115200)
// 主控板 UART 发送引脚。当前只接收数据时，这个引脚可不接。
#define OPENART_UART_TX_PIN         (UART1_TX_B12)
// 主控板 UART 接收引脚，连接到 OpenART 的 TX。
#define OPENART_UART_RX_PIN         (UART1_RX_B13)

// 地图最大列数，应与 OpenART 的 GRID_CONFIG["cols"] 保持一致。
#define OPENART_MAP_COLS_MAX        (12)
// 地图最大行数，应与 OpenART 的 GRID_CONFIG["rows"] 保持一致。
#define OPENART_MAP_ROWS_MAX        (16)
// 接收缓存中可保存的最大地图格子数量。
#define OPENART_MAP_CELL_MAX        (OPENART_MAP_COLS_MAX * OPENART_MAP_ROWS_MAX)

// 地图格子值：背景或空白区域。
#define OPENART_CELL_BACKGROUND     (0)
// 地图格子值：墙。
#define OPENART_CELL_WALL           (1)
// 地图格子值：目标点。
#define OPENART_CELL_GOAL           (2)
// 地图格子值：黄色箱子。
#define OPENART_CELL_YELLOW_BOX     (3)
// 地图格子值：未知或不匹配结果。
#define OPENART_CELL_UNKNOWN        (255)


typedef struct
{
    // 1 表示最新车位姿有效，0 表示 OpenART 没有找到完整车位姿。
    uint8 valid;
    // 1 表示收到了一包新的车位姿数据，用户读取后可清零。
    uint8 updated;
    // OpenART 的数据包序号，0-255 循环。
    uint8 seq;
    // 车的 x 坐标，单位是地图坐标乘以 10。
    int16 x10;
    // 车的 y 坐标，单位是地图坐标乘以 10。
    int16 y10;
    // 车的角度，单位是角度乘以 10。
    uint16 angle10;
} openart_pose_t;


typedef struct
{
    // 1 表示最新地图有效，0 表示当前没有可用地图。
    uint8 valid;
    // 1 表示收到了一包新的地图数据，用户读取后可清零。
    uint8 updated;
    // OpenART 的数据包序号，0-255 循环。
    uint8 seq;
    // 地图列数。
    uint8 cols;
    // 地图行数。
    uint8 rows;
    // 地图 ROI 宽度，单位是像素乘以 10。
    uint16 width10;
    // 地图 ROI 高度，单位是像素乘以 10。
    uint16 height10;
    // 按行优先存储的地图格子：cells[row * cols + col]。
    uint8 cells[OPENART_MAP_CELL_MAX];
} openart_map_t;


typedef struct
{
    // 从 OpenART 接收到的 UART 总字节数。
    uint32 rx_bytes;
    // 头部和校验都正确的数据包数量。
    uint16 packet_count;
    // 成功解析的车位姿数据包数量。
    uint16 pose_packets;
    // 成功解析的地图数据包数量。
    uint16 map_packets;
    // 因校验失败而丢弃的数据包数量。
    uint16 checksum_errors;
    // 因类型、长度或地图尺寸非法而丢弃的数据包数量。
    uint16 format_errors;
    // 因接收缓冲区满而丢弃的字节数。
    uint16 rx_overflows;
} openart_uart_status_t;


// 最新解析到的车位姿数据。
extern openart_pose_t openart_pose;
// 最新解析到的地图数据。
extern openart_map_t openart_map;
// UART 接收和解析的统计信息。
extern openart_uart_status_t openart_uart_status;

// 初始化 OpenART UART 并清空接收结果。
void openart_uart_init(void);
// 非阻塞 UART 解析函数，建议在主循环中反复调用。
void openart_uart_update(void);
// 将待处理的 UART 数据搬运到软件接收缓冲区。
void openart_uart_interrupt_handler(void);
// 只清除 updated 标志，不清除已经接收到的数据。
void openart_uart_clear_updated(void);

#endif
