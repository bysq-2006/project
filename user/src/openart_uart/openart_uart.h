#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_headfile.h"


// 主板用于接收 OpenART 数据的串口号。
#define OPENART_UART_INDEX          (UART_1)
// OpenART 与主板通信的串口波特率，两端必须一致。
#define OPENART_UART_BAUD           (115200)
// 主板串口发送引脚；如果主板暂时不向 OpenART 发命令，可以只接收不使用。
#define OPENART_UART_TX_PIN         (UART1_TX_B12)
// 主板串口接收引脚，需要连接 OpenART 的 TX。
#define OPENART_UART_RX_PIN         (UART1_RX_B13)

// 地图最大列数，对应 OpenART 端 GRID_CONFIG["cols"]。
#define OPENART_MAP_COLS_MAX        (12)
// 地图最大行数，对应 OpenART 端 GRID_CONFIG["rows"]。
#define OPENART_MAP_ROWS_MAX        (16)
// 地图格子最大数量，用来固定 cells 缓冲区大小。
#define OPENART_MAP_CELL_MAX        (OPENART_MAP_COLS_MAX * OPENART_MAP_ROWS_MAX)

// 地图格子识别结果：背景/空地。
#define OPENART_CELL_BACKGROUND     (0)
// 地图格子识别结果：墙。
#define OPENART_CELL_WALL           (1)
// 地图格子识别结果：终点。
#define OPENART_CELL_GOAL           (2)
// 地图格子识别结果：黄色箱子。
#define OPENART_CELL_YELLOW_BOX     (3)
// 地图格子识别结果：未知或未匹配。
#define OPENART_CELL_UNKNOWN        (255)


typedef struct
{
    // 1 表示本次小车姿态有效，0 表示 OpenART 当前没有识别到完整小车。
    uint8 valid;
    // 1 表示收到新的小车姿态包；用户读取后可以清 0。
    uint8 updated;
    // OpenART 发来的包序号，0-255 循环，用于判断是否有丢包或新旧数据。
    uint8 seq;
    // 小车在地图坐标系中的 x 坐标 * 10；还原时除以 10.0f。
    int16 x10;
    // 小车在地图坐标系中的 y 坐标 * 10；还原时除以 10.0f。
    int16 y10;
    // 小车角度 * 10，单位为度；还原时除以 10.0f。
    uint16 angle10;
} openart_pose_t;


typedef struct
{
    // 1 表示本次地图数据有效，0 表示 OpenART 当前没有有效地图识别结果。
    uint8 valid;
    // 1 表示收到新的地图包；用户读取后可以清 0。
    uint8 updated;
    // OpenART 发来的包序号，0-255 循环，用于判断是否有丢包或新旧数据。
    uint8 seq;
    // 地图列数，也就是每一行有多少个格子。
    uint8 cols;
    // 地图行数，也就是总共有多少行格子。
    uint8 rows;
    // 地图 ROI 宽度像素 * 10；还原时除以 10.0f。
    uint16 width10;
    // 地图 ROI 高度像素 * 10；还原时除以 10.0f。
    uint16 height10;
    // 地图格子识别结果，按行优先排列：cells[row * cols + col]。
    uint8 cells[OPENART_MAP_CELL_MAX];
} openart_map_t;


// 最新一次解析到的小车姿态数据。
extern openart_pose_t openart_pose;
// 最新一次解析到的地图识别数据。
extern openart_map_t openart_map;

// 初始化 OpenART 通信用 UART，并清空接收结果。
void openart_uart_init(void);
// 非阻塞处理串口数据；收到完整有效包后更新 openart_pose/openart_map。
void openart_uart_update(void);
// 同时清除小车和地图的 updated 标志，不会清除数据本身。
void openart_uart_clear_updated(void);

#endif
