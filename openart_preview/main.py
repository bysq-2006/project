# OpenART Plus 颜色目标检测预览。

import sensor
import time
import math
from machine import Pin
import cmm

from blob_detect import detect_targets
from draw_utils import draw_blob, draw_grid, draw_grid_results
from grid_classifier import classify_grid
from point_locator import get_detect_rois


# OpenMV 的 LAB 阈值格式：（L最小，L最大，A最小，A最大，B最小，B最大）。
# 是否在图像上画调试框、搜索线和中心点；False 时更接近真实 FPS。
DRAW_DEBUG = True
# 是否静打印识别地图里的每一个网格和字符（启用了之后程序会在一段时间后崩掉。但是可以简单查看使用）
CLASSIFY_GRID_MAP = True
# 地图宽度方向的格子数量。
GRID_MAP_COLS = 12
# 地图高度方向的格子数量。
GRID_MAP_ROWS = 16
# 地图 ROI 纠正和网格识别每隔多少毫秒运行一次。
MAP_UPDATE_INTERVAL_MS = 1000
# 网格分类用的颜色占比规则，格式：（名字，LAB阈值，最小占比）。
GRID_COLOR_RATIOS = (
    ("yellow_box", ((80, 100, -25, 5, 70, 110),), 0.20),
    ("wall", ((0, 100, -14, 70, -74, 25),), 0.25),
    ("goal", ((50, 72, 66, 104, -72, -27),), 0.25),
    ("background", ((32, 52, 33, 72, -106, -75),), 0.25),
)
# 是否把缓存的网格识别结果画到图像上。
DRAW_GRID_DEBUG = True
# 网格线显示颜色。
GRID_LINE_COLOR = (255, 255, 0)
# 网格识别字符显示颜色。
GRID_TEXT_COLOR = (0, 255, 0)
# 每种网格识别结果对应的显示字符。
GRID_DEBUG_SYMBOLS = {
    "wall": "#",
    "unknown": "?",
    "background": "-",
    "goal": "+",
    "yellow_box": "$",
}
# 每隔多少帧打印一次调试信息和 FPS。
PRINT_EVERY_N_FRAMES = 5
# 每一种目标颜色最多保留多少个色块。
MAX_BLOBS_PER_COLOR = 6
# 地图识别范围配置，集中放这里方便调参。
RANGE_CONFIG = {
    # 手动识别范围，格式：((左上x, 左上y), (右下x, 右下y))；不想手动限制就填 None。
    "manual_points": None,
    # 自动识别范围的阈值，多个颜色放在同一次联通检测里。
    "thresholds": (
        (25, 56, 17, 86, -113, -57),
        (80, 100, -25, 5, 70, 110),
        (45, 75, 70, 110, -75, -35),
        (78, 100, -65, -25, -35, 10),
        (72, 100, -110, -60, 55, 100),
    ),
    # 自动识别地图范围时，过滤小色块的最小像素/面积。
    "min_pixels": 200,
    # 自动 ROI 放大时，宽度乘以这个系数。
    "scale_w": 1.2,
    # 自动 ROI 放大时，高度乘以这个系数。
    "scale_h": 1.12,
    # 自动识别地图范围时，只搜索画面左侧的比例。
    "search_ratio": 2 / 3,
}


# 每一项格式：
# 位置1：目标名字，用来区分检测到的是什么。
# 位置2：LAB 阈值范围，格式是 ((L最小, L最大, A最小, A最大, B最小, B最大),)。
# 位置3：画框颜色，只影响显示，不影响识别。
# 位置4：最少像素/面积，小于这个值的色块会被过滤。
# 要检测的目标色块配置。
TARGETS = (
    ("yellow_box", ((80, 100, -25, 5, 70, 110),), (255, 255, 0), 30),
    # 由 RGB 采样值换算得到的初始 LAB 阈值。
    ("cyan_marker", ((78, 100, -65, -25, -35, 10),), (0, 255, 255), 25),
    ("green_marker", ((72, 100, -110, -60, 55, 100),), (0, 255, 0), 25),
)


# 生成可用的引脚对象。
def make_pin(name):
    try:
        return Pin(name)
    except Exception:
        return None


# 初始化并注册 cmm 的引脚映射。
def load_cmm_config():
    pin_map = {
        "hw.-": ("rt117x", "seekfree_art_plus", None, None),
        "pin.J25": ("-", "AD_26", make_pin("AD_26"), None),
        "pin.J26": ("-", "AD_27", make_pin("AD_27"), None),
        "pin.J27": ("-", "AD_28", make_pin("AD_28"), None),
        "pin.J28": ("-", "AD_29", make_pin("AD_29"), None),
        "pin.M12": ("-", "LPSR_12", make_pin("LPSR_12"), None),
        "pin.M11": ("-", "LPSR_11", make_pin("LPSR_11"), None),
        "pin.M10": ("-", "LPSR_10", make_pin("LPSR_10"), None),
        "pin.M9": ("-", "LPSR_09", make_pin("LPSR_09"), None),
        "pin.M4": ("-", "LPSR_04", make_pin("LPSR_04"), None),
        "pin.M5": ("-", "LPSR_05", make_pin("LPSR_05"), None),
        "pin.J6": ("-", "AD_07", make_pin("AD_07"), None),
    }
    cmm.add(pin_map)
    print("cmm config ok")


load_cmm_config()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.SIF)
sensor.skip_frames(time=2000)

clock = time.clock()
frame_id = 0
last_map_update_ms = None
last_base_roi = None
last_detect_roi = None
last_grid_map = None

while True:
    frame_id += 1
    should_print = PRINT_EVERY_N_FRAMES and (frame_id % PRINT_EVERY_N_FRAMES) == 0
    clock.tick()
    img = sensor.snapshot()

    now_ms = time.ticks_ms()
    should_update_map = (last_map_update_ms is None or
                         time.ticks_diff(now_ms, last_map_update_ms) >= MAP_UPDATE_INTERVAL_MS)
    grid_map = last_grid_map
    # 每隔一段时间更新一次地图识别范围和网格识别结果，其他时候继续使用上次的结果，避免频繁运行耗时的识别函数。
    if should_update_map:
        new_base_roi, new_detect_roi = get_detect_rois(img, RANGE_CONFIG)
        last_map_update_ms = now_ms
        if new_detect_roi is not None:
            last_base_roi = new_base_roi
            last_detect_roi = new_detect_roi

    base_roi = last_base_roi
    detect_roi = last_detect_roi

    marker_centers = {}
    # 检测目标色块
    detections = detect_targets(img, TARGETS, MAX_BLOBS_PER_COLOR, detect_roi)
    if should_update_map and CLASSIFY_GRID_MAP and detect_roi is not None:
        grid_map = classify_grid(img, detect_roi,
                                 GRID_MAP_COLS, GRID_MAP_ROWS,
                                 GRID_COLOR_RATIOS,
                                 "background")
        if grid_map:
            last_grid_map = grid_map

    for name, blob, target_color in detections:
        if name in ("cyan_marker", "green_marker") and name not in marker_centers:
            marker_centers[name] = (blob.cx(), blob.cy())

    angle = None
    angle_points = None
    if "cyan_marker" in marker_centers and "green_marker" in marker_centers:
        c1 = marker_centers["cyan_marker"]
        c2 = marker_centers["green_marker"]
        angle_points = (c1, c2)
        angle = math.degrees(math.atan2(c2[1] - c1[1], c2[0] - c1[0]))
        if angle < 0:
            angle += 360
        if should_print:
            print("cyan->green angle:", angle, "deg",
                  "cyan:", c1, "green:", c2)

    should_draw_grid_debug = (DRAW_GRID_DEBUG and detect_roi is not None and
                              last_grid_map is not None)

    # 所有绘图统一放到这里，先算完数据再画。
    if DRAW_DEBUG:
        search_line_x = int(img.width() * RANGE_CONFIG["search_ratio"])
        try:
            img.draw_line((search_line_x, 0, search_line_x, img.height()), color=(255, 255, 0), thickness=2)
        except TypeError:
            img.draw_line(search_line_x, 0, search_line_x, img.height(), color=(255, 255, 0))
        if detect_roi is not None:
            img.draw_rectangle(detect_roi, color=(255, 0, 0))
        if base_roi is not None:
            img.draw_rectangle(base_roi, color=(255, 255, 255))

    if should_draw_grid_debug:
        draw_grid_results(img, detect_roi, last_grid_map,
                          GRID_DEBUG_SYMBOLS, GRID_TEXT_COLOR)
    if should_draw_grid_debug:
        draw_grid(img, detect_roi, GRID_MAP_COLS, GRID_MAP_ROWS, GRID_LINE_COLOR)

    for name, blob, target_color in detections:
        draw_blob(img, name, blob, target_color, should_print, DRAW_DEBUG)

    if DRAW_DEBUG and angle_points is not None:
        c1, c2 = angle_points
        try:
            img.draw_line((c1[0], c1[1], c2[0], c2[1]), color=(255, 255, 255), thickness=2)
        except TypeError:
            img.draw_line(c1[0], c1[1], c2[0], c2[1], color=(255, 255, 255))

    if should_print:
        print("fps:", clock.fps())
