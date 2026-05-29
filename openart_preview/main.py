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
# 每隔多少帧打印一次调试信息和 FPS。
PRINT_EVERY_N_FRAMES = 5
# 每一种目标颜色最多保留多少个色块。
MAX_BLOBS_PER_COLOR = 6
# 网格分类配置，集中放这里方便调地图行列、阈值和调试显示。
GRID_CONFIG = {
    # 是否绘制网格和字符。
    "classify": False,
    # 地图宽度方向的格子数量。
    "cols": 12,
    # 地图高度方向的格子数量。
    "rows": 16,
    # 地图 ROI 纠正和网格识别每隔多少毫秒运行一次。
    "update_interval_ms": 1000,
    # 网格分类用的颜色占比规则，格式：（名字，LAB阈值，最小占比）。
    "color_ratios": (
        ("yellow_box", ((80, 100, -25, 5, 70, 110),), 0.20),
        ("wall", ((0, 100, -14, 70, -74, 25),), 0.25),
        ("goal", ((50, 72, 66, 104, -72, -27),), 0.25),
        ("background", ((32, 52, 33, 72, -106, -75),), 0.25),
    ),
    # 没有任何颜色占比达标时，默认当成这个类型。
    "fallback": "background",
    # 是否把缓存的网格识别结果画到图像上。
    "draw_debug": True,
    # 网格线显示颜色。
    "line_color": (255, 255, 0),
    # 网格识别字符显示颜色。
    "text_color": (0, 255, 0),
    # 每种网格识别结果对应的显示字符。
    "symbols": {
        "wall": "#",
        "unknown": "?",
        "background": "-",
        "goal": "+",
        "yellow_box": "$",
    },
}
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
                         time.ticks_diff(now_ms, last_map_update_ms) >= GRID_CONFIG["update_interval_ms"])
    grid_map = last_grid_map
    # 每隔一段时间更新一次地图识别范围和网格识别结果，其他时候继续使用上次的结果，避免频繁运行耗时的识别函数。
    if should_update_map:
        # 识别地图位置
        new_base_roi, new_detect_roi = get_detect_rois(img, RANGE_CONFIG)
        last_map_update_ms = now_ms
        if new_detect_roi is not None:
            last_base_roi = new_base_roi
            last_detect_roi = new_detect_roi
            # 识别地图里的网格信息
            if GRID_CONFIG["classify"]:
                grid_map = classify_grid(img, new_detect_roi,
                                         GRID_CONFIG["cols"], GRID_CONFIG["rows"],
                                         GRID_CONFIG["color_ratios"],
                                         GRID_CONFIG["fallback"])
                if grid_map:
                    last_grid_map = grid_map

    base_roi = last_base_roi
    detect_roi = last_detect_roi

    marker_centers = {}
    # 检测目标色块
    detections = detect_targets(img, TARGETS, MAX_BLOBS_PER_COLOR, detect_roi)

    for name, blob, target_color in detections:
        if name in ("cyan_marker", "green_marker") and name not in marker_centers:
            marker_centers[name] = (blob.cx(), blob.cy())

    # 检测小车朝向和位置
    angle = None
    angle_points = None
    car_center = None
    car_map_pos = None
    if "cyan_marker" in marker_centers and "green_marker" in marker_centers:
        front = marker_centers["cyan_marker"]
        back = marker_centers["green_marker"]
        angle_points = (front, back)
        car_center = ((front[0] + back[0]) / 2, (front[1] + back[1]) / 2)
        if detect_roi is not None:
            car_map_pos = (car_center[0] - detect_roi[0], car_center[1] - detect_roi[1])

        # 绿色在车尾，青色在车头；角度使用 x 向右、y 向上的坐标系。
        dx = front[0] - back[0]
        dy = back[1] - front[1]
        angle = math.degrees(math.atan2(dy, dx))
        if angle < 0:
            angle += 360
        if should_print:
            print("car angle:", angle, "deg",
                  "center:", car_center, "map_pos:", car_map_pos,
                  "cyan_front:", front, "green_back:", back)

    should_draw_grid_debug = (GRID_CONFIG["draw_debug"] and detect_roi is not None and
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
                          GRID_CONFIG["symbols"], GRID_CONFIG["text_color"])
    if should_draw_grid_debug:
        draw_grid(img, detect_roi, GRID_CONFIG["cols"], GRID_CONFIG["rows"],
                  GRID_CONFIG["line_color"])

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
