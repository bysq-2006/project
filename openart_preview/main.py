# OpenART Plus 颜色目标检测预览。

import sensor
import time
import math
from machine import Pin
import cmm

from blob_detect import detect_targets
from draw_utils import draw_blob, draw_split_blob
from point_locator import get_detect_rois


# OpenMV 的 LAB 阈值格式：（L最小，L最大，A最小，A最大，B最小，B最大）。
PRINT_EVERY_N_FRAMES = 5
MAX_BLOBS_PER_COLOR = 6
SPLIT_RATIO_MIN = 1.5
# 手动识别范围，格式：((左上x, 左上y), (右下x, 右下y))；不想手动限制就填 None。
MANUAL_RANGE_POINTS = None
# 自动识别范围的阈值，多个颜色放在同一次联通检测里。
AUTO_RANGE_THRESHOLDS = (
    (25, 56, 17, 86, -113, -57),
    (80, 100, -25, 5, 70, 110),
    (45, 75, 70, 110, -75, -35),
    (78, 100, -65, -25, -35, 10),
    (72, 100, -110, -60, 55, 100),
)
AUTO_RANGE_MIN_PIXELS = 200
AUTO_RANGE_SCALE_W = 1.2
AUTO_RANGE_SCALE_H = 1.12


# 每一项格式：
# 位置1：目标名字，用来区分检测到的是什么。
# 位置2：LAB 阈值范围，格式是 ((L最小, L最大, A最小, A最大, B最小, B最大),)。
# 位置3：画框颜色，只影响显示，不影响识别。
# 位置4：最少像素/面积，小于这个值的色块会被过滤。
# 位置5：是否拆框显示，True 表示长条色块会被拆成几段画。
TARGETS = (
    ("yellow_box", ((80, 100, -25, 5, 70, 110),), (255, 255, 0), 30, True),
    ("purple_goal", ((45, 75, 70, 110, -75, -35),), (255, 0, 255), 25, True),
    # 由 RGB 采样值换算得到的初始 LAB 阈值。
    ("cyan_marker", ((78, 100, -65, -25, -35, 10),), (0, 255, 255), 25, False),
    ("green_marker", ((72, 100, -110, -60, 55, 100),), (0, 255, 0), 25, False),
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

while True:
    frame_id += 1
    should_print = PRINT_EVERY_N_FRAMES and (frame_id % PRINT_EVERY_N_FRAMES) == 0
    clock.tick()
    img = sensor.snapshot()

    base_roi, detect_roi = get_detect_rois(img,
                                           MANUAL_RANGE_POINTS,
                                           AUTO_RANGE_THRESHOLDS,
                                           AUTO_RANGE_MIN_PIXELS,
                                           AUTO_RANGE_SCALE_W,
                                           AUTO_RANGE_SCALE_H)

    marker_centers = {}
    detections = detect_targets(img, TARGETS, MAX_BLOBS_PER_COLOR, detect_roi)
    if detect_roi is not None:
        img.draw_rectangle(detect_roi, color=(255, 0, 0))
    if base_roi is not None:
        img.draw_rectangle(base_roi, color=(255, 255, 255))

    for name, blob, target_color, should_split in detections:
        if should_split:
            draw_split_blob(img, name, blob, target_color, should_print, SPLIT_RATIO_MIN)
        else:
            draw_blob(img, name, blob, target_color, should_print)
            if name in ("cyan_marker", "green_marker") and name not in marker_centers:
                marker_centers[name] = (blob.cx(), blob.cy())

    if "cyan_marker" in marker_centers and "green_marker" in marker_centers:
        c1 = marker_centers["cyan_marker"]
        c2 = marker_centers["green_marker"]
        try:
            img.draw_line((c1[0], c1[1], c2[0], c2[1]), color=(255, 255, 255), thickness=2)
        except TypeError:
            img.draw_line(c1[0], c1[1], c2[0], c2[1], color=(255, 255, 255))
        angle = math.degrees(math.atan2(c2[1] - c1[1], c2[0] - c1[0]))
        if angle < 0:
            angle += 360
        if should_print:
            print("cyan->green angle:", angle, "deg",
                  "cyan:", c1, "green:", c2)

    if should_print:
        print("fps:", clock.fps())
