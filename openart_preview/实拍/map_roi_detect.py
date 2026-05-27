# OpenART/OpenMV left-map ROI detector.
#
# Purpose:
#   Only find the rectangular 2D map area on the left side of the frame.
#   This script does not classify grid cells.
#
# Method:
#   1. Scan only the left part of the frame.
#   2. Find dense strong-blue pixels from the map floor.
#   3. Build a histogram bbox from those blue pixels.
#   4. Expand the bbox a little to include the outer wall/border.

import sensor
import time
from machine import Pin
import cmm


# Camera/search tuning.
# Only detect the left map piece. Keep the search window on the left side so
# the right-side passage does not get pulled into the ROI.
SEARCH_RIGHT_RATIO = 0.52
SCAN_STEP = 2
PRINT_EVERY_N_FRAMES = 5

# Extra padding around the detected blue floor bbox.
# These are intentionally small/tunable because the map border thickness
# changes with camera distance and screen crop.
PAD_X_DIV = 100
PAD_TOP_DIV = 16
PAD_BOTTOM_DIV = 18

# Set this if the automatic ROI is close but you want to freeze it.
# Example for QVGA: MANUAL_MAP_RECT = (16, 8, 150, 224)
MANUAL_MAP_RECT = None

SAVE_ROI = False
SAVE_EVERY_N_FRAMES = 30

BLUE_BBOX_COLOR = (0, 255, 0)
MAP_ROI_COLOR = (255, 255, 255)


def make_pin(name):
    try:
        return Pin(name)
    except Exception:
        return None


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


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def get_rgb(img, x, y):
    p = img.get_pixel(x, y)
    if p is None:
        return None
    if isinstance(p, tuple):
        return p
    r = ((p >> 11) & 0x1f) * 255 // 31
    g = ((p >> 5) & 0x3f) * 255 // 63
    b = (p & 0x1f) * 255 // 31
    return (r, g, b)


def is_map_blue(rgb):
    if rgb is None:
        return False
    r, g, b = rgb
    # Stronger than a generic "blue" test.  This rejects the bluish black
    # background/reflection while keeping the saturated map floor.
    return b > 135 and b > r + 55 and b > g + 22 and g > 25


def first_hit(hist, limit, threshold):
    for i in range(limit):
        if hist[i] >= threshold:
            return i
    return -1


def last_hit(hist, limit, threshold):
    for i in range(limit - 1, -1, -1):
        if hist[i] >= threshold:
            return i
    return -1


def find_blue_bbox(img):
    iw = img.width()
    ih = img.height()
    search_w = int(iw * SEARCH_RIGHT_RATIO)
    if search_w <= 0:
        return None

    x_hist = [0] * iw
    y_hist = [0] * ih
    blue_count = 0

    for y in range(0, ih, SCAN_STEP):
        for x in range(0, search_w, SCAN_STEP):
            if is_map_blue(get_rgb(img, x, y)):
                x_hist[x] += 1
                y_hist[y] += 1
                blue_count += 1

    # Dense-hit threshold.  A single reflected blue pixel will not pass this.
    min_x_hits = max(3, ih // (SCAN_STEP * 20))
    min_y_hits = max(3, search_w // (SCAN_STEP * 20))

    x1 = first_hit(x_hist, search_w, min_x_hits)
    x2 = last_hit(x_hist, search_w, min_x_hits)
    y1 = first_hit(y_hist, ih, min_y_hits)
    y2 = last_hit(y_hist, ih, min_y_hits)

    if blue_count < 40 or x1 < 0 or y1 < 0 or x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2 - x1 + 1, y2 - y1 + 1)


def expand_to_map_rect(img, blue_rect):
    if MANUAL_MAP_RECT:
        return MANUAL_MAP_RECT
    if blue_rect is None:
        return None

    iw = img.width()
    ih = img.height()
    x, y, w, h = blue_rect

    pad_x = max(2, iw // PAD_X_DIV)
    pad_top = max(4, ih // PAD_TOP_DIV)
    pad_bottom = max(4, ih // PAD_BOTTOM_DIV)

    # If the blue bbox already starts near the top/bottom frame edge, avoid
    # expanding beyond the screen crop.
    pad_top = min(pad_top, max(0, y // 2))
    pad_bottom = min(pad_bottom, max(0, ih - (y + h)))

    x1 = clamp(x - pad_x, 0, iw - 1)
    y1 = clamp(y - pad_top, 0, ih - 1)
    x2 = clamp(x + w - 1 + pad_x, 0, iw - 1)
    y2 = clamp(y + h - 1 + pad_bottom, 0, ih - 1)
    return (x1, y1, x2 - x1 + 1, y2 - y1 + 1)


def save_roi(img, rect, frame_id):
    if not SAVE_ROI or rect is None:
        return
    if frame_id % SAVE_EVERY_N_FRAMES != 0:
        return
    try:
        roi_img = img.copy(roi=rect)
        path = "/sd/map_roi_%d.bmp" % frame_id
        roi_img.save(path)
        print("saved:", path)
    except Exception as e:
        print("save roi failed:", e)


load_cmm_config()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.SIF)
sensor.skip_frames(time=2000)

clock = time.clock()
frame_id = 0
last_map_rect = None

while True:
    frame_id += 1
    should_print = PRINT_EVERY_N_FRAMES and (frame_id % PRINT_EVERY_N_FRAMES) == 0

    clock.tick()
    img = sensor.snapshot()

    blue_rect = find_blue_bbox(img)
    map_rect = expand_to_map_rect(img, blue_rect)
    if map_rect is None:
        map_rect = last_map_rect
    else:
        last_map_rect = map_rect

    if blue_rect:
        img.draw_rectangle(blue_rect, color=BLUE_BBOX_COLOR)
    if map_rect:
        img.draw_rectangle(map_rect, color=MAP_ROI_COLOR)
        save_roi(img, map_rect, frame_id)

    if should_print:
        print("blue bbox:", blue_rect, "map roi:", map_rect,
              "fps:", clock.fps())
