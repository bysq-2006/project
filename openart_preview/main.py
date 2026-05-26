# OpenART Plus color target detection preview.

import sensor
import time
import math
from machine import Pin
import cmm


PRINT_EVERY_N_FRAMES = 5
MAX_BLOBS_PER_COLOR = 6
SPLIT_RATIO_MIN = 1.5


# OpenMV LAB thresholds: (L min, L max, A min, A max, B min, B max).
# First pass only detects the pure-color targets:
# box.png, Target.png, and car.png. WallBlock.png and bomb.png are disabled.
TARGETS = (
    ("yellow_box", ((80, 100, -25, 5, 70, 110),), (255, 255, 0), 30, True),
    ("purple_goal", ((45, 75, 70, 110, -75, -35),), (255, 0, 255), 25, True),
    # Initial LAB values converted from RGB samples.
    ("cyan_marker", ((78, 100, -65, -25, -35, 10),), (0, 255, 255), 25, False),
    ("green_marker", ((72, 100, -110, -60, 55, 100),), (0, 255, 0), 25, False),
)


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


def inverse_color(color):
    return (255 - color[0], 255 - color[1], 255 - color[2])


def draw_blob(img, name, blob, target_color, should_print):
    color = inverse_color(target_color)
    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color)
    if should_print:
        print("obj:", name,
              "cx:", blob.cx(), "cy:", blob.cy(),
              "w:", blob.w(), "h:", blob.h(),
              "pixels:", blob.pixels())


def good_blob(blob):
    w = blob.w()
    h = blob.h()
    if w < 3 or h < 3:
        return False
    if w > 90 or h > 90:
        return False
    ratio = w / h
    if ratio < 0.35 or ratio > 2.8:
        return False
    return True


def split_rect(rect):
    x, y, w, h = rect
    if w <= 0 or h <= 0:
        return [rect]

    # Split along the longer axis into roughly square pieces.
    if w >= h:
        ratio = w / h
        if ratio < SPLIT_RATIO_MIN:
            return [rect]
        parts = max(1, int((w + h / 2) / h))
        step = w // parts
        if step < 1:
            return [rect]
        result = []
        for i in range(parts):
            sx = x + i * step
            sw = step if i < parts - 1 else (x + w) - sx
            if sw > 0:
                result.append((sx, y, sw, h))
        return result

    ratio = h / w
    if ratio < SPLIT_RATIO_MIN:
        return [rect]
    parts = max(1, int((h + w / 2) / w))
    step = h // parts
    if step < 1:
        return [rect]
    result = []
    for i in range(parts):
        sy = y + i * step
        sh = step if i < parts - 1 else (y + h) - sy
        if sh > 0:
            result.append((x, sy, w, sh))
    return result


def draw_split_blob(img, name, blob, target_color, should_print):
    color = inverse_color(target_color)
    rects = split_rect(blob.rect())
    for rect in rects:
        rx, ry, rw, rh = rect
        img.draw_rectangle(rect, color=color)
        img.draw_cross(rx + rw // 2, ry + rh // 2, color=color)
    if should_print:
        print("obj:", name,
              "cx:", blob.cx(), "cy:", blob.cy(),
              "w:", blob.w(), "h:", blob.h(),
              "pixels:", blob.pixels(),
              "split:", len(rects))


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
    t0 = time.ticks_ms()
    img = sensor.snapshot()
    t1 = time.ticks_ms()

    found = 0
    marker_centers = {}
    detections = []
    for target in TARGETS:
        name, thresholds, target_color, min_pixels, should_split = target
        blobs = img.find_blobs(thresholds,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=False)
        blobs = [blob for blob in blobs if good_blob(blob)]
        blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
        for blob in blobs[:MAX_BLOBS_PER_COLOR]:
            detections.append((name, blob, target_color, should_split))
            found += 1

    for name, blob, target_color, should_split in detections:
        if should_split:
            draw_split_blob(img, name, blob, target_color, should_print)
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
    t2 = time.ticks_ms()

    if should_print:
        print("t1 snapshot cost:", time.ticks_diff(t1, t0), "ms")
        print("t2 detect cost  :", time.ticks_diff(t2, t1), "ms",
              "found:", found)
        print("fps:", clock.fps())
