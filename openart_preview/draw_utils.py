# 反转显示颜色，方便画到图像上。
def inverse_color(color):
    return (255 - color[0], 255 - color[1], 255 - color[2])


# 画单个色块的外框和中心点。
def draw_blob(img, name, blob, target_color, should_print):
    color = inverse_color(target_color)
    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color)
    if should_print:
        print("obj:", name,
              "cx:", blob.cx(), "cy:", blob.cy(),
              "w:", blob.w(), "h:", blob.h(),
              "pixels:", blob.pixels())


# 按长边方向把矩形拆成几段。
def split_rect(rect, split_ratio_min):
    x, y, w, h = rect
    if w <= 0 or h <= 0:
        return [rect]

    # 按较长边方向拆分，尽量分成接近正方形的小块。
    if w >= h:
        ratio = w / h
        if ratio < split_ratio_min:
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
    if ratio < split_ratio_min:
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


# 画拆分后的色块框。
def draw_split_blob(img, name, blob, target_color, should_print, split_ratio_min):
    color = inverse_color(target_color)
    rects = split_rect(blob.rect(), split_ratio_min)
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
