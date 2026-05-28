# 反转显示颜色，方便画到图像上。
def inverse_color(color):
    return (255 - color[0], 255 - color[1], 255 - color[2])


# 兼容不同固件版本的画线接口。
def draw_line_safe(img, x1, y1, x2, y2, color, thickness=1):
    try:
        img.draw_line((x1, y1, x2, y2), color=color, thickness=thickness)
    except TypeError:
        img.draw_line(x1, y1, x2, y2, color=color)


# 兼容不同固件版本的文字绘制接口。
def draw_string_safe(img, x, y, text, color):
    try:
        img.draw_string(x, y, text, color=color, scale=1, mono_space=False)
    except TypeError:
        try:
            img.draw_string(x, y, text, color=color, scale=1)
        except TypeError:
            img.draw_string(x, y, text, color=color)


# 画单个色块的外框和中心点。
def draw_blob(img, name, blob, target_color, should_print, draw_debug=True):
    color = inverse_color(target_color)
    if draw_debug:
        img.draw_rectangle(blob.rect(), color=color)
        img.draw_cross(blob.cx(), blob.cy(), color=color)
    if should_print:
        print("obj:", name,
              "cx:", blob.cx(), "cy:", blob.cy(),
              "w:", blob.w(), "h:", blob.h(),
              "pixels:", blob.pixels())


# 在 roi 内画固定行列数的网格线。
def draw_grid(img, roi, cols=12, rows=16, color=(255, 255, 0)):
    if roi is None:
        return

    x, y, w, h = roi
    if w <= 0 or h <= 0:
        return

    for col in range(cols + 1):
        line_x = x + col * w // cols
        draw_line_safe(img, line_x, y, line_x, y + h, color)

    for row in range(rows + 1):
        line_y = y + row * h // rows
        draw_line_safe(img, x, line_y, x + w, line_y, color)


# 在每个网格中心画识别结果的单字符符号。
def draw_grid_results(img, roi, grid_map, symbols, color=(255, 255, 255)):
    if roi is None or not grid_map:
        return

    x, y, w, h = roi
    rows = len(grid_map)
    if rows <= 0 or w <= 0 or h <= 0:
        return

    for row_index in range(rows):
        row = grid_map[row_index]
        cols = len(row)
        if cols <= 0:
            continue
        for col_index in range(cols):
            name = row[col_index]
            text = symbols.get(name, "?")
            if not text:
                continue
            cell_x1 = x + col_index * w // cols
            cell_y1 = y + row_index * h // rows
            cell_x2 = x + (col_index + 1) * w // cols
            cell_y2 = y + (row_index + 1) * h // rows
            text_x = cell_x1 + (cell_x2 - cell_x1) // 2 - 3
            text_y = cell_y1 + (cell_y2 - cell_y1) // 2 - 4
            draw_string_safe(img, text_x, text_y, text, color)
