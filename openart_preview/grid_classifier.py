# 把图像按指定网格拆开，并用 LAB 均值距离判断每格最接近的颜色类型。
def classify_grid(img, roi=None, grid_cols=12, grid_rows=16,
                  color_means=None, max_score=None, fallback_name="background"):
    # 每个格子只统计中间区域，减少边框和相邻格子的干扰。
    sample_ratio = 0.6
    # LAB 三个通道的距离缩放，避免某个通道数值变化过大时完全压过其他通道。
    lab_scales = (30.0, 80.0, 80.0)
    # 默认均值先用原来阈值的中心点，后续可以用 lab_roi_tool_gui.py 采样后的均值替换。
    if color_means is None:
        color_means = (
            ("wall", (8.0, 0.0, 0.0)),
            ("blue_floor", (40.5, 51.5, -85.0)),
            ("purple_goal", (60.0, 90.0, -55.0)),
        )

    # 没有传入 roi 时，默认把整张图当成地图区域。
    if roi is None:
        roi = (0, 0, img.width(), img.height())

    # 把 roi 限制到图像范围内，防止手动传入的区域越界。
    x, y, w, h = roi
    x1 = max(0, min(x, img.width()))
    y1 = max(0, min(y, img.height()))
    x2 = max(0, min(x + w, img.width()))
    y2 = max(0, min(y + h, img.height()))
    w = x2 - x1
    h = y2 - y1
    if w <= 0 or h <= 0:
        return []
    if w < grid_cols or h < grid_rows:
        return []

    # 计算一个 LAB 值到某个颜色均值的归一化均方误差。
    def color_score(lab, mean):
        total = 0
        for i in range(3):
            delta = (lab[i] - mean[i]) / lab_scales[i]
            total += delta * delta
        return total / 3

    # 逐行、逐列统计每个格子的中心区域。
    result = []
    for row in range(grid_rows):
        grid_row = []
        for col in range(grid_cols):
            # 地图最外圈固定是围墙，直接写入结果，不再做颜色采样。
            if row == 0 or row == grid_rows - 1 or col == 0 or col == grid_cols - 1:
                grid_row.append("wall")
                continue

            cell_x1 = x1 + col * w // grid_cols
            cell_y1 = y1 + row * h // grid_rows
            cell_x2 = x1 + (col + 1) * w // grid_cols
            cell_y2 = y1 + (row + 1) * h // grid_rows
            cell_w = cell_x2 - cell_x1
            cell_h = cell_y2 - cell_y1
            if cell_w <= 0 or cell_h <= 0:
                grid_row.append("unknown")
                continue

            sample_w = int(cell_w * sample_ratio)
            sample_h = int(cell_h * sample_ratio)
            if sample_w < 1:
                sample_w = 1
            if sample_h < 1:
                sample_h = 1
            sample_x = cell_x1 + (cell_w - sample_w) // 2
            sample_y = cell_y1 + (cell_h - sample_h) // 2

            stats = img.get_statistics(roi=(sample_x, sample_y, sample_w, sample_h))
            lab = (stats.l_mean(), stats.a_mean(), stats.b_mean())

            best_name = "unknown"
            best_score = None
            for name, mean in color_means:
                score = color_score(lab, mean)
                if best_score is None or score < best_score:
                    best_name = name
                    best_score = score

            if best_score is None:
                best_name = "unknown"
            elif max_score is not None and best_score > max_score:
                best_name = fallback_name
            grid_row.append(best_name)
        result.append(grid_row)

    return result
