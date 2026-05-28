# 把图像按指定网格拆开，并用颜色占比判断每格类型。
def classify_grid(img, roi=None, grid_cols=12, grid_rows=16,
                  ratio_rules=None, fallback_name="background"):
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

    # 统计某个阈值在格子里占了多少比例。
    def threshold_ratio(cell_roi, thresholds, cell_area):
        pixels = 0
        blobs = img.find_blobs(thresholds,
                               roi=cell_roi,
                               pixels_threshold=1,
                               area_threshold=1,
                               merge=False)
        for blob in blobs:
            pixels += blob.pixels()
        return pixels / cell_area

    # 用颜色占比判断格子类型。
    def classify_by_ratio(cell_roi, cell_area):
        best_name = None
        best_ratio = None

        for name, thresholds, min_ratio in ratio_rules:
            ratio = threshold_ratio(cell_roi, thresholds, cell_area)
            if ratio < min_ratio:
                continue

            if best_name is None or ratio > best_ratio:
                best_name = name
                best_ratio = ratio

        if best_name is None:
            return fallback_name
        return best_name

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

            cell_roi = (cell_x1, cell_y1, cell_w, cell_h)
            grid_row.append(classify_by_ratio(cell_roi, cell_w * cell_h))
        result.append(grid_row)

    return result
