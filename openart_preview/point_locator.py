# 自动识别地图范围，返回左上角和右下角两个点。
def auto_range_points(img, thresholds, min_pixels, search_ratio):
    best_blob = None
    search_w = int(img.width() * search_ratio)
    if search_w <= 0:
        return None

    blobs = img.find_blobs(thresholds,
                           roi=(0, 0, search_w, img.height()),
                           pixels_threshold=min_pixels,
                           area_threshold=min_pixels,
                           merge=False)
    for blob in blobs:
        if best_blob is None or blob.pixels() > best_blob.pixels():
            best_blob = blob

    if best_blob is None:
        return None

    x, y, w, h = best_blob.rect()
    return ((x, y), (x + w, y + h))


# 把左上角和右下角两个点转换成 OpenMV 需要的 roi 格式。
def points_to_roi(img, range_points):
    p1, p2 = range_points
    x1, y1 = p1
    x2, y2 = p2

    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    x1 = max(0, min(x1, img.width()))
    y1 = max(0, min(y1, img.height()))
    x2 = max(0, min(x2, img.width()))
    y2 = max(0, min(y2, img.height()))

    if x2 > x1 and y2 > y1:
        return (x1, y1, x2 - x1, y2 - y1)
    return None


# 以中心点不变的方式分别放大 roi 的宽和高。
def scale_roi(img, roi, scale_w, scale_h):
    if roi is None:
        return None

    x, y, w, h = roi
    new_w = int(w * scale_w)
    new_h = int(h * scale_h)
    if new_w < w:
        new_w = w
    if new_h < h:
        new_h = h

    cx = x + w / 2
    cy = y + h / 2
    x1 = int(cx - new_w / 2)
    y1 = int(cy - new_h / 2)
    x2 = x1 + new_w
    y2 = y1 + new_h

    return points_to_roi(img, ((x1, y1), (x2, y2)))


# 根据手动输入或自动识别返回原始 roi 和放大后的检测 roi。
def get_detect_rois(img, manual_points, thresholds, min_pixels,
                    scale_w, scale_h, search_ratio):
    if manual_points is None:
        range_points = auto_range_points(img, thresholds, min_pixels, search_ratio)
    else:
        range_points = manual_points

    if range_points is None:
        return None, None

    base_roi = points_to_roi(img, range_points)
    detect_roi = scale_roi(img, base_roi, scale_w, scale_h)
    return base_roi, detect_roi


# 只返回真正用于检测的 roi。
def get_detect_roi(img, manual_points, thresholds, min_pixels,
                   scale_w, scale_h, search_ratio):
    base_roi, detect_roi = get_detect_rois(img, manual_points,
                                           thresholds, min_pixels,
                                           scale_w, scale_h,
                                           search_ratio)
    return detect_roi
