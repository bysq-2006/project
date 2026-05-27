# 自动识别范围的逻辑先留空，后面再根据现场方案补。
def auto_range_points(img):
    return None


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


# 根据手动输入或自动识别返回检测范围 roi。
def get_detect_roi(img, manual_points=None):
    if manual_points is None:
        range_points = auto_range_points(img)
    else:
        range_points = manual_points

    if range_points is None:
        return None
    return points_to_roi(img, range_points)
