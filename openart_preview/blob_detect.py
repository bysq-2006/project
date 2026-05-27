# 过滤掉明显不合格的色块。
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


# 找出某一个目标颜色下的有效色块。
def find_target_blobs(img, target, max_blobs_per_color, roi=None):
    name, thresholds, target_color, min_pixels, should_split = target
    if roi is None:
        blobs = img.find_blobs(thresholds,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=False)
    else:
        blobs = img.find_blobs(thresholds,
                               roi=roi,
                               pixels_threshold=min_pixels,
                               area_threshold=min_pixels,
                               merge=False)
    blobs = [blob for blob in blobs if good_blob(blob)]
    blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

    detections = []
    for blob in blobs[:max_blobs_per_color]:
        detections.append((name, blob, target_color, should_split))
    return detections


# 依次检测所有目标并汇总结果。
def detect_targets(img, targets, max_blobs_per_color, roi=None):
    detections = []
    for target in targets:
        detections.extend(find_target_blobs(img, target, max_blobs_per_color, roi))
    return detections
