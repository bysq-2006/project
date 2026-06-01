import time

from map_detect.grid_classifier import classify_grid
from map_detect.point_locator import get_detect_rois


def _empty_votes(rows, cols):
    votes = []
    for _ in range(rows):
        row_votes = []
        for _ in range(cols):
            row_votes.append({})
        votes.append(row_votes)
    return votes


def _add_grid_votes(votes, grid_map, rows, cols):
    for row in range(rows):
        for col in range(cols):
            cell = grid_map[row][col]
            cell_votes = votes[row][col]
            cell_votes[cell] = cell_votes.get(cell, 0) + 1


def _vote_grid(votes, rows, cols, fallback_name):
    grid_map = []
    for row in range(rows):
        grid_row = []
        for col in range(cols):
            best_name = fallback_name
            best_count = 0
            for name, count in votes[row][col].items():
                if count > best_count:
                    best_name = name
                    best_count = count
            grid_row.append(best_name)
        grid_map.append(grid_row)
    return grid_map


def _add_roi_sum(roi_sum, roi):
    for i in range(4):
        roi_sum[i] += roi[i]


def _average_roi(roi_sum, count):
    if count <= 0:
        return None
    return tuple((roi_sum[i] + count // 2) // count for i in range(4))


def _valid_grid_size(grid_map, rows, cols):
    if len(grid_map) != rows:
        return False
    for row in grid_map:
        if len(row) != cols:
            return False
    return True


def build_confident_map(snapshot_func, config):
    cols = config["cols"]
    rows = config["rows"]
    fallback = config["fallback"]

    if not config.get("enabled", True):
        return None, None, None

    votes = _empty_votes(rows, cols)
    base_roi_sum = [0, 0, 0, 0]
    detect_roi_sum = [0, 0, 0, 0]
    sample_count = 0
    start_ms = time.ticks_ms()

    while True:
        img = snapshot_func()
        base_roi, detect_roi = get_detect_rois(img, config)
        if detect_roi is not None:
            grid_map = classify_grid(img, detect_roi,
                                     cols, rows,
                                     config["color_ratios"],
                                     fallback)
            if _valid_grid_size(grid_map, rows, cols):
                _add_grid_votes(votes, grid_map, rows, cols)
                _add_roi_sum(base_roi_sum, base_roi)
                _add_roi_sum(detect_roi_sum, detect_roi)
                sample_count += 1

        if time.ticks_diff(time.ticks_ms(), start_ms) >= config["sample_ms"]:
            break

    if sample_count <= 0:
        return None, None, None

    return (_average_roi(base_roi_sum, sample_count),
            _average_roi(detect_roi_sum, sample_count),
            _vote_grid(votes, rows, cols, fallback))
