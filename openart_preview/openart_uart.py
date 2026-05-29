import time
from machine import UART


UART_ID = 12
UART_BAUD = 115200
DEBUG_PRINT = True
DEBUG_PRINT_INTERVAL_MS = 1000

PACKET_HEADER = b"\xAA\x55"
PACKET_TYPE_POSE = b"PO"
PACKET_TYPE_MAP = b"MP"
POSE_SCALE = 10
MAP_SIZE_SCALE = 10

CELL_CODE = {
    "background": 0,
    "wall": 1,
    "goal": 2,
    "yellow_box": 3,
    "unknown": 255,
}

_pose_uart = None
_packet_seq = 0
_debug_last_ms = 0
_debug_tx_count = 0
_debug_pose_count = 0
_debug_map_count = 0
_debug_write_fail_count = 0
_debug_last_type = "--"
_debug_last_len = 0
_debug_last_write = 0
_debug_pose_text = "pose:none"
_debug_map_text = "map:none"


def init_uart(uart_id=UART_ID, baudrate=UART_BAUD):
    global _pose_uart

    if _pose_uart is not None:
        return _pose_uart

    try:
        _pose_uart = UART(uart_id, baudrate)
    except TypeError:
        _pose_uart = UART(uart_id)
        try:
            _pose_uart.init(baudrate, bits=8, parity=None, stop=1)
        except TypeError:
            _pose_uart.init(baudrate)

    if DEBUG_PRINT:
        print("openart uart init ok id:", uart_id, "baud:", baudrate)
    return _pose_uart


def _put_u16(buf, value):
    value &= 0xFFFF
    buf.append(value & 0xFF)
    buf.append((value >> 8) & 0xFF)


def _put_i16(buf, value):
    _put_u16(buf, value)


def _scaled_i16(value):
    value = int(value * POSE_SCALE)
    if value > 32767:
        value = 32767
    elif value < -32768:
        value = -32768
    return value


def _scaled_u16(value, scale=POSE_SCALE):
    value = int(value * scale)
    if value < 0:
        value = 0
    elif value > 65535:
        value = 65535
    return value


def _debug_maybe_print():
    global _debug_last_ms

    if not DEBUG_PRINT:
        return

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, _debug_last_ms) < DEBUG_PRINT_INTERVAL_MS:
        return
    _debug_last_ms = now_ms

    print("openart tx total:", _debug_tx_count,
          "pose:", _debug_pose_count,
          "map:", _debug_map_count,
          "fail:", _debug_write_fail_count,
          "last:", _debug_last_type,
          "payload:", _debug_last_len,
          "write:", _debug_last_write)
    print(_debug_pose_text)
    print(_debug_map_text)


def _debug_record(packet_type, payload_len, write_result):
    global _debug_tx_count, _debug_write_fail_count
    global _debug_last_type, _debug_last_len, _debug_last_write

    _debug_tx_count += 1
    _debug_last_type = packet_type.decode() if hasattr(packet_type, "decode") else str(packet_type)
    _debug_last_len = payload_len
    _debug_last_write = write_result
    if write_result is None or write_result <= 0:
        _debug_write_fail_count += 1
    _debug_maybe_print()


def _send_packet(packet_type, payload):
    uart = init_uart()
    packet = bytearray()

    packet.extend(PACKET_HEADER)
    packet.extend(packet_type)
    _put_u16(packet, len(payload))
    packet.extend(payload)

    checksum = 0
    for byte in packet[2:]:
        checksum = (checksum + byte) & 0xFFFF
    _put_u16(packet, checksum)

    return uart.write(packet)


def build_car_pose_payload(valid, map_x, map_y, angle_deg):
    global _packet_seq

    payload = bytearray()
    payload.append(_packet_seq & 0xFF)
    payload.append(1 if valid else 0)

    if valid:
        _put_i16(payload, _scaled_i16(map_x))
        _put_i16(payload, _scaled_i16(map_y))
        _put_u16(payload, _scaled_u16(angle_deg))
    else:
        _put_i16(payload, 0)
        _put_i16(payload, 0)
        _put_u16(payload, 0)

    _packet_seq = (_packet_seq + 1) & 0xFF
    return payload


def send_car_pose(valid, map_x, map_y, angle_deg):
    global _debug_pose_count, _debug_pose_text

    payload = build_car_pose_payload(valid, map_x, map_y, angle_deg)
    write_result = _send_packet(PACKET_TYPE_POSE, payload)
    _debug_pose_count += 1
    if valid:
        _debug_pose_text = ("pose valid:1 seq:%d x10:%d y10:%d angle10:%d" %
                            (payload[0],
                             _scaled_i16(map_x),
                             _scaled_i16(map_y),
                             _scaled_u16(angle_deg)))
    else:
        _debug_pose_text = "pose valid:0 seq:%d" % payload[0]
    _debug_record(PACKET_TYPE_POSE, len(payload), write_result)
    return write_result


def send_detected_car(car_map_pos, angle_deg):
    if car_map_pos is None or angle_deg is None:
        return send_car_pose(False, 0, 0, 0)
    return send_car_pose(True, car_map_pos[0], car_map_pos[1], angle_deg)


def _grid_size(grid_map, default_cols, default_rows):
    rows = len(grid_map) if grid_map else default_rows
    cols = len(grid_map[0]) if grid_map and rows > 0 else default_cols
    return cols, rows


def build_map_payload(valid, grid_map, map_width, map_height, default_cols, default_rows):
    global _packet_seq

    cols, rows = _grid_size(grid_map, default_cols, default_rows)

    payload = bytearray()
    payload.append(_packet_seq & 0xFF)
    payload.append(1 if valid else 0)
    payload.append(cols & 0xFF)
    payload.append(rows & 0xFF)
    _put_u16(payload, _scaled_u16(map_width, MAP_SIZE_SCALE) if valid else 0)
    _put_u16(payload, _scaled_u16(map_height, MAP_SIZE_SCALE) if valid else 0)

    if valid and grid_map:
        for row in grid_map:
            for cell in row:
                payload.append(CELL_CODE.get(cell, 255))

    _packet_seq = (_packet_seq + 1) & 0xFF
    return payload


def send_map(valid, grid_map, map_width, map_height, default_cols, default_rows, debug_reason=""):
    global _debug_map_count, _debug_map_text

    payload = build_map_payload(valid, grid_map, map_width, map_height, default_cols, default_rows)
    write_result = _send_packet(PACKET_TYPE_MAP, payload)
    _debug_map_count += 1
    if valid:
        cols, rows = _grid_size(grid_map, default_cols, default_rows)
        _debug_map_text = ("map valid:1 seq:%d cols:%d rows:%d w10:%d h10:%d cells:%d" %
                           (payload[0],
                            cols,
                            rows,
                            _scaled_u16(map_width, MAP_SIZE_SCALE),
                            _scaled_u16(map_height, MAP_SIZE_SCALE),
                            cols * rows))
    else:
        cols, rows = _grid_size(grid_map, default_cols, default_rows)
        if debug_reason:
            _debug_map_text = ("map valid:0 seq:%d cols:%d rows:%d reason:%s" %
                               (payload[0], cols, rows, debug_reason))
        else:
            _debug_map_text = "map valid:0 seq:%d cols:%d rows:%d" % (payload[0], cols, rows)
    _debug_record(PACKET_TYPE_MAP, len(payload), write_result)
    return write_result


def send_detected_map(grid_map, map_roi, default_cols, default_rows):
    if map_roi is None:
        return send_map(False, None, 0, 0, default_cols, default_rows, "no_roi")
    if not grid_map:
        return send_map(False, None, 0, 0, default_cols, default_rows, "no_grid")
    return send_map(True, grid_map, map_roi[2], map_roi[3], default_cols, default_rows)
