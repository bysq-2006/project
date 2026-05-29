from machine import UART


UART_ID = 12
UART_BAUD = 115200

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
    payload = build_car_pose_payload(valid, map_x, map_y, angle_deg)
    return _send_packet(PACKET_TYPE_POSE, payload)


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


def send_map(valid, grid_map, map_width, map_height, default_cols, default_rows):
    payload = build_map_payload(valid, grid_map, map_width, map_height, default_cols, default_rows)
    return _send_packet(PACKET_TYPE_MAP, payload)


def send_detected_map(grid_map, map_roi, default_cols, default_rows):
    if map_roi is None:
        return send_map(False, None, 0, 0, default_cols, default_rows)
    if not grid_map:
        return send_map(False, None, 0, 0, default_cols, default_rows)
    return send_map(True, grid_map, map_roi[2], map_roi[3], default_cols, default_rows)
