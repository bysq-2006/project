# OpenART Plus low-res UART image preview.
# UART packet: AA 55 49 4D, width, height, frame_id_l, frame_id_h, checksum_l, checksum_h, image bytes.

import sensor
import time
from machine import Pin
from machine import UART
import cmm


FRAME_W = 80
FRAME_H = 60
SAMPLE_STEP = 2
BAUD = 115200
UART_ID = 12
HEADER = b"\xAA\x55IM"


def make_pin(name):
    try:
        return Pin(name)
    except Exception:
        return None


def load_cmm_config():
    pin_map = {
        "hw.-": ("rt117x", "seekfree_art_plus", None, None),
        "pin.J25": ("-", "AD_26", make_pin("AD_26"), None),
        "pin.J26": ("-", "AD_27", make_pin("AD_27"), None),
        "pin.J27": ("-", "AD_28", make_pin("AD_28"), None),
        "pin.J28": ("-", "AD_29", make_pin("AD_29"), None),
        "pin.M12": ("-", "LPSR_12", make_pin("LPSR_12"), None),
        "pin.M11": ("-", "LPSR_11", make_pin("LPSR_11"), None),
        "pin.M10": ("-", "LPSR_10", make_pin("LPSR_10"), None),
        "pin.M9": ("-", "LPSR_09", make_pin("LPSR_09"), None),
        "pin.M4": ("-", "LPSR_04", make_pin("LPSR_04"), None),
        "pin.M5": ("-", "LPSR_05", make_pin("LPSR_05"), None),
        "pin.J6": ("-", "AD_07", make_pin("AD_07"), None),
        "uart.12.TXD": ("-", "LPSR_06", make_pin("LPSR_06"), None),
        "uart.12.RXD": ("-", "LPSR_07", make_pin("LPSR_07"), None),
    }
    cmm.add(pin_map)
    print("cmm config ok")


def open_uart():
    try:
        uart = UART(UART_ID, BAUD, timeout_char=1000)
    except TypeError:
        uart = UART(UART_ID)
        uart.init(BAUD, bits=8, parity=None, stop=1, timeout_char=1000)
    print("uart ok:", UART_ID, BAUD)
    return uart


def append_u16_le(buffer, value):
    buffer.append(value & 0xFF)
    buffer.append((value >> 8) & 0xFF)


load_cmm_config()
uart = open_uart()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1000)

clock = time.clock()
frame_id = 0
frame = bytearray(FRAME_W * FRAME_H)

while True:
    clock.tick()
    img = sensor.snapshot()

    index = 0
    checksum = 0
    for y in range(FRAME_H):
        src_y = y * SAMPLE_STEP
        for x in range(FRAME_W):
            value = img.get_pixel(x * SAMPLE_STEP, src_y)
            frame[index] = value
            checksum = (checksum + value) & 0xFFFF
            index += 1

    packet = bytearray(HEADER)
    packet.append(FRAME_W)
    packet.append(FRAME_H)
    append_u16_le(packet, frame_id)
    append_u16_le(packet, checksum)

    uart.write(packet)
    uart.write(frame)

    frame_id = (frame_id + 1) & 0xFFFF
    time.sleep_ms(200)
