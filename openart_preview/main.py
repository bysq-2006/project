import time
from machine import Pin
import cmm

from openart_uart import init_uart, receive_packet


def make_pin(name):
    try:
        return Pin(name)
    except Exception:
        return None


def load_cmm_config():
    cmm.add({
        "hw.-": ("rt117x", "seekfree_art_plus", None, None),
        "uart.12.TXD": ("-", "LPSR_06", make_pin("LPSR_06"), None),
        "uart.12.RXD": ("-", "LPSR_07", make_pin("LPSR_07"), None),
    })


load_cmm_config()
init_uart()
print("uart receive test start")

while True:
    packet = receive_packet()
    if packet is not None:
        packet_type, payload = packet
        print("recv:", packet_type, list(payload))
    time.sleep_ms(10)
