# K230D 40Pin UART2 link test.
#
# This script does not start the camera or vision detector. It only sends a
# valid APPROACH_TARGET frame with valid=0 every 100 ms, so STM32 `VI` should
# show both rx_bytes and frames increasing while VA remains off.

import struct
import time

from machine import UART, FPIOA

MSG_APPROACH_TARGET = 0x07

HEAD = b"\xAA\x55"
TAIL = 0x0D


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(payload):
    length = bytes([len(payload)])
    checksum = crc16(length + payload)
    return HEAD + length + payload + struct.pack("<H", checksum) + bytes([TAIL])


def main():
    fpioa = FPIOA()
    # Lite-K230D 40Pin: Pin 11/GPIO05 TX, Pin 13/GPIO06 RX.
    fpioa.set_function(5, fpioa.UART2_TXD)
    fpioa.set_function(6, fpioa.UART2_RXD)
    uart = UART(
        UART.UART2,
        baudrate=921600,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )

    # valid=0: exercises the UART/parser path without commanding motion.
    payload = struct.pack("<BhhBB", MSG_APPROACH_TARGET, 0, 0, 0, 0)
    frame = encode(payload)
    sent = 0
    try:
        print("UART2 link test: GPIO05 TX -> PA3, GPIO06 RX <- PA2")
        while True:
            uart.write(frame)
            sent += 1
            if sent % 10 == 0:
                print("sent frames:", sent)
            time.sleep_ms(100)
    except KeyboardInterrupt:
        print("stopped, sent:", sent)
    finally:
        try:
            uart.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
