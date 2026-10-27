# K230D 40Pin UART3 link test.
#
# Wiring:
#   K230 Pin 37 / GPIO32 / UART3_TXD -> STM32 PA3 / USART2_RX
#   K230 Pin 40 / GPIO33 / UART3_RXD <- STM32 PA2 / USART2_TX
#   K230 Pin 39 / GND               <-> STM32 GND
#
# The frame has valid=0, so it exercises the link without moving the chassis.

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
    fpioa.set_function(
        32, fpioa.UART3_TXD, ie=1, oe=1, pu=1, pd=0, ds=7
    )
    fpioa.set_function(
        33, fpioa.UART3_RXD, ie=1, oe=0, pu=1, pd=0, ds=7
    )
    uart = UART(
        UART.UART3,
        baudrate=921600,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )

    payload = struct.pack("<BhhBB", MSG_APPROACH_TARGET, 0, 0, 0, 0)
    frame = encode(payload)
    sent = 0
    try:
        print("UART3 link test: Pin37 TX -> PA3, Pin40 RX <- PA2")
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
