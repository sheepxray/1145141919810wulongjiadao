# CanMV K230D - water-can visual approach transmitter.
#
# The K230 only estimates the target:
#   err_pm  : horizontal normalized error, -1000 left, +1000 right
#   size_pm : bounding-box height / frame height, 0..1000
#   conf    : heuristic confidence, 0..100
#   valid   : target present
#
# The STM32 owns the motion controller and may stop if frames time out.

import os
import sys
import time
import struct

from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

try:
    from water_can_detect import FRAME_W, FRAME_H, detect_water_can
except ImportError:
    raise RuntimeError(
        "water_can_detect.py is missing; save it to /sdcard with this file"
    )

UART_ID = UART.UART3
UART_BAUD = 921600
# Lite-K230D 40Pin header:
#   Pin 37 = GPIO32 = UART3_TXD
#   Pin 40 = GPIO33 = UART3_RXD
UART_TX_PIN = 32
UART_RX_PIN = 33

MSG_HELLO = 0x05
MSG_APPROACH_TARGET = 0x07

HEAD = b"\xAA\x55"
TAIL = 0x0D
SEND_PERIOD_MS = 50
TARGET_HOLD_MS = 250
ERR_SMOOTH_ALPHA = 0.35
SIZE_SMOOTH_ALPHA = 0.25


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


def make_hello():
    return encode(struct.pack("<BBBI", MSG_HELLO, 1, 0, 0))


def make_approach_target(err_pm, size_pm, conf, valid):
    err_pm = max(-1000, min(1000, int(err_pm)))
    size_pm = max(0, min(1000, int(size_pm)))
    conf = max(0, min(100, int(conf)))
    payload = struct.pack(
        "<BhhBB",
        MSG_APPROACH_TARGET,
        err_pm,
        size_pm,
        conf,
        1 if valid else 0,
    )
    return encode(payload)


def init_uart3():
    fpioa = FPIOA()
    fpioa.set_function(
        UART_TX_PIN,
        fpioa.UART3_TXD,
        ie=1,
        oe=1,
        pu=1,
        pd=0,
        ds=7,
    )
    fpioa.set_function(
        UART_RX_PIN,
        fpioa.UART3_RXD,
        ie=1,
        oe=0,
        pu=1,
        pd=0,
        ds=7,
    )
    return UART(
        UART_ID,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )


def main():
    sensor = None
    uart = None
    try:
        uart = init_uart3()
        uart.write(make_hello())

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()
        for _ in range(10):
            os.exitpoint()
            sensor.snapshot()

        last_send_ms = time.ticks_ms() - SEND_PERIOD_MS
        last_seen_ms = time.ticks_ms() - TARGET_HOLD_MS
        filtered_err = 0.0
        filtered_size = 0.0
        filtered_conf = 0.0
        have_filter = False
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()
            target = detect_water_can(img, draw=True)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_send_ms) >= SEND_PERIOD_MS:
                if target is not None:
                    raw_err = target["nx"] * 1000.0
                    raw_size = target["h"] * 1000.0 / FRAME_H
                    raw_conf = max(40, min(100, int(target["score"] * 55)))
                    if not have_filter:
                        filtered_err = raw_err
                        filtered_size = raw_size
                        filtered_conf = float(raw_conf)
                        have_filter = True
                    else:
                        filtered_err += (
                            raw_err - filtered_err
                        ) * ERR_SMOOTH_ALPHA
                        filtered_size += (
                            raw_size - filtered_size
                        ) * SIZE_SMOOTH_ALPHA
                        filtered_conf += (
                            raw_conf - filtered_conf
                        ) * 0.25
                    last_seen_ms = now
                    err_pm = int(filtered_err)
                    size_pm = int(filtered_size)
                    conf = max(40, min(100, int(filtered_conf)))
                    frame = make_approach_target(err_pm, size_pm, conf, True)
                    print(
                        "APPROACH err=%d size=%d conf=%d fps=%.1f"
                        % (err_pm, size_pm, conf, clock.fps())
                    )
                elif (
                    have_filter
                    and time.ticks_diff(now, last_seen_ms) <= TARGET_HOLD_MS
                ):
                    conf = max(20, int(filtered_conf) - 10)
                    frame = make_approach_target(
                        int(filtered_err),
                        int(filtered_size),
                        conf,
                        True,
                    )
                    print(
                        "APPROACH hold err=%d size=%d conf=%d fps=%.1f"
                        % (int(filtered_err), int(filtered_size), conf, clock.fps())
                    )
                else:
                    have_filter = False
                    frame = make_approach_target(0, 0, 0, False)
                    print("APPROACH lost fps=%.1f" % clock.fps())
                uart.write(frame)
                last_send_ms = now

            Display.show_image(img)
            img = None

    except KeyboardInterrupt as exc:
        print("stopped:", exc)
    except BaseException as exc:
        sys.print_exception(exc)
    finally:
        try:
            if uart is not None:
                uart.write(make_approach_target(0, 0, 0, False))
        except Exception:
            pass
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
