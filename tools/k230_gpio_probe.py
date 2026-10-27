#!/usr/bin/env python3
"""Find which K230 GPIO drives STM32 PA3.

This is a bench diagnostic. It momentarily drives each candidate K230 GPIO
high and reads GPIOA.IDR on STM32 through PWLINK2. Every tested pin is returned
to high-impedance input mode immediately.
"""

from __future__ import annotations

import time

import serial
from pyocd.core.helpers import ConnectHelper


K230_PORT = "COM10"
K230_BAUD = 115200

PROBE_ID = "B0921D28FC440ED079A8BA25068D10B6"
TARGET = "stm32g474vetx"
PACK = "D:/keil/.Download/Keil.STM32G4xx_DFP.1.5.0.pack"
GPIOA_IDR = 0x48000010
PA3_MASK = 1 << 3

# 40Pin GPIO-capable pins excluding power/ground, QSPI flash pins, and the
# USB OTG/HDMI control pin.
CANDIDATES = [
    (2, 7), (3, 3), (4, 10), (5, 11), (6, 13), (10, 22),
    (20, 12), (32, 37), (33, 40), (34, 38), (35, 36),
    (36, 29), (37, 31), (40, 28), (41, 27), (48, 5),
    (49, 3), (59, 33), (60, 35), (62, 26), (63, 32),
]


def send_command(port: serial.Serial, command: str, wait: float = 0.18) -> str:
    port.reset_input_buffer()
    port.write((command + "\r").encode())
    port.flush()
    time.sleep(wait)
    return port.read(4096).decode("utf-8", "replace")


def main() -> int:
    k230 = serial.Serial(K230_PORT, K230_BAUD, timeout=0.2)
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=PROBE_ID,
        target_override=TARGET,
        pack=PACK,
    )
    session.open()
    target = session.target

    found = []
    try:
        k230.write(b"\x03\r\n")
        time.sleep(0.3)
        k230.reset_input_buffer()
        baseline = target.read32(GPIOA_IDR)
        print(f"baseline GPIOA_IDR={baseline:08x}")

        for gpio, physical_pin in CANDIDATES:
            set_high = (
                f"f.set_function({gpio},getattr(f,'GPIO{gpio}'),"
                "ie=1,oe=1,pu=0,pd=0,ds=7); "
                f"gp=machine.Pin({gpio},machine.Pin.OUT,"
                "pull=machine.Pin.PULL_NONE,drive=7); gp.value(1)"
            )
            set_safe = (
                f"gp=machine.Pin({gpio},machine.Pin.IN,"
                "pull=machine.Pin.PULL_NONE)"
            )
            send_command(k230, set_high)
            time.sleep(0.12)
            value = target.read32(GPIOA_IDR)
            is_high = bool(value & PA3_MASK)
            print(
                f"K230 Pin {physical_pin:02d} / GPIO{gpio:02d}: "
                f"GPIOA_IDR={value:08x} PA3={'HIGH' if is_high else 'low'}"
            )
            if is_high:
                found.append((physical_pin, gpio))
            send_command(k230, set_safe)
    finally:
        for gpio, _ in CANDIDATES:
            try:
                send_command(
                    k230,
                    f"gp=machine.Pin({gpio},machine.Pin.IN,"
                    "pull=machine.Pin.PULL_NONE)",
                    wait=0.02,
                )
            except Exception:
                pass
        session.close()
        k230.close()

    if found:
        print("FOUND:", ", ".join(f"Pin {pin}/GPIO{gpio}" for pin, gpio in found))
    else:
        print("FOUND: none")
    return 0 if found else 2


if __name__ == "__main__":
    raise SystemExit(main())
