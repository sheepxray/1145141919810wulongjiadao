#!/usr/bin/env python3
"""Validate the K230 approach frame encoder against tools/protocol.py."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def install_stubs() -> None:
    machine = types.ModuleType("machine")

    class UART:
        UART2 = 2
        UART3 = 3
        EIGHTBITS = 8
        PARITY_NONE = 0
        STOPBITS_ONE = 1

        def __init__(self, *args, **kwargs):
            pass

    class FPIOA:
        UART2_TXD = 1
        UART2_RXD = 2
        UART3_TXD = 3
        UART3_RXD = 4

        def set_function(self, pin, function):
            pass

    machine.UART = UART
    machine.FPIOA = FPIOA
    sys.modules["machine"] = machine

    media = types.ModuleType("media")
    sensor = types.ModuleType("media.sensor")
    display = types.ModuleType("media.display")
    media_module = types.ModuleType("media.media")
    for name in ("Sensor", "Display", "MediaManager"):
        setattr(sensor, name, type(name, (), {}))
        setattr(display, name, type(name, (), {}))
        setattr(media_module, name, type(name, (), {}))
    media.sensor = sensor
    media.display = display
    media.media = media_module
    sys.modules["media"] = media
    sys.modules["media.sensor"] = sensor
    sys.modules["media.display"] = display
    sys.modules["media.media"] = media_module

    detector = types.ModuleType("water_can_detect")
    detector.FRAME_W = 640
    detector.FRAME_H = 480
    detector.detect_water_can = lambda *args, **kwargs: None
    sys.modules["water_can_detect"] = detector


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    install_stubs()
    k230 = load_module(
        "vision_approach_test",
        ROOT / "firmware" / "k230" / "vision_approach.py",
    )
    protocol = load_module("protocol_test", ROOT / "tools" / "protocol.py")

    cases = [
        (-125, 430, 88, 1),
        (0, 0, 0, 0),
        (1000, 700, 100, 1),
        (-1000, 120, 40, 1),
    ]
    for case in cases:
        frame = k230.make_approach_target(*case)
        parser = protocol.Parser()
        messages = parser.feed_bytes(frame)
        assert len(messages) == 1
        msg_id, payload, raw = messages[0]
        assert raw == frame
        assert msg_id == protocol.MSG_APPROACH_TARGET
        parsed = protocol.parse_approach_target(payload)
        assert parsed["err_pm"] == case[0]
        assert parsed["size_pm"] == case[1]
        assert parsed["conf"] == case[2]
        assert parsed["valid"] is bool(case[3])

    print("vision approach frame round-trip: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
