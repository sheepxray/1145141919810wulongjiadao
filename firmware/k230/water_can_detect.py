# CanMV K230D - blue-and-white water-can detector
#
# This is a classical vision baseline for an Aquafina-style can.
# It uses LAB color blobs plus tall/solid shape filtering. It does not need
# YOLO. The ROI and LAB threshold must be checked against the real camera.

import os
import time
import gc
import math

from media.sensor import *
from media.display import *
from media.media import *

__all__ = ["detect_water_can", "measure_can_blob", "is_water_can"]

FRAME_W = 640
FRAME_H = 480

# LAB thresholds: (L_min, L_max, A_min, A_max, B_min, B_max).
# The narrow threshold was verified against the current K230D camera. It
# rejects the bright wall while keeping the shaded can as one connected blob.
# The wider threshold is a fallback for brighter or more distant scenes.
BLUE_CAN_THRESHOLDS = [
    (5, 30, -20, 25, -35, -2),
    (5, 40, -25, 30, -45, -2),
]

# Search ROI: x, y, width, height. Keep it around the expected object area.
ROI = (
    int(FRAME_W * 0.12),
    int(FRAME_H * 0.02),
    int(FRAME_W * 0.76),
    int(FRAME_H * 0.72),
)

PIXELS_THRESHOLD = 800
AREA_THRESHOLD = 800
MERGE = True
MERGE_MARGIN = 12

MIN_AREA_FRAC = 0.008
MAX_AREA_FRAC = 0.75
MIN_WIDTH_FRAC = 0.06
MIN_HEIGHT_FRAC = 0.15
MIN_ASPECT = 1.2
MAX_ASPECT = 5.0
MIN_SOLIDITY = 0.25
MIN_FILL = 0.25

COLOR_BOX = (0, 255, 0)
COLOR_CENTER = (255, 0, 0)
MAX_PRINT = 8


def measure_can_blob(blob):
    width = blob.w()
    height = blob.h()
    area = blob.area()
    aspect = max(width / height, height / width) if width and height else 0.0
    box_area = float(width * height)
    fill = area / box_area if box_area else 0.0

    return {
        "x": blob.x(),
        "y": blob.y(),
        "w": width,
        "h": height,
        "cx": blob.cx(),
        "cy": blob.cy(),
        "pixels": blob.pixels(),
        "area": area,
        "aspect": aspect,
        "solidity": blob.solidity(),
        "fill": fill,
    }


def is_water_can(measurement, frame_w=FRAME_W, frame_h=FRAME_H):
    frame_area = float(frame_w * frame_h)
    area_fraction = measurement["area"] / frame_area
    if not MIN_AREA_FRAC <= area_fraction <= MAX_AREA_FRAC:
        return False
    if measurement["w"] < frame_w * MIN_WIDTH_FRAC:
        return False
    if measurement["h"] < frame_h * MIN_HEIGHT_FRAC:
        return False
    if not MIN_ASPECT <= measurement["aspect"] <= MAX_ASPECT:
        return False
    if measurement["solidity"] < MIN_SOLIDITY:
        return False
    if measurement["fill"] < MIN_FILL:
        return False
    return True


def _can_score(measurement, frame_w=FRAME_W, frame_h=FRAME_H):
    area_fraction = measurement["area"] / float(frame_w * frame_h)
    shape_score = min(measurement["aspect"] / 3.0, 1.0)
    return area_fraction * 2.0 + shape_score * 0.6 + measurement["solidity"] * 0.4


def detect_water_can(img, draw=True, verbose=False):
    """Return the best water-can candidate, or None."""
    frame_w = img.width()
    frame_h = img.height()
    candidates = []

    for threshold_index, threshold in enumerate(BLUE_CAN_THRESHOLDS):
        blobs = img.find_blobs(
            [threshold],
            roi=ROI,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=MERGE,
            margin=MERGE_MARGIN,
        )

        for index, blob in enumerate(blobs):
            measurement = measure_can_blob(blob)
            accepted = is_water_can(measurement, frame_w, frame_h)
            if verbose and index < MAX_PRINT:
                print(
                    "threshold=%d candidate area=%d aspect=%.2f "
                    "solid=%.2f fill=%.2f %s"
                    % (
                        threshold_index,
                        measurement["area"],
                        measurement["aspect"],
                        measurement["solidity"],
                        measurement["fill"],
                        "OK" if accepted else "--",
                    )
                )
            if accepted:
                measurement["score"] = _can_score(measurement, frame_w, frame_h)
                measurement["threshold_index"] = threshold_index
                measurement["_blob"] = blob
                candidates.append(measurement)

        # The verified narrow threshold is both cleaner and faster. Only try
        # the wider threshold when the primary pass found nothing.
        if candidates:
            break

    if not candidates:
        return None

    best = max(candidates, key=lambda item: item["score"])
    blob = best.pop("_blob")
    best["label"] = "water_can"

    # Normalized horizontal/vertical error in [-1, 1]. This is convenient
    # for visual servoing without camera calibration.
    best["nx"] = (best["cx"] - frame_w / 2.0) / (frame_w / 2.0)
    best["ny"] = (best["cy"] - frame_h / 2.0) / (frame_h / 2.0)
    best["angle_deg"] = math.degrees(math.atan2(best["nx"], 1.0))

    if draw:
        img.draw_rectangle(
            best["x"],
            best["y"],
            best["w"],
            best["h"],
            color=COLOR_BOX,
            thickness=2,
        )
        img.draw_cross(
            best["cx"],
            best["cy"],
            color=COLOR_CENTER,
            size=8,
        )
        img.draw_string_advanced(
            best["x"],
            max(0, best["y"] - 18),
            16,
            "water-can",
            color=COLOR_BOX,
        )

    return best


def main():
    sensor = None
    try:
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

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()
            target = detect_water_can(img, draw=True)

            if target:
                print(
                    "water_can cx=%d cy=%d w=%d h=%d nx=%.3f score=%.3f fps=%.1f"
                    % (
                        target["cx"],
                        target["cy"],
                        target["w"],
                        target["h"],
                        target["nx"],
                        target["score"],
                        clock.fps(),
                    )
                )
            else:
                print("no water can fps=%.1f" % clock.fps())

            Display.show_image(img)
            img = None
            gc.collect()

    except KeyboardInterrupt as exc:
        print("stopped:", exc)
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
