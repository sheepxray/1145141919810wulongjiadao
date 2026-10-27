#!/usr/bin/env python3
"""Detect a blue-and-white water can with classical computer vision.

This is a desktop prototype for tuning before porting the idea to CanMV.
It uses color segmentation, connected regions, and can-like shape filtering.
It does not require a trained YOLO model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def imread_unicode(path: Path) -> np.ndarray | None:
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_unicode(path: Path, image: np.ndarray) -> None:
    suffix = path.suffix or ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise OSError(f"Failed to encode image: {path}")
    path.write_bytes(encoded.tobytes())


def parse_roi(value: str) -> tuple[float, float, float, float]:
    parts = [float(part) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("ROI must be x1,y1,x2,y2")
    if not all(0.0 <= part <= 1.0 for part in parts):
        raise argparse.ArgumentTypeError("ROI values must be between 0 and 1")
    return tuple(parts)


def odd_kernel(value: int) -> int:
    value = max(3, value)
    return value if value % 2 == 1 else value + 1


def build_blue_mask(
    image: np.ndarray,
    roi: tuple[float, float, float, float],
) -> np.ndarray:
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)

    # Dark blue ink and pale cyan body. The saturation floor keeps most
    # neutral backgrounds out, while the ROI removes the desk edges.
    mask = (
        (hue >= 85)
        & (hue <= 145)
        & (saturation >= 18)
        & (value >= 50)
    ).astype(np.uint8) * 255

    x1, y1, x2, y2 = roi
    roi_mask = np.zeros((height, width), dtype=np.uint8)
    roi_mask[int(y1 * height):int(y2 * height), int(x1 * width):int(x2 * width)] = 255
    mask = cv2.bitwise_and(mask, roi_mask)

    kernel = odd_kernel(int(min(height, width) * 0.04))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    return mask


def detect_water_can(
    image: np.ndarray,
    roi: tuple[float, float, float, float] = (0.18, 0.10, 0.80, 0.87),
) -> tuple[dict | None, np.ndarray]:
    height, width = image.shape[:2]
    frame_area = float(width * height)
    mask = build_blue_mask(image, roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates: list[dict] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        area_fraction = area / frame_area
        if not 0.015 <= area_fraction <= 0.75:
            continue

        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width < width * 0.08 or box_height < height * 0.20:
            continue

        aspect = max(box_width / box_height, box_height / box_width)
        if not 1.2 <= aspect <= 5.0:
            continue

        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / hull_area if hull_area > 0 else 0.0
        fill = area / float(box_width * box_height)
        if solidity < 0.35 or fill < 0.25:
            continue

        # Long, solid blue-white regions score higher than small blue patches.
        shape_score = min(aspect / 3.0, 1.0)
        score = area_fraction * 2.0 + shape_score * 0.6 + solidity * 0.4
        candidates.append(
            {
                "bbox": [int(x), int(y), int(box_width), int(box_height)],
                "center": [int(x + box_width / 2), int(y + box_height / 2)],
                "area_fraction": round(area_fraction, 5),
                "aspect": round(aspect, 3),
                "solidity": round(solidity, 3),
                "fill": round(fill, 3),
                "score": round(score, 4),
            }
        )

    if not candidates:
        return None, mask

    best = max(candidates, key=lambda item: item["score"])
    best["label"] = "water_can"
    return best, mask


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Input image path")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/water_can_detect"))
    parser.add_argument(
        "--roi",
        type=parse_roi,
        default=(0.18, 0.10, 0.80, 0.87),
        help="Search ROI as x1,y1,x2,y2 in normalized coordinates",
    )
    parser.add_argument("--show", action="store_true", help="Open a preview window")
    args = parser.parse_args()

    image = imread_unicode(args.image)
    if image is None:
        raise SystemExit(f"Cannot read image: {args.image}")

    detection, mask = detect_water_can(image, args.roi)
    annotated = image.copy()
    if detection is not None:
        x, y, width, height = detection["bbox"]
        center_x, center_y = detection["center"]
        cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 255, 0), 3)
        cv2.circle(annotated, (center_x, center_y), 7, (0, 0, 255), -1)
        cv2.putText(
            annotated,
            f"{detection['label']}  score={detection['score']:.2f}",
            (x, max(28, y - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    imwrite_unicode(args.output_dir / "annotated.jpg", annotated)
    imwrite_unicode(args.output_dir / "mask.png", mask)
    (args.output_dir / "detection.json").write_text(
        json.dumps(detection, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(json.dumps(detection, ensure_ascii=False, indent=2))
    if args.show:
        cv2.imshow("water can detection", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0 if detection is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
