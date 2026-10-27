#!/usr/bin/env python3
"""Run the final 480x480 ONNX wallet detector on the PC for quick checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
ONNX_PATH = ROOT / "checkpoints" / "wallet_yolov8n_480.onnx"
SOURCE_DIR = ROOT / "dataset" / "images" / "val"
OUTPUT_DIR = ROOT / "predictions_pc"
IMAGE_SIZE = 480
CONF_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
LABEL = "wallet"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=Path, default=ONNX_PATH)
    parser.add_argument("--source", type=Path, default=SOURCE_DIR)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--confidence", type=float, default=CONF_THRESHOLD)
    parser.add_argument("--nms", type=float, default=NMS_THRESHOLD)
    return parser.parse_args()


def preprocess(image: Image.Image) -> np.ndarray:
    resized = image.convert("RGB").resize(
        (IMAGE_SIZE, IMAGE_SIZE), Image.Resampling.BILINEAR
    )
    data = np.asarray(resized, dtype=np.float32) / 255.0
    return data.transpose(2, 0, 1)[None, ...]


def iou(box_a: list[float], box_b: list[float]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def nms(
    boxes: list[list[float]],
    scores: list[float],
    threshold: float,
) -> list[int]:
    order = sorted(range(len(scores)), key=lambda index: scores[index], reverse=True)
    keep: list[int] = []
    while order:
        current = order.pop(0)
        keep.append(current)
        order = [
            index
            for index in order
            if iou(boxes[current], boxes[index]) <= threshold
        ]
    return keep


def decode(
    output: np.ndarray,
    frame_width: int,
    frame_height: int,
    confidence: float,
    nms_threshold: float,
) -> list[dict]:
    prediction = output[0]
    if prediction.shape[0] == 5:
        boxes_ori = prediction[:4, :]
        scores_ori = prediction[4, :]
        boxes_first = False
    elif prediction.shape[1] == 5:
        boxes_ori = prediction[:, :4]
        scores_ori = prediction[:, 4]
        boxes_first = True
    else:
        raise ValueError(f"Unexpected output shape: {prediction.shape}")

    candidates: list[tuple[list[float], float]] = []
    for index, score in enumerate(scores_ori):
        if float(score) < confidence:
            continue
        if boxes_first:
            x, y, width, height = [float(value) for value in boxes_ori[index]]
        else:
            x, y, width, height = [float(value) for value in boxes_ori[:, index]]
        x1 = max(0.0, (x - width * 0.5) * frame_width / IMAGE_SIZE)
        y1 = max(0.0, (y - height * 0.5) * frame_height / IMAGE_SIZE)
        x2 = min(float(frame_width), (x + width * 0.5) * frame_width / IMAGE_SIZE)
        y2 = min(float(frame_height), (y + height * 0.5) * frame_height / IMAGE_SIZE)
        if x2 > x1 and y2 > y1:
            candidates.append(([x1, y1, x2, y2], float(score)))

    if not candidates:
        return []
    candidates.sort(key=lambda item: item[1], reverse=True)
    candidates = candidates[:100]
    boxes = [item[0] for item in candidates]
    scores = [item[1] for item in candidates]
    keep = nms(boxes, scores, nms_threshold)
    return [
        {"box": [round(value, 2) for value in boxes[index]], "score": round(scores[index], 5)}
        for index in keep
    ]


def draw(image: Image.Image, detections: list[dict]) -> Image.Image:
    result = image.convert("RGB").copy()
    draw_context = ImageDraw.Draw(result)
    for detection in detections:
        x1, y1, x2, y2 = detection["box"]
        draw_context.rectangle((x1, y1, x2, y2), outline=(255, 0, 0), width=4)
        draw_context.text(
            (x1, max(0, y1 - 16)),
            f"{LABEL} {detection['score']:.3f}",
            fill=(255, 0, 0),
        )
    return result


def main() -> int:
    args = parse_args()
    image_paths = sorted(
        path
        for path in args.source.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    )
    if not image_paths:
        raise SystemExit(f"No images found under {args.source}")
    if not args.onnx.is_file():
        raise SystemExit(f"ONNX model not found: {args.onnx}")

    args.output.mkdir(parents=True, exist_ok=True)
    session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    all_results: dict[str, list[dict]] = {}

    for image_path in image_paths:
        image = Image.open(image_path)
        tensor = preprocess(image)
        output = session.run(None, {input_name: tensor})[0]
        detections = decode(
            output,
            image.width,
            image.height,
            args.confidence,
            args.nms,
        )
        all_results[image_path.name] = detections
        draw(image, detections).save(args.output / f"{image_path.stem}_pred.jpg", quality=92)
        print(
            f"{image_path.name}: {len(detections)} detection(s), "
            f"top={detections[0]['score'] if detections else 0:.4f}"
        )

    (args.output / "predictions.json").write_text(
        json.dumps(all_results, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
