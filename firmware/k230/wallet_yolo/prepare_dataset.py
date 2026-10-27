#!/usr/bin/env python3
"""Prepare a YOLO detection dataset from the Label Studio export.

The uploaded archive contains YOLO-segmentation label text but no images.
The matching source images are in C:/Users/sheep/Desktop/data1123.  For a
K230 deployment, the polygon labels are converted to a single-class bounding
box dataset.
"""

from __future__ import annotations

import json
import math
import random
import re
import shutil
import zipfile
from pathlib import Path
from urllib.parse import unquote

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
ZIP_PATH = Path(r"C:\Users\sheep\Downloads\project-1-at-2026-10-23-23-42-9e46ba4d.zip")
IMAGE_SOURCE = Path(r"C:\Users\sheep\Desktop\data1123")
DATASET_DIR = ROOT / "dataset"
CLASS_NAME = "钱包"
CLASS_ID = 0
VAL_RATIO = 0.20
SPLIT_SEED = 42


def _decode_label_stem(entry_name: str) -> str:
    """Map a Label Studio label name to the source image stem."""
    decoded = unquote(Path(entry_name).stem)
    return re.sub(r"^[0-9a-fA-F]{8}-", "", decoded)


def _read_polygon_box(line: str) -> tuple[float, float, float, float] | None:
    parts = line.strip().split()
    if not parts:
        return None
    if len(parts) < 7 or (len(parts) - 1) % 2:
        raise ValueError(f"Invalid YOLO polygon line: {line[:120]!r}")

    class_id = int(float(parts[0]))
    if class_id != CLASS_ID:
        raise ValueError(
            f"Expected class {CLASS_ID} ({CLASS_NAME}), got {class_id}"
        )

    xs = [float(value) for value in parts[1::2]]
    ys = [float(value) for value in parts[2::2]]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)
    x1, x2 = max(0.0, x1), min(1.0, x2)
    y1, y2 = max(0.0, y1), min(1.0, y2)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("Degenerate polygon after conversion")
    return x1, y1, x2, y2


def _copy_image(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _write_box(label_path: Path, box: tuple[float, float, float, float]) -> None:
    x1, y1, x2, y2 = box
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    width = x2 - x1
    height = y2 - y1
    label_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.write_text(
        f"{CLASS_ID} {cx:.8f} {cy:.8f} {width:.8f} {height:.8f}\n",
        encoding="utf-8",
    )


def _make_preview(
    samples: list[dict],
    output_path: Path,
    thumb_size: tuple[int, int] = (320, 240),
) -> None:
    """Create a quick visual check of the generated boxes and split."""
    if not samples:
        return
    columns = 4
    rows = math.ceil(len(samples) / columns)
    canvas = Image.new("RGB", (columns * thumb_size[0], rows * thumb_size[1]), "white")
    draw = ImageDraw.Draw(canvas)

    for index, sample in enumerate(samples):
        image = Image.open(sample["image_path"]).convert("RGB")
        image.thumbnail(thumb_size)
        tile = Image.new("RGB", thumb_size, "white")
        offset_x = (thumb_size[0] - image.width) // 2
        offset_y = (thumb_size[1] - image.height) // 2
        tile.paste(image, (offset_x, offset_y))
        tile_draw = ImageDraw.Draw(tile)
        x1, y1, x2, y2 = sample["box"]
        scale_x = image.width
        scale_y = image.height
        rect = (
            offset_x + x1 * scale_x,
            offset_y + y1 * scale_y,
            offset_x + x2 * scale_x,
            offset_y + y2 * scale_y,
        )
        color = (255, 80, 80) if sample["split"] == "train" else (30, 140, 255)
        tile_draw.rectangle(rect, outline=color, width=3)
        tile_draw.text((4, 4), f"{sample['split']} wallet", fill=color)
        col = index % columns
        row = index // columns
        canvas.paste(tile, (col * thumb_size[0], row * thumb_size[1]))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, quality=92)


def main() -> int:
    if not ZIP_PATH.is_file():
        raise SystemExit(f"Label archive not found: {ZIP_PATH}")
    if not IMAGE_SOURCE.is_dir():
        raise SystemExit(f"Source image directory not found: {IMAGE_SOURCE}")

    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    source_images = {
        path.stem: path
        for path in IMAGE_SOURCE.iterdir()
        if path.is_file()
        and path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    }
    labels: dict[str, tuple[float, float, float, float]] = {}
    missing_images: list[str] = []
    empty_images: list[str] = []

    with zipfile.ZipFile(ZIP_PATH) as archive:
        label_entries = [
            entry
            for entry in archive.infolist()
            if entry.filename.startswith("labels/")
            and entry.filename.lower().endswith(".txt")
        ]
        packed_image_entries = [
            entry
            for entry in archive.infolist()
            if entry.filename.startswith("images/")
            and not entry.is_dir()
        ]

        for entry in label_entries:
            image_stem = _decode_label_stem(entry.filename)
            image_path = source_images.get(image_stem)
            if image_path is None:
                missing_images.append(image_stem)
                continue
            image_name = image_path.name

            text = archive.read(entry).decode("utf-8-sig")
            lines = [line for line in text.splitlines() if line.strip()]
            if not lines:
                empty_images.append(image_path.name)
                continue

            boxes = [_read_polygon_box(line) for line in lines]
            boxes = [box for box in boxes if box is not None]
            if len(boxes) != 1:
                raise ValueError(
                    f"{image_name}: expected exactly one wallet polygon, "
                    f"found {len(boxes)}"
                )
            labels[image_name] = boxes[0]

    if missing_images:
        raise SystemExit(f"Labels have no matching image: {missing_images[:5]}")
    if not labels:
        raise SystemExit(
            "No image/label pairs were created. "
            f"images/ in archive contains {len(packed_image_entries)} files."
        )

    image_names = sorted(labels)
    rng = random.Random(SPLIT_SEED)
    rng.shuffle(image_names)
    val_count = max(1, round(len(image_names) * VAL_RATIO))
    val_names = set(image_names[:val_count])

    samples: list[dict] = []
    for image_name in image_names:
        split = "val" if image_name in val_names else "train"
        source_path = IMAGE_SOURCE / image_name
        image_path = DATASET_DIR / "images" / split / image_name
        label_path = DATASET_DIR / "labels" / split / f"{Path(image_name).stem}.txt"
        _copy_image(source_path, image_path)
        _write_box(label_path, labels[image_name])
        samples.append(
            {
                "image_name": image_name,
                "image_path": image_path,
                "source_image_path": source_path,
                "box": labels[image_name],
                "split": split,
            }
        )

    classes_path = DATASET_DIR / "classes.txt"
    classes_path.write_text(CLASS_NAME + "\n", encoding="utf-8")
    dataset_yaml = DATASET_DIR / "wallet.yaml"
    dataset_yaml.write_text(
        "\n".join(
            [
                f"path: {DATASET_DIR.as_posix()}",
                "train: images/train",
                "val: images/val",
                "names:",
                f"  0: {CLASS_NAME}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    split_counts = {
        "train": sum(sample["split"] == "train" for sample in samples),
        "val": sum(sample["split"] == "val" for sample in samples),
        "total": len(samples),
    }
    box_sizes = [sample["box"] for sample in samples]
    summary = {
        "source_archive": str(ZIP_PATH),
        "source_images": str(IMAGE_SOURCE),
        "label_format": "YOLO segmentation polygons converted to detection boxes",
        "class_name": CLASS_NAME,
        "class_id": CLASS_ID,
        "image_count": len(samples),
        "split_counts": split_counts,
        "packed_image_count": len(packed_image_entries),
        "zip_images_directory_empty": len(packed_image_entries) == 0,
        "image_dimensions": {
            sample["image_name"]: list(Image.open(sample["source_image_path"]).size)
            for sample in samples
        },
        "box_size_stats_normalized": {
            "average_width": round(
                sum(box[2] - box[0] for box in box_sizes) / len(box_sizes), 6
            ),
            "average_height": round(
                sum(box[3] - box[1] for box in box_sizes) / len(box_sizes), 6
            ),
            "min_width": round(min(box[2] - box[0] for box in box_sizes), 6),
            "min_height": round(min(box[3] - box[1] for box in box_sizes), 6),
        },
        "empty_label_files": empty_images,
        "dataset_yaml": str(dataset_yaml),
        "classes_file": str(classes_path),
    }
    (DATASET_DIR / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    _make_preview(samples, DATASET_DIR / "preview.jpg")

    print(f"Dataset: {DATASET_DIR}")
    print(f"Train: {split_counts['train']}  Val: {split_counts['val']}")
    print(f"Class: {CLASS_ID} {CLASS_NAME}")
    print(f"YAML: {dataset_yaml}")
    print(f"Preview: {DATASET_DIR / 'preview.jpg'}")
    print(f"Archive images/ entries: {len(packed_image_entries)} (expected 0 here)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
