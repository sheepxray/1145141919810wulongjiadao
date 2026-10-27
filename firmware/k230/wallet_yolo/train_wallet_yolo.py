#!/usr/bin/env python3
"""Train a K230-oriented YOLOv8n wallet detector.

The epoch count is chosen from the training-set size and then capped by
Ultralytics early stopping.  The exported model is fixed at 320x320 because
that is the practical input size for the K230D KPU.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import torch
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
DATASET_YAML = ROOT / "dataset" / "wallet.yaml"
RUN_ROOT = ROOT / "runs"
CHECKPOINTS = ROOT / "checkpoints"
ARTIFACTS = ROOT / "artifacts"

MODEL_NAME = "yolov8n.pt"
TRAIN_IMAGE_SIZE = 640
EXPORT_IMAGE_SIZE = 480
BATCH_SIZE = 8
TARGET_OPTIMIZER_STEPS = 1200
MIN_EPOCHS = 100
MAX_EPOCHS = 300
EARLY_STOP_PATIENCE = 60
SEED = 42


def choose_epochs(train_count: int) -> int:
    """Select an epoch budget from the number of optimizer steps available."""
    steps_per_epoch = max(1, math.ceil(train_count / BATCH_SIZE))
    epochs = round(TARGET_OPTIMIZER_STEPS / steps_per_epoch)
    return max(MIN_EPOCHS, min(MAX_EPOCHS, epochs))


def count_images(split: str) -> int:
    split_dir = ROOT / "dataset" / "images" / split
    return sum(
        path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        for path in split_dir.iterdir()
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--train-imgsz",
        type=int,
        default=TRAIN_IMAGE_SIZE,
        help="Training resolution. 640 keeps more detail for this 4K dataset.",
    )
    parser.add_argument(
        "--export-imgsz",
        type=int,
        default=EXPORT_IMAGE_SIZE,
        help="ONNX export resolution; 480 preserves more accuracy on K230D.",
    )
    parser.add_argument(
        "--run-name",
        default="wallet_yolov8n_640_simple",
        help="Ultralytics run directory name.",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Use a short 40-epoch smoke run for pipeline checks.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not DATASET_YAML.is_file():
        raise SystemExit(
            f"Dataset YAML not found: {DATASET_YAML}. "
            "Run prepare_dataset.py first."
        )

    train_count = count_images("train")
    val_count = count_images("val")
    if train_count < 2 or val_count < 1:
        raise SystemExit(
            f"Dataset split is too small: train={train_count}, val={val_count}"
        )

    epochs = 40 if args.quick else choose_epochs(train_count)
    device = 0 if torch.cuda.is_available() else "cpu"
    CHECKPOINTS.mkdir(parents=True, exist_ok=True)
    ARTIFACTS.mkdir(parents=True, exist_ok=True)

    print(
        f"Training {MODEL_NAME} at {args.train_imgsz}x{args.train_imgsz}, "
        f"train={train_count}, val={val_count}, auto_epochs={epochs}, "
        f"device={device}"
    )

    model = YOLO(MODEL_NAME)
    results = model.train(
        data=str(DATASET_YAML),
        imgsz=args.train_imgsz,
        epochs=epochs,
        batch=BATCH_SIZE,
        patience=EARLY_STOP_PATIENCE,
        workers=4,
        device=device,
        seed=SEED,
        pretrained=True,
        optimizer="AdamW",
        lr0=0.001,
        cos_lr=True,
        amp=True,
        cache="disk",
        plots=True,
        save=True,
        val=True,
        project=str(RUN_ROOT),
        name=args.run_name,
        exist_ok=True,
        degrees=0.0,
        translate=0.05,
        scale=0.30,
        shear=0.0,
        perspective=0.0,
        fliplr=0.50,
        mosaic=0.0,
        close_mosaic=0,
        mixup=0.0,
        copy_paste=0.0,
        verbose=True,
    )

    run_dir = RUN_ROOT / args.run_name
    best_pt = run_dir / "weights" / "best.pt"
    if not best_pt.is_file():
        raise RuntimeError(f"Training finished but best.pt was not found: {best_pt}")

    artifact_pt = CHECKPOINTS / f"wallet_yolov8n_{args.export_imgsz}.pt"
    shutil.copy2(best_pt, artifact_pt)

    export_model = YOLO(str(best_pt))
    exported = export_model.export(
        format="onnx",
        imgsz=args.export_imgsz,
        opset=11,
        simplify=True,
        dynamic=False,
        half=False,
    )
    exported_path = Path(exported)
    artifact_onnx = CHECKPOINTS / f"wallet_yolov8n_{args.export_imgsz}.onnx"
    shutil.copy2(exported_path, artifact_onnx)

    metrics = {}
    try:
        val_results = export_model.val(
            data=str(DATASET_YAML),
            imgsz=args.export_imgsz,
            batch=BATCH_SIZE,
            device=device,
            plots=False,
            verbose=False,
        )
        metrics = {
            "metrics/mAP50-95(B)": float(val_results.box.map),
            "metrics/mAP50(B)": float(val_results.box.map50),
            "metrics/precision(B)": float(val_results.box.mp),
            "metrics/recall(B)": float(val_results.box.mr),
        }
    except Exception as exc:  # keep training artifacts even if validation fails
        metrics = {"validation_error": str(exc)}

    summary = {
        "model": MODEL_NAME,
        "train_image_size": args.train_imgsz,
        "export_image_size": args.export_imgsz,
        "batch_size": BATCH_SIZE,
        "train_images": train_count,
        "val_images": val_count,
        "auto_epoch_budget": epochs,
        "augmentation": {
            "mosaic": 0.0,
            "mixup": 0.0,
            "copy_paste": 0.0,
            "degrees": 0.0,
            "translate": 0.05,
            "scale": 0.30,
            "fliplr": 0.50,
        },
        "early_stop_patience": EARLY_STOP_PATIENCE,
        "max_epochs": MAX_EPOCHS,
        "target_optimizer_steps": TARGET_OPTIMIZER_STEPS,
        "checkpoint_pt": str(artifact_pt),
        "onnx": str(artifact_onnx),
        "run_dir": str(run_dir),
        "validation": metrics,
    }
    (ROOT / "training_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
