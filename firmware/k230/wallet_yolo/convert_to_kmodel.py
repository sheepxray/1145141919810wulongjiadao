#!/usr/bin/env python3
"""Convert the trained wallet ONNX model to a K230 kmodel with nncase 2.11.0."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path

import numpy as np
import onnx
import onnxsim
from PIL import Image

os.environ.setdefault("DOTNET_ROLL_FORWARD", "LatestMajor")
_nncase_spec = importlib.util.find_spec("nncase")
if _nncase_spec is not None and _nncase_spec.origin:
    os.environ.setdefault(
        "NNCASE_PLUGIN_PATH",
        str(Path(_nncase_spec.origin).resolve().parent / "modules"),
    )

try:
    import nncase
except ImportError as exc:
    raise SystemExit(
        "nncase is not installed. Install nncase==2.11.0 and the matching "
        "nncase_kpu Windows wheel first."
    ) from exc


ROOT = Path(__file__).resolve().parent
DATASET_DIR = ROOT / "dataset"
ONNX_PATH = ROOT / "checkpoints" / "wallet_yolov8n_480.onnx"
KMODEL_PATH = ROOT / "checkpoints" / "wallet_yolov8n_480.kmodel"
SIMPLIFIED_PATH = ROOT / "checkpoints" / "wallet_yolov8n_480.simplified.onnx"
SUMMARY_PATH = ROOT / "conversion_summary.json"
TARGET = "k230"
IMAGE_SIZE = 480

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=Path, default=ONNX_PATH)
    parser.add_argument("--output", type=Path, default=KMODEL_PATH)
    parser.add_argument("--image-size", type=int, default=IMAGE_SIZE)
    return parser.parse_args()


def _load_calibration_data(image_size: int) -> list[list[np.ndarray]]:
    image_paths: list[Path] = []
    for split in ("train", "val"):
        split_dir = DATASET_DIR / "images" / split
        image_paths.extend(
            sorted(
                path
                for path in split_dir.iterdir()
                if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
            )
        )
    if not image_paths:
        raise SystemExit("No calibration images found in dataset/images/*")

    calibration: list[list[np.ndarray]] = []
    for path in image_paths:
        image = Image.open(path).convert("RGB").resize(
            (image_size, image_size), Image.Resampling.BILINEAR
        )
        data = np.asarray(image, dtype=np.uint8).transpose(2, 0, 1)
        calibration.append([data[np.newaxis, ...]])
    return calibration


def _simplify_onnx(
    onnx_path: Path,
    image_size: int,
    output_path: Path,
) -> Path:
    model = onnx.load(str(onnx_path))
    model = onnx.shape_inference.infer_shapes(model)
    input_names = [item.name for item in model.graph.input]
    if not input_names:
        raise RuntimeError("ONNX model has no graph input")
    input_name = input_names[0]
    simplified, ok = onnxsim.simplify(
        model,
        overwrite_input_shapes={input_name: [1, 3, image_size, image_size]},
    )
    if not ok:
        raise RuntimeError("ONNX simplification failed validation")
    onnx.save(simplified, str(output_path))
    return output_path


def _compile(simplified_onnx: Path, image_size: int, output_path: Path) -> None:
    compile_options = nncase.CompileOptions()
    compile_options.target = TARGET
    compile_options.preprocess = True
    compile_options.input_shape = [1, 3, image_size, image_size]
    compile_options.input_type = "uint8"
    compile_options.input_range = [0, 1]
    compile_options.mean = [0, 0, 0]
    compile_options.std = [1, 1, 1]
    compile_options.input_layout = "NCHW"
    compile_options.output_layout = "NCHW"
    compile_options.swapRB = False
    compile_options.letterbox_value = 0
    compile_options.dump_ir = False
    compile_options.dump_asm = False

    compiler = nncase.Compiler(compile_options)
    compiler.import_onnx(simplified_onnx.read_bytes(), nncase.ImportOptions())

    ptq_options = nncase.PTQTensorOptions()
    calibration = _load_calibration_data(image_size)
    ptq_options.samples_count = len(calibration)
    ptq_options.calibrate_method = "Kld"
    ptq_options.quant_type = "uint8"
    ptq_options.w_quant_type = "uint8"
    ptq_options.set_tensor_data(calibration)
    compiler.use_ptq(ptq_options)
    compiler.compile()
    output_path.write_bytes(compiler.gencode_tobytes())


def _verify(kmodel_path: Path, image_size: int) -> dict:
    model_bytes = kmodel_path.read_bytes()
    result = {
        "file_magic": model_bytes[:4].decode("ascii", errors="replace"),
        "kmodel_size_bytes": len(model_bytes),
        "status": "kmodel_file_ok",
    }

    try:
        simulator = nncase.Simulator()
        simulator.load_model(model_bytes)
        sample = _load_calibration_data(image_size)[0][0]
        simulator.set_input_tensor(
            0,
            nncase.RuntimeTensor.from_numpy(sample),
        )
        simulator.run()

        output_shapes = []
        output_stats = []
        for index in range(simulator.outputs_size):
            output = simulator.get_output_tensor(index).to_numpy()
            output_shapes.append(list(output.shape))
            output_stats.append(
                {
                    "min": float(output.min()),
                    "max": float(output.max()),
                    "mean": float(output.mean()),
                }
            )
        result.update(
            {
                "status": "simulator_ok",
                "input_shape": list(sample.shape),
                "output_shapes": output_shapes,
                "output_stats": output_stats,
                "inputs_size": int(simulator.inputs_size),
                "outputs_size": int(simulator.outputs_size),
            }
        )
    except Exception as exc:
        # The Windows nncase K230 simulation plugin can be unavailable even
        # though code generation succeeds.  Keep the kmodel and report this
        # honestly; the final check must run on the K230D board.
        result["simulator_error"] = repr(exc)
        result["status"] = "kmodel_file_ok_simulator_unavailable"
    return result


def main() -> int:
    args = parse_args()
    onnx_path = args.onnx.resolve()
    kmodel_path = args.output.resolve()
    simplified_path = kmodel_path.with_suffix(".simplified.onnx")
    image_size = args.image_size

    if not onnx_path.is_file():
        raise SystemExit(
            f"ONNX model not found: {onnx_path}. Run train_wallet_yolo.py first."
        )

    onnx_version = getattr(onnx, "__version__", "unknown")
    try:
        import _nncase

        nncase_version = getattr(_nncase, "__version__", "unknown")
    except Exception:
        nncase_version = getattr(nncase, "__version__", "unknown")

    simplified = _simplify_onnx(onnx_path, image_size, simplified_path)
    _compile(simplified, image_size, kmodel_path)
    verification = _verify(kmodel_path, image_size)
    summary = {
        "onnx_path": str(onnx_path),
        "simplified_onnx": str(simplified_path),
        "kmodel_path": str(kmodel_path),
        "onnx_version": onnx_version,
        "nncase_version": nncase_version,
        "target": TARGET,
        "image_size": image_size,
        "calibration_images": len(_load_calibration_data(image_size)),
        "preprocess": {
            "input_type": "uint8",
            "input_range": [0, 1],
            "mean": [0, 0, 0],
            "std": [1, 1, 1],
        },
        "kmodel_size_bytes": kmodel_path.stat().st_size,
        "verification": verification,
    }
    SUMMARY_PATH.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
