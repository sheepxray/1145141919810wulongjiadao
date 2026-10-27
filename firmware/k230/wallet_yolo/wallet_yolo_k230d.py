#!/usr/bin/env python3
"""CanMV K230D debug script for the wallet YOLOv8n kmodel.

Copy this file and wallet_yolov8n_320.kmodel to /sdcard, then run this file
in CanMV IDE.  The script uses the official nncase_runtime KPU/AI2D APIs and
displays the frame in the IDE virtual display.
"""

import gc
import os
import time

import ulab.numpy as np
import nncase_runtime as nn
from media.display import *
from media.media import *
from media.sensor import *


KMODEL_PATH = "/sdcard/wallet_yolov8n_480.kmodel"
FRAME_W = 640
FRAME_H = 480
MODEL_W = 480
MODEL_H = 480
CONF_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
MAX_CANDIDATES = 100
LABEL = "wallet"


def iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    intersection = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def nms(boxes, scores, threshold):
    order = sorted(range(len(scores)), key=lambda index: scores[index], reverse=True)
    keep = []
    while order:
        current = order.pop(0)
        keep.append(current)
        remaining = []
        for candidate in order:
            if iou(boxes[current], boxes[candidate]) <= threshold:
                remaining.append(candidate)
        order = remaining
    return keep


def image_to_model_input(image):
    """Convert a camera RGB565 image to a 1x3xHxW RGB888 array."""
    rgb888 = image.to_rgb888()
    hwc = rgb888.to_numpy_ref()
    shape = hwc.shape
    pixels = hwc.reshape((shape[0] * shape[1], shape[2]))
    chw = pixels.transpose().copy().reshape(
        (1, shape[2], shape[0], shape[1])
    )
    return chw


def decode_output(output, frame_w, frame_h):
    """Decode YOLOv8 [1, 5, N] or [1, N, 5] output."""
    shape = output.shape
    if len(shape) == 3:
        if shape[1] == 5:
            boxes_first = False
            count = shape[2]
            result = output[0]
            boxes_ori = result[0:4, :]
            scores_ori = result[4]
        elif shape[2] == 5:
            boxes_first = True
            count = shape[1]
            result = output[0]
            boxes_ori = result[:, 0:4]
            scores_ori = result[:, 4]
        else:
            raise ValueError("Unexpected YOLO output shape: %s" % (shape,))
    else:
        raise ValueError("Unexpected YOLO output rank: %s" % (shape,))

    candidates = []
    for index in range(count):
        score = float(scores_ori[index])
        if score < CONF_THRESHOLD:
            continue

        if boxes_first:
            x = float(boxes_ori[index, 0])
            y = float(boxes_ori[index, 1])
            w = float(boxes_ori[index, 2])
            h = float(boxes_ori[index, 3])
        else:
            x = float(boxes_ori[0, index])
            y = float(boxes_ori[1, index])
            w = float(boxes_ori[2, index])
            h = float(boxes_ori[3, index])
        x1 = max(0.0, (x - w * 0.5) * frame_w / MODEL_W)
        y1 = max(0.0, (y - h * 0.5) * frame_h / MODEL_H)
        x2 = min(float(frame_w), (x + w * 0.5) * frame_w / MODEL_W)
        y2 = min(float(frame_h), (y + h * 0.5) * frame_h / MODEL_H)
        if x2 <= x1 or y2 <= y1:
            continue
        candidates.append(([x1, y1, x2, y2], score))

    if not candidates:
        return []

    candidates.sort(key=lambda item: item[1], reverse=True)
    candidates = candidates[:MAX_CANDIDATES]
    boxes = [item[0] for item in candidates]
    scores = [item[1] for item in candidates]
    keep = nms(boxes, scores, NMS_THRESHOLD)
    return [(boxes[index], scores[index]) for index in keep]


def draw_detections(image, detections):
    for box, score in detections:
        x1, y1, x2, y2 = (int(round(value)) for value in box)
        width = max(1, x2 - x1)
        height = max(1, y2 - y1)
        image.draw_rectangle(x1, y1, width, height, color=(255, 0, 0), thickness=2)
        image.draw_cross(
            x1 + width // 2,
            y1 + height // 2,
            color=(255, 255, 0),
            size=6,
        )
        image.draw_string_advanced(
            x1,
            max(0, y1 - 18),
            16,
            "%s %.2f" % (LABEL, score),
            color=(255, 0, 0),
        )


def main():
    print("WALLET_BOOT")
    sensor = None
    display_ready = False
    kpu = None
    ai2d = None
    ai2d_builder = None
    ai2d_output_tensor = None
    try:
        sensor = Sensor()
        print("WALLET_SENSOR_NEW")
        sensor.reset()
        print("WALLET_SENSOR_RESET")
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)
        print("WALLET_SENSOR_CONFIG")

        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        print("WALLET_DISPLAY_READY")
        display_ready = True
        MediaManager.init()
        sensor.run()
        print("WALLET_SENSOR_RUN")

        for _ in range(10):
            os.exitpoint()
            sensor.snapshot()

        kpu = nn.kpu()
        kpu.load_kmodel(KMODEL_PATH)
        print("WALLET_KMODEL_LOADED")
        ai2d = nn.ai2d()
        ai2d.set_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )
        ai2d.set_resize_param(
            True,
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel,
        )
        ai2d_builder = ai2d.build(
            [1, 3, FRAME_H, FRAME_W],
            [1, 3, MODEL_H, MODEL_W],
        )
        ai2d_output_data = np.ones((1, 3, MODEL_H, MODEL_W), dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(ai2d_output_data)
        print("WALLET_AI2D_READY")

        clock = time.clock()
        frame_index = 0
        print("WALLET_LOOP_START")
        while True:
            os.exitpoint()
            clock.tick()
            image = sensor.snapshot()
            ai2d_input_tensor = nn.from_numpy(image_to_model_input(image))
            ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

            kpu.set_input_tensor(0, ai2d_output_tensor)
            kpu.run()
            output_tensor = kpu.get_output_tensor(0)
            detections = decode_output(output_tensor.to_numpy(), FRAME_W, FRAME_H)
            del output_tensor

            draw_detections(image, detections)
            if detections:
                best_box, best_score = detections[0]
                x1, y1, x2, y2 = best_box
                cx = (x1 + x2) * 0.5
                cy = (y1 + y2) * 0.5
                nx = (cx - FRAME_W * 0.5) / (FRAME_W * 0.5)
                ny = (cy - FRAME_H * 0.5) / (FRAME_H * 0.5)
                print(
                    "DET wallet cx=%d cy=%d w=%d h=%d nx=%.3f ny=%.3f "
                    "score=%.3f n=%d fps=%.1f"
                    % (
                        int(cx),
                        int(cy),
                        int(x2 - x1),
                        int(y2 - y1),
                        nx,
                        ny,
                        best_score,
                        len(detections),
                        clock.fps(),
                    )
                )
            else:
                print("NO_DET fps=%.1f" % clock.fps())

            Display.show_image(image)
            frame_index += 1
            if frame_index % 30 == 0:
                gc.collect()

    except KeyboardInterrupt as exc:
        print("stopped:", exc)
    except BaseException as exc:
        print("wallet_yolo error:", exc)
        try:
            import traceback

            traceback.print_exception(exc)
        except Exception:
            pass
    finally:
        try:
            if ai2d_builder is not None:
                del ai2d_builder
            if ai2d_output_tensor is not None:
                del ai2d_output_tensor
            if ai2d is not None:
                del ai2d
            if kpu is not None:
                del kpu
            gc.collect()
            nn.shrink_memory_pool()
        except Exception:
            pass
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        if display_ready:
            Display.deinit()
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
