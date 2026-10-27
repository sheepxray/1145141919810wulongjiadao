# 钱包 YOLOv8n K230D 训练与部署

这个目录把 `project-1-at-2026-10-23-23-42-9e46ba4d.zip` 的 Label Studio
多边形标注转换成了单类 `钱包` 的 YOLO 检测数据集，并以 640x640 训练、
导出 480x480 的 YOLOv8n 模型给 K230D KPU 使用。

## 输入数据

- 标注：`C:\Users\sheep\Downloads\project-1-at-2026-10-23-23-42-9e46ba4d.zip`
- 原图：`C:\Users\sheep\Desktop\data1123`
- 标签格式：每个文件是一行 YOLO 分割多边形
- 训练格式：多边形外接矩形，转换成 YOLO 检测框

压缩包里的 `images/` 是空的，所以训练脚本使用同名的 `data1123` 原图。

## 目录

```text
wallet_yolo_k230/
  prepare_dataset.py               生成训练集
  train_wallet_yolo.py             自动轮数训练 + 导出 ONNX
  convert_to_kmodel.py             nncase 2.11.0 转换 + kmodel 文件校验
  predict_wallet_pc.py             PC 侧验证最终 ONNX 并输出标注图
  wallet_yolo_k230d.py             K230D CanMV IDE 调试脚本
  dataset/                         转换后的数据集
  checkpoints/                     .pt / .onnx / .kmodel
  runs/                            Ultralytics 训练日志和曲线
```

## 在本机运行

```powershell
cd C:\Users\sheep\Desktop\乌龙驾到\outputs\wallet_yolo_k230
python prepare_dataset.py
python train_wallet_yolo.py
python convert_to_kmodel.py
python predict_wallet_pc.py
```

训练轮数不是固定值：

```text
steps_per_epoch = ceil(train_images / batch_size)
epochs = clamp(round(1200 / steps_per_epoch), 100, 300)
```

同时使用 `patience=60` 做早停。这样即使初始化轮数较大，也会在验证集
不再改善时自动停止。

## K230D 部署

1. 把 `checkpoints/wallet_yolov8n_480.kmodel` 和
   `wallet_yolo_k230d.py` 拷到 K230D 的 `/sdcard`。
2. 在 CanMV IDE 打开 `wallet_yolo_k230d.py`，点击运行。
3. IDE 虚拟屏幕应显示摄像头画面和红色钱包检测框。
4. 串口应打印 `DET wallet ...` 或 `NO_DET ...`。

脚本默认使用：

```text
640x480 RGB565 camera frame
480x480 kmodel input
confidence threshold = 0.25
NMS threshold = 0.45
```

训练默认关闭 `mosaic`、`mixup` 和 `copy_paste`，因为当前只有 16 张图，
复合增强容易让单类目标学习不稳定。需要做快速冒烟测试时可以运行：

```powershell
python train_wallet_yolo.py --quick
```

## 注意

- 当前只有 16 张训练图片，模型只能用于流程验证和初步调试，不能当作
  比赛最终模型。建议补拍不同距离、角度、光照和背景，至少 100 到 300 张
  真实场景图片后再重训。
- 本次 480x480 ONNX 在 3 张验证图上的结果为 `Precision=0.734`、
  `Recall=0.935`、`mAP50=0.913`、`mAP50-95=0.64`。验证集很小，只用于
  说明流程已经跑通，不代表泛化性能。
- 320 输入虽然更省算力，但当前数据下置信度和定位损失明显，最终选择
  480 输入；如果 K230D 实测帧率不够，再回退到 320 并补数据重训。
- Windows 上 nncase 需要同时安装 `nncase==2.11.0` 和对应版本的
  `nncase_kpu-2.11.0-py2.py3-none-win_amd64.whl`，否则不能以 `k230`
  为编译目标；nncase 2.11.0 还需要 .NET 运行时可被主机加载。当前机器
  已通过 `DOTNET_ROLL_FORWARD=LatestMajor` 使用现有 .NET 8/9/10。
- Windows 侧 nncase 的 K230 模拟器插件在当前机器上报缺少运行模块；
  kmodel 已经正常生成并通过 `LDMK` 文件头校验，最终推理仍需在 K230D
  上实机验证。
- 当前 kmodel 输入是 `uint8 NCHW 480x480`，nncase 内部按 `[0,1]`
  反量化。K230 脚本用 AI2D 做双线性缩放后再送入 KPU。
