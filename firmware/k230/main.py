# CanMV K230 摄像头预览
# 点击「运行」按钮执行脚本

from media.sensor import *
from media.display import *
from media.media import *
import time, os

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB565)

# to_ide=True 将画面发送到 IDE 帧缓冲面板
Display.init(Display.VIRT, width=640, height=480, to_ide=True)
MediaManager.init()
sensor.run()

try:
    while True:
        os.exitpoint()
        img = sensor.snapshot()
        Display.show_image(img)
except KeyboardInterrupt as e:
    print("用户停止:", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
