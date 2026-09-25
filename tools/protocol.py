#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K230 <-> STM32 通信协议参考实现
===============================

对应的协议定义见 docs/11-通信协议.md。

本文件是**可直接使用的实现**，不是伪代码：
  - K230 侧（Python）：直接 `import protocol` 使用
  - STM32 侧（C）   ：按同样的帧格式与 CRC 参数移植，见 docs/11 §7

帧格式：
    ┌──────┬──────┬──────┬─────────────┬────────┬──────┐
    │ 0xAA │ 0x55 │ LEN  │   PAYLOAD   │ CRC16  │ 0x0D │
    └──────┴──────┴──────┴─────────────┴────────┴──────┘
    CRC 覆盖 LEN + PAYLOAD，小端存放

运行自测：
    python tools/protocol.py
"""

import struct
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


HEAD = b"\xAA\x55"
TAIL = 0x0D
# LEN 字段是 u8（0-255），但协议限制在 128 以简化 MCU 侧的缓冲区分配。
#
# 早期版本设为 64，但自测发现 TARGET_LIST 满 8 项时负载为
#   6 (头部) + 8 x 8 (条目) = 70 字节 > 64
# 即协议自身的上限与规范矛盾。详见 docs/11。
MAX_PAYLOAD = 128

# ---- 消息 ID ----
MSG_TARGET_LIST      = 0x01    # K230 -> STM32
MSG_CLASSIFY_RESULT  = 0x02
MSG_SERVO_ERROR      = 0x03
MSG_HEARTBEAT_ACK    = 0x04
MSG_HELLO            = 0x05
MSG_WALL_RANGE       = 0x06

MSG_HEARTBEAT        = 0x81    # STM32 -> K230
MSG_MODE_CMD         = 0x82
MSG_STATE_INFO       = 0x83
MSG_PARAM_SET        = 0x84

MSG_NAMES = {
    MSG_TARGET_LIST: "TARGET_LIST",
    MSG_CLASSIFY_RESULT: "CLASSIFY_RESULT",
    MSG_SERVO_ERROR: "SERVO_ERROR",
    MSG_HEARTBEAT_ACK: "HEARTBEAT_ACK",
    MSG_HELLO: "HELLO",
    MSG_WALL_RANGE: "WALL_RANGE",
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_MODE_CMD: "MODE_CMD",
    MSG_STATE_INFO: "STATE_INFO",
    MSG_PARAM_SET: "PARAM_SET",
}

# ---- K230 工作模式 ----
MODE_IDLE = 0
MODE_SEARCH = 1
MODE_CLASSIFY = 2
MODE_SERVO = 3

# ---- 兵种编号 ----
PIECE_UNKNOWN = 0
PIECE_PAWN = 1       # 兵/卒
PIECE_CANNON = 2     # 炮
PIECE_HORSE = 3      # 马
PIECE_CHARIOT = 4    # 车
PIECE_ELEPHANT = 5   # 象/相
PIECE_ADVISOR = 6    # 士/仕
PIECE_KING = 7       # 帅/将

PIECE_NAMES = {
    PIECE_UNKNOWN: "未识别", PIECE_PAWN: "兵/卒", PIECE_CANNON: "炮",
    PIECE_HORSE: "马", PIECE_CHARIOT: "车", PIECE_ELEPHANT: "象/相",
    PIECE_ADVISOR: "士/仕", PIECE_KING: "帅/将",
}

COLOR_UNKNOWN, COLOR_RED, COLOR_BLACK = 0, 1, 2
COLOR_NAMES = {COLOR_UNKNOWN: "未知", COLOR_RED: "红", COLOR_BLACK: "黑"}


# ==========================================================================
# CRC16 / CCITT-FALSE : poly=0x1021, init=0xFFFF, 无反射, xorout=0
# ==========================================================================

def _build_crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes) -> int:
    """表驱动 CRC16/CCITT-FALSE。MCU 侧可直接照搬这张表。"""
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# ==========================================================================
# 编码
# ==========================================================================

def encode(payload: bytes) -> bytes:
    """把 PAYLOAD 打包成完整帧。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    ln = bytes([len(payload)])
    crc = crc16(ln + payload)
    return HEAD + ln + payload + struct.pack("<H", crc) + bytes([TAIL])


# ---- 各消息的构造器 ----

def make_target_list(timestamp_ms: int, targets) -> bytes:
    """
    targets: 可迭代的 (x_mm, y_mm, color, conf) 元组，最多 8 项。
    """
    targets = list(targets)[:8]
    buf = struct.pack("<BBI", MSG_TARGET_LIST, len(targets), timestamp_ms)
    for x_mm, y_mm, color, conf in targets:
        buf += struct.pack("<hhBBH", int(x_mm), int(y_mm), int(color),
                           int(conf), 0)
    return encode(buf)


def make_classify_result(piece_class: int, color: int, score_value: int,
                         is_king: int, conf: int, x_mm: int, y_mm: int) -> bytes:
    buf = struct.pack("<BBBBBBhh", MSG_CLASSIFY_RESULT, piece_class, color,
                      score_value, is_king, conf, int(x_mm), int(y_mm))
    return encode(buf)


def make_servo_error(err_mm: int, valid: int) -> bytes:
    buf = struct.pack("<BhB", MSG_SERVO_ERROR, int(err_mm), int(valid))
    return encode(buf)


def make_heartbeat_ack(seq: int) -> bytes:
    return encode(struct.pack("<BH", MSG_HEARTBEAT_ACK, seq))


def make_hello(fw_major: int, fw_minor: int, boot_count: int) -> bytes:
    return encode(struct.pack("<BBBI", MSG_HELLO, fw_major, fw_minor, boot_count))


def make_wall_range(front_mm, left_mm, right_mm, valid_mask) -> bytes:
    buf = struct.pack("<BhhhB", MSG_WALL_RANGE, int(front_mm), int(left_mm),
                      int(right_mm), int(valid_mask))
    return encode(buf)


def make_heartbeat(seq: int) -> bytes:
    return encode(struct.pack("<BH", MSG_HEARTBEAT, seq))


def make_mode_cmd(mode: int) -> bytes:
    return encode(struct.pack("<BB", MSG_MODE_CMD, int(mode)))


def make_state_info(state: int, v_mm_s: int, omega_mrad_s: int,
                    heading_mrad: int, flags: int) -> bytes:
    buf = struct.pack("<BBhhhB", MSG_STATE_INFO, state, int(v_mm_s),
                      int(omega_mrad_s), int(heading_mrad), int(flags))
    return encode(buf)


# ==========================================================================
# 流式解码器
# ==========================================================================

class Parser:
    """
    流式解码器（对应 docs/11 §6 的状态机）。

    用法：
        p = Parser()
        for byte in incoming:
            msg = p.feed(byte)
            if msg is not None:
                handle(msg)

    msg 是 (msg_id, payload_bytes, raw_frame_bytes)。

    实现说明
    --------
    早期版本用手写状态机逐字节推进。它有一个**真实的重同步缺陷**（由本文件的自测捕获）：

        噪声中出现假的 `AA 55` 后，紧接着的真实帧的帧头会被当成 LEN 字段读走
        （0xAA = 170 > MAX_PAYLOAD），于是解析器把整个缓冲区丢弃 ——
        连后面的真实帧也一起丢了，导致"一帧噪声后持续丢包"。

    现在改为**缓冲区 + 重扫描**：任何一步校验失败时，只丢弃当前候选帧头的
    **第一个字节**，然后从剩余字节里重新搜索 `AA 55`。这样假帧头不会吃掉真帧头。
    """

    def __init__(self):
        self.buf = bytearray()
        self.stats = {"ok": 0, "crc_err": 0, "tail_err": 0, "len_err": 0}

    def reset(self):
        self.buf = bytearray()

    def feed(self, byte: int):
        """喂入一个字节，返回解析出的消息或 None。"""
        self.buf.append(byte & 0xFF)
        return self._try_parse()

    def feed_bytes(self, data: bytes):
        """喂入一串字节，返回解析出的消息列表。"""
        out = []
        for b in data:
            m = self.feed(b)
            if m is not None:
                out.append(m)
        return out

    def _try_parse(self):
        while True:
            # 1) 定位帧头
            idx = self.buf.find(HEAD)
            if idx < 0:
                # 没有完整帧头。保留最后 1 字节（可能是半个帧头 0xAA）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return None
            if idx > 0:
                del self.buf[:idx]

            # 2) 需要 3 字节才能读到 LEN
            if len(self.buf) < 3:
                return None

            length = self.buf[2]
            if length > MAX_PAYLOAD:
                # LEN 非法 -> 这是个假帧头。只丢第一个字节，继续找真帧头。
                self.stats["len_err"] += 1
                del self.buf[0]
                continue

            frame_len = len(HEAD) + 1 + length + 2 + 1   # 帧头+LEN+负载+CRC+帧尾
            if len(self.buf) < frame_len:
                return None                               # 等待更多字节

            payload = bytes(self.buf[3:3 + length])
            crc_rx = self.buf[3 + length] | (self.buf[4 + length] << 8)
            crc_calc = crc16(bytes([length]) + payload)
            tail = self.buf[5 + length]

            if crc_rx != crc_calc:
                self.stats["crc_err"] += 1
                del self.buf[0]                           # 同上：只丢一个字节
                continue
            if tail != TAIL:
                self.stats["tail_err"] += 1
                del self.buf[0]
                continue

            # 3) 成功
            raw = bytes(self.buf[:frame_len])
            del self.buf[:frame_len]
            self.stats["ok"] += 1
            msg_id = payload[0] if payload else -1
            return (msg_id, payload, raw)


# ==========================================================================
# 解码辅助
# ==========================================================================

def parse_target_list(payload: bytes):
    msg_id, count, ts = struct.unpack_from("<BBI", payload, 0)
    targets = []
    off = 6
    for _ in range(count):
        x, y, color, conf, _res = struct.unpack_from("<hhBBH", payload, off)
        targets.append({"x_mm": x, "y_mm": y, "color": color, "conf": conf})
        off += 8
    return {"timestamp_ms": ts, "targets": targets}


def parse_classify_result(payload: bytes):
    (msg_id, piece_class, color, score_value, is_king, conf,
     x, y) = struct.unpack_from("<BBBBBBhh", payload, 0)
    return {"piece_class": piece_class, "color": color,
            "score_value": score_value, "is_king": bool(is_king),
            "conf": conf, "x_mm": x, "y_mm": y}


def parse_servo_error(payload: bytes):
    msg_id, err, valid = struct.unpack_from("<BhB", payload, 0)
    return {"err_mm": err, "valid": bool(valid)}


def describe(msg_id: int, payload: bytes) -> str:
    """把消息渲染成可读字符串（用于日志）。"""
    name = MSG_NAMES.get(msg_id, f"UNKNOWN(0x{msg_id:02X})")
    try:
        if msg_id == MSG_TARGET_LIST:
            d = parse_target_list(payload)
            items = ", ".join(
                f"({t['x_mm']},{t['y_mm']}){COLOR_NAMES.get(t['color'],'?')}"
                f"@{t['conf']}" for t in d["targets"])
            return f"{name}: ts={d['timestamp_ms']} [{items}]"
        if msg_id == MSG_CLASSIFY_RESULT:
            d = parse_classify_result(payload)
            return (f"{name}: {COLOR_NAMES.get(d['color'],'?')}"
                    f"{PIECE_NAMES.get(d['piece_class'],'?')}"
                    f" 分值{d['score_value']}"
                    f"{' 帅/将' if d['is_king'] else ''} conf={d['conf']}")
        if msg_id == MSG_SERVO_ERROR:
            d = parse_servo_error(payload)
            return f"{name}: err={d['err_mm']}mm valid={d['valid']}"
        if msg_id in (MSG_HEARTBEAT, MSG_HEARTBEAT_ACK):
            seq = struct.unpack_from("<H", payload, 1)[0]
            return f"{name}: seq={seq}"
        if msg_id == MSG_HELLO:
            ma, mi, bc = struct.unpack_from("<BBI", payload, 1)
            return f"{name}: fw={ma}.{mi} boot_count={bc}"
        if msg_id == MSG_WALL_RANGE:
            f, l, r, m = struct.unpack_from("<hhhB", payload, 1)
            return f"{name}: front={f} left={l} right={r} mask=0b{m:03b}"
        if msg_id == MSG_MODE_CMD:
            mode = struct.unpack_from("<B", payload, 1)[0]
            return f"{name}: mode={mode}"
        if msg_id == MSG_STATE_INFO:
            st, v, w, hd, fl = struct.unpack_from("<BhhhB", payload, 1)
            return f"{name}: state={st} v={v}mm/s w={w}mrad/s hdg={hd}mrad"
    except struct.error as e:
        return f"{name}: <解析失败: {e}>"
    return f"{name}: {payload.hex()}"


# ==========================================================================
# 自测
# ==========================================================================

def selftest():
    print("=" * 78)
    print("协议实现自测")
    print("=" * 78)

    # ---- CRC 已知向量 ----
    print("\n[1] CRC16/CCITT-FALSE 已知向量")
    # "123456789" 的 CRC-16/CCITT-FALSE 标准值是 0x29B1
    v = crc16(b"123456789")
    print(f'  crc16("123456789") = 0x{v:04X}  (期望 0x29B1)  '
          f'{"通过" if v == 0x29B1 else "失败"}')
    assert v == 0x29B1, "CRC 实现错误"

    # ---- 逐消息 round-trip ----
    print("\n[2] 编码 -> 解码 round-trip")
    cases = [
        ("TARGET_LIST", make_target_list(12345, [
            (850, -120, COLOR_RED, 90),
            (1200, 300, COLOR_BLACK, 75),
        ])),
        ("CLASSIFY_RESULT", make_classify_result(
            PIECE_KING, COLOR_RED, 10, 1, 96, 320, -15)),
        ("SERVO_ERROR", make_servo_error(-23, 1)),
        ("HEARTBEAT_ACK", make_heartbeat_ack(42)),
        ("HELLO", make_hello(1, 2, 7)),
        ("WALL_RANGE", make_wall_range(412, -1, 680, 0b101)),
        ("HEARTBEAT", make_heartbeat(99)),
        ("MODE_CMD", make_mode_cmd(MODE_SERVO)),
        ("STATE_INFO", make_state_info(2, 800, 120, -350, 0)),
    ]
    for label, frame in cases:
        p = Parser()
        msgs = p.feed_bytes(frame)
        ok = len(msgs) == 1 and msgs[0][2] == frame
        print(f"  {label:<18} {'通过' if ok else '失败'}   {describe(*msgs[0][:2]) if msgs else ''}")
        assert ok, f"{label} round-trip 失败"

    # ---- 粘包 / 半包 / 噪声 ----
    print("\n[3] 流式解析健壮性")
    f1 = make_heartbeat(1)
    f2 = make_classify_result(PIECE_CHARIOT, COLOR_BLACK, 9, 0, 88, 400, 10)
    f3 = make_servo_error(5, 1)

    p = Parser()
    msgs = p.feed_bytes(f1 + f2 + f3)     # 粘包
    print(f"  粘包(3帧一次喂入)      解析 {len(msgs)} 帧  "
          f"{'通过' if len(msgs) == 3 else '失败'}")
    assert len(msgs) == 3

    p = Parser()
    out = []
    for i in range(0, len(f1), 3):        # 半包，每次喂 3 字节
        out += p.feed_bytes(f1[i:i + 3])
    print(f"  半包(每次3字节)        解析 {len(out)} 帧  "
          f"{'通过' if len(out) == 1 else '失败'}")
    assert len(out) == 1

    p = Parser()
    noise = bytes([0x00, 0xFF, 0xAA, 0x11, 0x55, 0xAA, 0xAA, 0x55])
    out = p.feed_bytes(noise + f2)        # 噪声 + 帧头欺骗
    print(f"  噪声+帧头欺骗          解析 {len(out)} 帧  "
          f"{'通过' if len(out) == 1 else '失败'}")
    assert len(out) == 1

    # ---- CRC 错误检测 ----
    print("\n[4] 错误检测")
    p = Parser()
    bad = bytearray(f2)
    bad[6] ^= 0xFF                        # 翻转 payload 中一字节
    out = p.feed_bytes(bytes(bad))
    print(f"  payload 位翻转         crc_err={p.stats['crc_err']}  "
          f"{'通过(已拒绝)' if len(out) == 0 and p.stats['crc_err'] >= 1 else '失败'}")
    assert len(out) == 0 and p.stats["crc_err"] >= 1

    p = Parser()
    bad_tail = f1[:-1] + bytes([0x00])    # 帧尾错误
    out = p.feed_bytes(bad_tail)
    print(f"  帧尾错误               tail_err={p.stats['tail_err']}  "
          f"{'通过(已拒绝)' if len(out) == 0 else '失败'}")
    assert len(out) == 0

    # ---- 超长 payload 保护 ----
    print("\n[5] 边界保护")
    try:
        encode(b"\x00" * (MAX_PAYLOAD + 1))
        print("  超长 payload         失败（未抛出异常）")
        raise SystemExit(1)
    except ValueError:
        print(f"  超长 payload          通过(已拒绝)")

    p = Parser()
    out = p.feed_bytes(b"\xAA\x55\xFF" + b"\x00" * 300)   # LEN=255 超限
    print(f"  LEN 超限               len_err={p.stats['len_err']}  "
          f"{'通过(已拒绝)' if len(out) == 0 else '失败'}")
    assert len(out) == 0

    # ---- 实际带宽核算 ----
    print("\n[6] 带宽核算（验证 921600 的选择）")
    tl = make_target_list(999, [(800, -100, COLOR_RED, 90)] * 8)
    print(f"  TARGET_LIST(8项) = {len(tl)} 字节")
    print(f"  SERVO_ERROR      = {len(make_servo_error(5,1))} 字节")
    print(f"  10Hz 下 TARGET_LIST 占用 = {len(tl)*10*10} bps "   # 10bit/byte
          f"（921600 bps 的 {len(tl)*10*10/921600*100:.2f}%）")
    print(f"  100Hz 下 SERVO_ERROR 占用 = {len(make_servo_error(5,1))*100*10} bps "
          f"（921600 bps 的 {len(make_servo_error(5,1))*100*10/921600*100:.2f}%）")

    print("\n" + "=" * 78)
    print("全部自测通过。")
    print("=" * 78)


if __name__ == "__main__":
    selftest()
