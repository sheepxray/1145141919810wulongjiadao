#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
su_client.py — 从外部向 SketchUp 里的 su_bridge 发 Ruby 代码并取回结果。

用法:
    python su_client.py "Sketchup.active_model.title"
    python su_client.py -f script.rb
    type script.rb | python su_client.py

选项:
    --host / --port   默认 127.0.0.1:47123
    --timeout         秒，默认 30（SketchUp 首次启动较慢）
    --raw             只打印 JSON
退出码:
    0 成功; 1 桥报错(ok=false); 2 连接/超时失败
"""

import argparse
import base64
import json
import socket
import sys


def call(code, host="127.0.0.1", port=47123, timeout=30.0):
    payload = base64.b64encode(code.encode("utf-8")) + b"\n"
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(payload)
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
    if not buf.strip():
        raise RuntimeError("empty response (bridge not answering)")
    return json.loads(base64.b64decode(buf.strip()).decode("utf-8"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("code", nargs="?", help="Ruby 代码串")
    ap.add_argument("-f", "--file", help="从文件读取 Ruby 代码")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=47123)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--raw", action="store_true")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    if args.file:
        with open(args.file, "r", encoding="utf-8") as fh:
            code = fh.read()
    elif args.code:
        code = args.code
    else:
        code = sys.stdin.read()

    if not code.strip():
        print("no code provided", file=sys.stderr)
        return 2

    try:
        result = call(code, args.host, args.port, args.timeout)
    except (socket.timeout, ConnectionError, OSError, RuntimeError) as exc:
        print("CONNECT FAIL: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
        return 2

    if args.raw:
        print(json.dumps(result, ensure_ascii=False))
    elif result.get("ok"):
        print(result.get("result", ""))
    else:
        print("ERR: %s" % result.get("error"), file=sys.stderr)
        for line in result.get("backtrace", []) or []:
            print("  " + str(line), file=sys.stderr)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
