"""Run on K230 to print SHA-256 hashes for the deployed wallet files."""

import uhashlib
import ubinascii


def sha256_file(path, chunk_size=8192):
    digest = uhashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            data = handle.read(chunk_size)
            if not data:
                break
            digest.update(data)
    return ubinascii.hexlify(digest.digest()).decode()


for remote_path in (
    "/sdcard/wallet_yolo_k230d.py",
    "/sdcard/wallet_yolov8n_480.kmodel",
):
    print(remote_path, sha256_file(remote_path))
