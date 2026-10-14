
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import socket
import time
from datetime import datetime
from pathlib import Path

from mgx23_parser import HEADER, FRAME_LEN, TAIL, parse_frame


# ============================================================
# MGX23 配置
# ============================================================

IP = "172.16.1.250"
PORT = 9000

DURATION_SEC = 300          # 记录 5 分钟
RECONNECT_INTERVAL = 2.0    # 断线后每隔 2 秒重试
SOCKET_TIMEOUT = 0.5

# 日志保存目录
LOG_DIR = (
    Path.home()
    / "agri_robot"
    / "mgx23_ws"
    / "data"
    / "mgx23_logs"
)


# ============================================================
# CSV 字段
# ============================================================

CSV_FIELDS = [
    "frame_index",
    "host_time",
    "elapsed_sec",
    "device_time",
    "latitude",
    "longitude",
    "altitude",
    "vel_north",
    "vel_east",
    "vel_down",
    "roll",
    "pitch",
    "yaw",
    "accuracy",
    "acc_bias_x",
    "acc_bias_y",
    "acc_bias_z",
    "gyro_bias_x",
    "gyro_bias_y",
    "gyro_bias_z",
    "temperature",
    "status",
    "status_hex",
    "checksum_received",
    "checksum_calculated",
    "checksum_ok",
    "raw_frame_hex",
]


def main():
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    # 每次运行建立独立文件，避免覆盖之前的记录
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = LOG_DIR / f"mgx23_{session}.csv"
    bin_path = LOG_DIR / f"mgx23_{session}_raw.bin"
    info_path = LOG_DIR / f"mgx23_{session}_summary.txt"

    start_time = time.monotonic()
    frame_count = 0
    checksum_ok_count = 0
    checksum_error_count = 0
    connection_count = 0
    reconnect_count = 0
    last_connect_attempt = 0.0

    buffer = bytearray()
    sock = None

    print("=" * 60)
    print("MGX23 Independent Data Logger")
    print(f"Target   : {IP}:{PORT}")
    print(f"Duration : {DURATION_SEC} seconds")
    print(f"CSV      : {csv_path}")
    print(f"Raw data : {bin_path}")
    print("=" * 60)

    try:
        with (
            open(csv_path, "w", newline="", encoding="utf-8-sig") as csv_file,
            open(bin_path, "wb") as raw_file,
        ):
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            csv_file.flush()

            while time.monotonic() - start_time < DURATION_SEC:

                # ------------------------------------------------
                # 1. 连接设备；断线后自动重连
                # ------------------------------------------------
                if sock is None:
                    now = time.monotonic()

                    if now - last_connect_attempt < RECONNECT_INTERVAL:
                        time.sleep(0.05)
                        continue

                    last_connect_attempt = now
                    candidate = socket.socket(
                        socket.AF_INET, socket.SOCK_STREAM
                    )
                    candidate.settimeout(SOCKET_TIMEOUT)

                    try:
                        candidate.connect((IP, PORT))
                        sock = candidate
                        connection_count += 1

                        if connection_count > 1:
                            reconnect_count += 1

                        buffer.clear()
                        print(
                            f"[CONNECTED] {IP}:{PORT} "
                            f"at {datetime.now().isoformat(timespec='milliseconds')}"
                        )

                    except OSError as exc:
                        candidate.close()
                        print(f"[CONNECT FAILED] {exc}")
                        continue

                # ------------------------------------------------
                # 2. 接收 TCP 字节流
                # 原始文件保存收到的全部字节，不只保存正确帧
                # ------------------------------------------------
                try:
                    chunk = sock.recv(4096)

                    if not chunk:
                        print("[DISCONNECTED] Device closed connection")
                        sock.close()
                        sock = None
                        buffer.clear()
                        continue

                    raw_file.write(chunk)
                    raw_file.flush()
                    buffer.extend(chunk)

                except socket.timeout:
                    continue

                except OSError as exc:
                    print(f"[RECEIVE ERROR] {exc}")
                    sock.close()
                    sock = None
                    buffer.clear()
                    continue

                # ------------------------------------------------
                # 3. 从 TCP 缓存中提取完整数据帧
                # ------------------------------------------------
                while True:
                    header_index = buffer.find(HEADER)

                    if header_index < 0:
                        # 保留可能被 TCP 拆开的帧头
                        if len(buffer) > len(HEADER) - 1:
                            del buffer[:- (len(HEADER) - 1)]
                        break

                    if header_index > 0:
                        del buffer[:header_index]

                    if len(buffer) < FRAME_LEN:
                        break

                    if buffer[FRAME_LEN - 2:FRAME_LEN] != TAIL:
                        # 帧尾错误，逐字节重新寻找帧头
                        del buffer[0]
                        continue

                    frame = bytes(buffer[:FRAME_LEN])
                    del buffer[:FRAME_LEN]

                    data = parse_frame(frame)

                    if data is None:
                        continue

                    frame_count += 1
                    if data["checksum_ok"]:
                        checksum_ok_count += 1
                    else:
                        checksum_error_count += 1

                    elapsed = time.monotonic() - start_time

                    row = {
                        "frame_index": frame_count,
                        "host_time": datetime.now().isoformat(
                            timespec="milliseconds"
                        ),
                        "elapsed_sec": f"{elapsed:.6f}",
                        "device_time": data["time"],
                        "latitude": data["latitude"],
                        "longitude": data["longitude"],
                        "altitude": data["altitude"],
                        "vel_north": data["vel_north"],
                        "vel_east": data["vel_east"],
                        "vel_down": data["vel_down"],
                        "roll": data["roll"],
                        "pitch": data["pitch"],
                        "yaw": data["yaw"],
                        "accuracy": data["accuracy"],
                        "acc_bias_x": data["acc_bias_x"],
                        "acc_bias_y": data["acc_bias_y"],
                        "acc_bias_z": data["acc_bias_z"],
                        "gyro_bias_x": data["gyro_bias_x"],
                        "gyro_bias_y": data["gyro_bias_y"],
                        "gyro_bias_z": data["gyro_bias_z"],
                        "temperature": data["temperature"],
                        "status": data["status"],
                        "status_hex": f"0x{data['status']:08X}",
                        "checksum_received": data["checksum_received"],
                        "checksum_calculated": data["checksum_calculated"],
                        "checksum_ok": data["checksum_ok"],
                        "raw_frame_hex": frame.hex(),
                    }

                    writer.writerow(row)
                    csv_file.flush()

                    if frame_count % 50 == 0:
                        print(
                            f"[RECORDING] elapsed={elapsed:.1f}s, "
                            f"frames={frame_count}, "
                            f"checksum_errors={checksum_error_count}"
                        )

    except KeyboardInterrupt:
        print("\n[STOPPED] Interrupted by user")

    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

        elapsed_total = time.monotonic() - start_time

        checksum_rate = (
            checksum_ok_count / frame_count * 100.0
            if frame_count else 0.0
        )

        summary = [
            "MGX23 Recording Summary",
            f"Target: {IP}:{PORT}",
            f"Start time: {session}",
            f"Elapsed seconds: {elapsed_total:.3f}",
            f"Total parsed frames: {frame_count}",
            f"Checksum OK: {checksum_ok_count}",
            f"Checksum errors: {checksum_error_count}",
            f"Checksum pass rate (%): {checksum_rate:.3f}",
            f"TCP connections: {connection_count}",
            f"Reconnects: {reconnect_count}",
            f"CSV file: {csv_path}",
            f"Raw stream file: {bin_path}",
        ]

        info_path.write_text("\n".join(summary) + "\n", encoding="utf-8")

        print("\n" + "\n".join(summary))
        print(f"Summary file: {info_path}")


if __name__ == "__main__":
    main()

