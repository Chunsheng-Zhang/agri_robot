#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time


# ============================================================
# MGX23 Network Configuration
# ============================================================

IP = "172.16.1.250"
PORT = 9000

HEADER = b"fmin"
TAIL = b"ed"

FRAME_LEN = 100


# ============================================================
# Parse one complete NAVI frame
# ============================================================

def parse_frame(frame):

    if len(frame) != FRAME_LEN:
        return None

    # Check header
    if frame[0:4] != HEADER:
        return None

    # Check tail
    if frame[98:100] != TAIL:
        return None

    # --------------------------------------------------------
    # NAVI protocol
    # Little-endian
    # --------------------------------------------------------

    data = {}

    # bytes 5-12
    data["time"] = struct.unpack_from("<d", frame, 4)[0]

    # bytes 13-20
    data["latitude"] = struct.unpack_from("<d", frame, 12)[0]

    # bytes 21-28
    data["longitude"] = struct.unpack_from("<d", frame, 20)[0]

    # bytes 29-36
    data["altitude"] = struct.unpack_from("<d", frame, 28)[0]

    # bytes 37-40
    data["vel_north"] = struct.unpack_from("<f", frame, 36)[0]

    # bytes 41-44
    data["vel_east"] = struct.unpack_from("<f", frame, 40)[0]

    # bytes 45-48
    data["vel_down"] = struct.unpack_from("<f", frame, 44)[0]

    # bytes 49-52
    data["roll"] = struct.unpack_from("<f", frame, 48)[0]

    # bytes 53-56
    data["pitch"] = struct.unpack_from("<f", frame, 52)[0]

    # bytes 57-60
    data["yaw"] = struct.unpack_from("<f", frame, 56)[0]

    # bytes 61-64
    data["accuracy"] = struct.unpack_from("<f", frame, 60)[0]

    # bytes 65-68
    data["acc_bias_x"] = struct.unpack_from("<f", frame, 64)[0]

    # bytes 69-72
    data["acc_bias_y"] = struct.unpack_from("<f", frame, 68)[0]

    # bytes 73-76
    data["acc_bias_z"] = struct.unpack_from("<f", frame, 72)[0]

    # bytes 77-80
    data["gyro_bias_x"] = struct.unpack_from("<f", frame, 76)[0]

    # bytes 81-84
    data["gyro_bias_y"] = struct.unpack_from("<f", frame, 80)[0]

    # bytes 85-88
    data["gyro_bias_z"] = struct.unpack_from("<f", frame, 84)[0]

    # bytes 89-92
    data["temperature"] = struct.unpack_from("<f", frame, 88)[0]

    # bytes 93-96
    data["status"] = struct.unpack_from("<I", frame, 92)[0]

    # bytes 97-98
    data["checksum_received"] = struct.unpack_from("<H", frame, 96)[0]

    # --------------------------------------------------------
    # Checksum
    # Manual: sum bytes 1-96
    # --------------------------------------------------------

    data["checksum_calculated"] = sum(frame[:96]) & 0xFFFF

    data["checksum_ok"] = (
        data["checksum_received"]
        == data["checksum_calculated"]
    )

    return data


# ============================================================
# Print NAVI data
# ============================================================

def print_data(data):

    print()
    print("=" * 58)
    print("NAVI DATA")
    print("=" * 58)

    print(f"Time      : {data['time']:.2f}")

    print()
    print("Position")
    print(f"  Latitude  : {data['latitude']:.10f} deg")
    print(f"  Longitude : {data['longitude']:.10f} deg")
    print(f"  Altitude  : {data['altitude']:.4f} m")

    print()
    print("Velocity")
    print(f"  North     : {data['vel_north']:.4f} m/s")
    print(f"  East      : {data['vel_east']:.4f} m/s")
    print(f"  Down      : {data['vel_down']:.4f} m/s")

    print()
    print("Attitude")
    print(f"  Roll      : {data['roll']:.6f} rad")
    print(f"  Pitch     : {data['pitch']:.6f} rad")
    print(f"  Yaw       : {data['yaw']:.6f} rad")

    print()
    print("Position Accuracy")
    print(f"  Accuracy  : {data['accuracy']:.4f} m")

    print()
    print("Sensor Bias")
    print(f"  Acc X     : {data['acc_bias_x']:.6f} m/s^2")
    print(f"  Acc Y     : {data['acc_bias_y']:.6f} m/s^2")
    print(f"  Acc Z     : {data['acc_bias_z']:.6f} m/s^2")

    print(f"  Gyro X    : {data['gyro_bias_x']:.6f} rad/s")
    print(f"  Gyro Y    : {data['gyro_bias_y']:.6f} rad/s")
    print(f"  Gyro Z    : {data['gyro_bias_z']:.6f} rad/s")

    print()
    print("Device")
    print(f"  Temperature : {data['temperature']:.2f} C")
    print(f"  Status      : 0x{data['status']:08X}")

    print()
    print("Checksum")
    print(f"  Received  : 0x{data['checksum_received']:04X}")
    print(f"  Calculated: 0x{data['checksum_calculated']:04X}")
    print(f"  Result    : {'OK' if data['checksum_ok'] else 'ERROR'}")

    print("=" * 58)


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 58)
    print("MGX23 NAVI TCP Receiver")
    print("=" * 58)
    print(f"Target    : {IP}:{PORT}")
    print(f"Frame     : {FRAME_LEN} bytes")
    print(f"Header    : {HEADER.decode()}")
    print(f"Tail      : {TAIL.decode()}")
    print("=" * 58)

    while True:

        sock = None

        try:

            print()
            print("Connecting...")

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            sock.settimeout(5)

            sock.connect((IP, PORT))

            print("Connected.")

            buffer = b""

            while True:

                data = sock.recv(4096)

                if not data:
                    print("Connection closed by MGX23.")
                    break

                buffer += data

                # ------------------------------------------------
                # Extract complete 100-byte NAVI frames
                # ------------------------------------------------

                while True:

                    # Find fmin header
                    start = buffer.find(HEADER)

                    if start < 0:

                        # Keep only the last few bytes in case
                        # the header is split across TCP packets.
                        buffer = buffer[-3:]
                        break

                    # Remove bytes before header
                    if start > 0:
                        buffer = buffer[start:]

                    # Wait for complete frame
                    if len(buffer) < FRAME_LEN:
                        break

                    frame = buffer[:FRAME_LEN]

                    # ------------------------------------------------
                    # Check frame tail
                    # ------------------------------------------------

                    if frame[98:100] != TAIL:

                        # Header found but frame invalid.
                        # Move forward by one byte and search again.
                        buffer = buffer[1:]
                        continue

                    # Remove this frame from buffer
                    buffer = buffer[FRAME_LEN:]

                    # Parse
                    result = parse_frame(frame)

                    if result is not None:

                        print_data(result)

        except socket.timeout:

            print("Connection timeout.")

        except ConnectionRefusedError:

            print("Connection refused.")

        except ConnectionResetError:

            print("Connection reset by MGX23.")

        except KeyboardInterrupt:

            print()
            print("Program stopped.")
            break

        except Exception as e:

            print(f"Error: {e}")

        finally:

            if sock is not None:

                try:
                    sock.close()
                except Exception:
                    pass

        print()
        print("Reconnect after 2 seconds...")
        time.sleep(2)


if __name__ == "__main__":
    main()