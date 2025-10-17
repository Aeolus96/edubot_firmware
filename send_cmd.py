#!/usr/bin/env python3
# tools/send_cmd.py
# Send framed CBOR {1: linear_x_mmps, 2: angular_z_mrps, 3: enable (0/1)} (enable optional, inferred from non-zero).

import argparse
import struct
import time
import serial
import cbor2

SYNC = b"\xaa\x55"
MSG_TYPE_COMMAND = 0x01


def crc16_ccitt(data: bytes, crc=0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_command(linear_x_mps: float, angular_z_rps: float, enable: bool = True) -> bytes:
    scale_vel = 1000.0
    payload = {
        1: int(round(linear_x_mps * scale_vel)),  # mm/s
        2: int(round(angular_z_rps * scale_vel)),  # milli-rad/s
    }
    if enable:  # Optional; inferred if omitted or non-zero vels
        payload[3] = int(1 if enable else 0)

    cbor_payload = cbor2.dumps(payload)
    header = bytes([MSG_TYPE_COMMAND]) + struct.pack("<H", len(cbor_payload))
    crc = crc16_ccitt(header + cbor_payload)
    frame = SYNC + header + cbor_payload + struct.pack("<H", crc)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=str, default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate_hz", type=float, default=20.0)
    ap.add_argument("--enable", type=int, default=1)
    ap.add_argument("--linear_x", type=float, default=0.2, help="m/s")
    ap.add_argument("--angular_z", type=float, default=0.0, help="rad/s")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    period = 1.0 / max(args.rate_hz, 1e-3)

    try:
        while True:
            frame = build_command(args.linear_x, args.angular_z, bool(args.enable))
            ser.write(frame)
            time.sleep(period)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
