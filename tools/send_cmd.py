#!/usr/bin/env python3
# tools/send_cmd.py
# Send velocity commands and print live wheel joint poses from the same port.
# DTR/RTS disabled to avoid resetting the PRIZM so joint poses persist across runs.

import argparse
import struct
import threading
import time
import serial
from cobs import cobs

PKT_CMD_VELOCITY = 0x10
PKT_TEL_WHEELS = 0x21
PKT_TEL_STATUS = 0x22


def crc16_ccitt(data: bytes, crc=0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) & 0xFFFF) ^ 0x1021 if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc


def build_cmd(linear_x_mps: float, angular_z_rps: float, enable: bool) -> bytes:
    lin_mmps = int(round(linear_x_mps * 1000.0))
    ang_mrps = int(round(angular_z_rps * 1000.0))
    payload_wo_crc = struct.pack("<BhhB", PKT_CMD_VELOCITY, lin_mmps, ang_mrps, 1 if enable else 0)
    crc = crc16_ccitt(payload_wo_crc)
    payload = payload_wo_crc + struct.pack("<H", crc)
    return cobs.encode(payload) + b"\x00"


def reader_thread(port: serial.Serial, stop_evt: threading.Event):
    buf = bytearray()
    while not stop_evt.is_set():
        chunk = port.read(256)
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            try:
                idx = buf.index(0)
            except ValueError:
                break
            frame = bytes(buf[:idx])
            del buf[: idx + 1]
            if not frame:
                continue
            try:
                payload = cobs.decode(frame)
            except Exception:
                continue
            if len(payload) < 3:
                continue
            pkt_type = payload[0]
            crc_exp = struct.unpack_from("<H", payload, len(payload) - 2)[0]
            # CRC over [type+fields]
            if crc_exp != crc16_ccitt(payload[:-2]):
                continue
            if pkt_type == PKT_TEL_WHEELS and len(payload) == 7:
                left_deg_mod, right_deg_mod = struct.unpack_from("<HH", payload, 1)
                print(f"[WHEELS] L={left_deg_mod}° R={right_deg_mod}°")
            elif pkt_type == PKT_TEL_STATUS and len(payload) == 9:
                batt_cV, up_ms = struct.unpack_from("<HI", payload, 1)
                print(f"[STATUS] batt={batt_cV / 100.0:.2f}V uptime={up_ms}ms")


def main():
    ap = argparse.ArgumentParser(description="PRIZM PacketSerial sender with live telemetry.")
    ap.add_argument("--port", default="/dev/edubot_prizm")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate_hz", type=float, default=20.0)
    ap.add_argument("--linear_x", type=float, default=None)
    ap.add_argument("--angular_z", type=float, default=None)
    ap.add_argument("--enable", type=int, default=1)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02, dsrdtr=False, rtscts=False, write_timeout=0.2)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    stop_evt = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_evt), daemon=True)
    t.start()

    print(f"Connected to {ser.port} at {args.baud} baud")
    try:
        if args.linear_x is None and args.angular_z is None:
            print("No command specified, just reading telemetry (Ctrl+C to stop)")
            while True:
                time.sleep(1.0)
        else:
            period = 1.0 / max(args.rate_hz, 1e-3)
            print(
                f"Sending linear_x={args.linear_x} m/s, angular_z={args.angular_z} rad/s at {args.rate_hz} Hz (Ctrl+C to stop)"
            )
            while True:
                ser.write(build_cmd(args.linear_x, args.angular_z, bool(args.enable)))
                time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_evt.set()
        t.join(timeout=0.2)
        ser.close()


if __name__ == "__main__":
    main()
