#!/usr/bin/env python3
# tools/telemetry_log.py
# Dedicated telemetry reader for PRIZM PacketSerial (COBS) with CRC verification.

import serial
import struct
from cobs import cobs

PKT_TEL_WHEELS = 0x21
PKT_TEL_STATUS = 0x22


def crc16_ccitt(data: bytes, crc=0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) & 0xFFFF) ^ 0x1021 if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc


def main():
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.02, dsrdtr=False, rtscts=False)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    buf = bytearray()
    print("Listening for telemetry...")
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
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
                if crc_exp != crc16_ccitt(payload[:-2]):
                    continue
                if pkt_type == PKT_TEL_WHEELS and len(payload) == 7:
                    left_deg_mod, right_deg_mod = struct.unpack_from("<HH", payload, 1)
                    print(f"[WHEELS] L={left_deg_mod}° R={right_deg_mod}°")
                elif pkt_type == PKT_TEL_STATUS and len(payload) == 9:
                    batt_cV, up_ms = struct.unpack_from("<HI", payload, 1)
                    print(f"[STATUS] batt={batt_cV / 100.0:.2f}V uptime={up_ms}ms")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
