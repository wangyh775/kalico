#!/usr/bin/env python3
"""After baud rate change: scan all baud rates to find device, and try reading reg 0x07D2."""
import serial
import struct
import time

def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def build_frame(slave, func, reg, value):
    payload = struct.pack('>BBHH', slave, func, reg, value)
    return payload + modbus_crc(payload)

def try_read(baud, label, slave=1, reg=0x07D2, count=1):
    try:
        ser = serial.Serial('/dev/ttyUSB0', baud, bytesize=8, parity='N', stopbits=1, timeout=0.3)
        time.sleep(0.05)
        frame = build_frame(slave, 0x03, reg, count)
        ser.write(frame)
        time.sleep(0.1)
        resp = ser.read(256)
        ser.close()
        if resp and len(resp) >= 5 and resp[0] == slave and resp[1] == 0x03:
            val = struct.unpack('>H', resp[3:5])[0]
            print(f"  {label}: RESPONDED! reg 0x{reg:04X} = {val}")
            return True
        else:
            print(f"  {label}: {'got ' + resp.hex() if resp else 'no response'}")
    except Exception as e:
        print(f"  {label}: error: {e}")
    return False

# Scan all baud rates to see where the device is now
print("=== Scanning all baud rates ===")
for baud in [115200, 9600, 57600, 38400, 19200, 4800, 1200]:
    try_read(baud, f"{baud}")

# Also try reading temperature register at 115200
print("\n=== Try reading temp reg 0x0000 at 115200 ===")
try_read(115200, "115200 reg0", slave=1, reg=0x0000, count=1)

# Maybe the baud change takes a moment - wait and retry
print("\n=== Waiting 2s and retrying 115200 ===")
time.sleep(2)
try_read(115200, "115200 after wait")
