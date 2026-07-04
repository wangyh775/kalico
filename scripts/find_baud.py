#!/usr/bin/env python3
"""Try to communicate at 115200 after baud rate change, and try to restore if needed."""
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

def read_reg(slave, reg, count):
    payload = struct.pack('>BBHH', slave, 0x03, reg, count)
    return payload + modbus_crc(payload)

def try_read(baud, label, slave=1, reg=0x0000, count=1):
    try:
        ser = serial.Serial('/dev/ttyUSB0', baud, bytesize=8, parity='N', stopbits=1, timeout=0.3)
        time.sleep(0.05)
        frame = read_reg(slave, reg, count)
        ser.write(frame)
        time.sleep(0.1)
        resp = ser.read(256)
        ser.close()
        if resp and len(resp) >= 5 and resp[0] == slave and resp[1] == 0x03:
            val = struct.unpack('>H', resp[3:5])[0]
            print(f"  {label}: RESPONDED! value={val}")
            return True
        elif resp:
            print(f"  {label}: got {len(resp)} bytes: {resp.hex()}")
        else:
            print(f"  {label}: no response")
    except Exception as e:
        print(f"  {label}: error: {e}")
    return False

print("=== Scanning all baud rates to find device ===")
for baud in [115200, 9600, 4800, 19200, 38400]:
    if try_read(baud, f"{baud} baud"):
        break

# Also try 8E1 parity
print("\n=== Trying 115200 with different parity ===")
for parity, pname in [('E', 'Even'), ('N', 'None'), ('O', 'Odd')]:
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity=parity, stopbits=1, timeout=0.3)
        time.sleep(0.05)
        frame = read_reg(1, 0x0000, 1)
        ser.write(frame)
        time.sleep(0.1)
        resp = ser.read(256)
        ser.close()
        if resp and len(resp) >= 5 and resp[0] == 1 and resp[1] == 0x03:
            print(f"  115200 {pname}: RESPONSED!")
            break
        else:
            print(f"  115200 {pname}: {'got ' + resp.hex() if resp else 'no response'}")
    except Exception as e:
        print(f"  115200 {pname}: error: {e}")
