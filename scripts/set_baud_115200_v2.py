#!/usr/bin/env python3
"""Set SHZK RS485 baud rate to 115200 via register 0x07D2."""
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

def read_regs(ser, slave, reg, count):
    frame = build_frame(slave, 0x03, reg, count)
    ser.timeout = 0.5
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.1)
    resp = ser.read(256)
    if resp and len(resp) >= 5 and resp[0] == slave and resp[1] == 0x03:
        vals = []
        for i in range(resp[2] // 2):
            vals.append(struct.unpack('>H', resp[3 + i*2 : 5 + i*2])[0])
        return vals
    return None

ser = serial.Serial('/dev/ttyUSB0', 9600, bytesize=8, parity='N', stopbits=1, timeout=0.5)
time.sleep(0.1)

# Read current config registers (0x07D0 = 2000, 0x07D1 = 2001, 0x07D2 = 2002)
print("=== Current config registers ===")
vals = read_regs(ser, 1, 0x07D0, 3)
if vals:
    print(f"  Reg 0x07D0 (device addr): {vals[0]}")
    print(f"  Reg 0x07D1 (RS232 baud):  {vals[1]}")
    print(f"  Reg 0x07D2 (RS485 baud):  {vals[2]}")
else:
    print("  Failed to read config registers")

# Write RS485 baud rate = 8 (115200) to register 0x07D2
print("\n=== Setting RS485 baud to 115200 (write 8 to 0x07D2) ===")
frame = build_frame(1, 0x06, 0x07D2, 8)
print(f"  Command: {frame.hex()}")
ser.write(frame)
time.sleep(0.2)
resp = ser.read(256)
print(f"  Response: {resp.hex() if resp else 'none'}")

# Verify at 115200
print("\n=== Verifying at 115200 baud ===")
ser.close()
time.sleep(0.5)
ser = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity='N', stopbits=1, timeout=0.5)
time.sleep(0.1)

vals = read_regs(ser, 1, 0x07D0, 3)
if vals:
    print(f"  SUCCESS at 115200!")
    print(f"  Reg 0x07D0 (device addr): {vals[0]}")
    print(f"  Reg 0x07D1 (RS232 baud):  {vals[1]}")
    print(f"  Reg 0x07D2 (RS485 baud):  {vals[2]}")

    # Also read temperature to confirm full functionality
    temps = read_regs(ser, 1, 0x0000, 8)
    if temps:
        print(f"  Temperature registers: {temps}")
else:
    print("  No response at 115200")

ser.close()
print("\nDone.")
