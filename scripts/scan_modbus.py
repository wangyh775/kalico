#!/usr/bin/env python3
"""Scan SHZK Modbus registers to find actual temperature data."""
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

def build_read_frame(slave_addr, func_code, start_reg, num_regs):
    payload = struct.pack('>BBHH', slave_addr, func_code, start_reg, num_regs)
    return payload + modbus_crc(payload)

def read_regs(ser, slave, func, start, count):
    frame = build_read_frame(slave, func, start, count)
    ser.timeout = 0.5
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.1)
    resp = ser.read(256)
    if resp and len(resp) >= 5 and resp[0] == slave and resp[1] == func:
        data_bytes = resp[2]
        num_regs = data_bytes // 2
        vals = []
        for i in range(num_regs):
            raw = struct.unpack('>H', resp[3 + i*2 : 5 + i*2])[0]
            vals.append(raw)
        return vals
    return None

ser = serial.Serial('/dev/ttyUSB0', 9600, bytesize=8, parity='N', stopbits=1, timeout=0.5)
time.sleep(0.1)

# Scan holding registers (0x03) from 0 to 100 in chunks of 10
print("=== Scanning Holding Registers (0x03) ===")
for start in range(0, 100, 10):
    vals = read_regs(ser, 1, 0x03, start, 10)
    if vals:
        # Show non-30000 values prominently
        interesting = [(i, v) for i, v in enumerate(vals) if v != 30000]
        if interesting:
            print(f"  Reg[{start}-{start+9}]: {vals}  ** INTERESTING: {interesting} **")
        else:
            print(f"  Reg[{start}-{start+9}]: {vals}")
    else:
        print(f"  Reg[{start}-{start+9}]: no response")

# Also try input registers (0x04)
print("\n=== Scanning Input Registers (0x04) ===")
for start in range(0, 100, 10):
    vals = read_regs(ser, 1, 0x04, start, 10)
    if vals:
        interesting = [(i, v) for i, v in enumerate(vals) if v != 0]
        if interesting:
            print(f"  Reg[{start}-{start+9}]: {vals}  ** INTERESTING: {interesting} **")
        else:
            print(f"  Reg[{start}-{start+9}]: all zero")
    else:
        print(f"  Reg[{start}-{start+9}]: no response")

# Try reading just register 0 with different function codes
print("\n=== Trying different slave addresses (func 0x03, reg 0, count 1) ===")
for slave in range(1, 10):
    frame = build_read_frame(slave, 0x03, 0, 1)
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.1)
    resp = ser.read(256)
    if resp and len(resp) >= 5 and resp[0] == slave:
        print(f"  Slave {slave}: responded, data={resp.hex()}")

# Try reading register 0 with count 1 to see the raw response more carefully
print("\n=== Raw response for reg[0], count=1 ===")
frame = build_read_frame(1, 0x03, 0, 1)
ser.reset_input_buffer()
ser.write(frame)
time.sleep(0.1)
resp = ser.read(256)
print(f"  Hex: {resp.hex() if resp else 'none'}")
if resp:
    val = struct.unpack('>H', resp[3:5])[0]
    print(f"  Value: {val} (0x{val:04X})")
    # Try interpreting as signed
    signed = struct.unpack('>h', resp[3:5])[0]
    print(f"  Signed: {signed}")
    # Try as 0.1C
    print(f"  As 0.1C: {signed / 10.0}")
    # Try as 0.01C
    print(f"  As 0.01C: {signed / 100.0}")

ser.close()
print("\nDone.")
